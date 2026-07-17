"""Safe cuVSLAM-loss -> cuVGL -> tracking recovery state machine."""

from __future__ import annotations

import json
import time

from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class RecoveryState:
    CAMERA_WARMUP = "camera_warmup"
    TRIGGERING_VGL = "triggering_vgl"
    WAITING_FOR_VGL_POSE = "waiting_for_vgl_pose"
    WAITING_FOR_TRACKING = "waiting_for_tracking"
    NAVIGATION_READY = "navigation_ready"
    FAILED_SAFE = "failed_safe"


class LocalizationRecoveryManager(Node):
    """Own localization readiness and keep the robot stopped during recovery."""

    def __init__(self) -> None:
        super().__init__("localization_recovery_manager")
        self.declare_parameter("max_trigger_attempts", 3)
        self.declare_parameter("camera_warmup_s", 0.5)
        self.declare_parameter("obstruction_clearance_s", 4.0)
        self.declare_parameter("retry_period_s", 1.0)
        self.declare_parameter("vgl_pose_timeout_s", 8.0)
        self.declare_parameter("tracking_timeout_s", 8.0)
        self.declare_parameter("status_timeout_s", 1.0)
        self.declare_parameter("tracking_samples_required", 20)
        self.declare_parameter("enable_surround_cameras", False)
        self.max_attempts = int(self.get_parameter("max_trigger_attempts").value)
        self.camera_warmup = float(self.get_parameter("camera_warmup_s").value)
        self.obstruction_clearance = float(
            self.get_parameter("obstruction_clearance_s").value
        )
        self.retry_period = float(self.get_parameter("retry_period_s").value)
        self.pose_timeout = float(self.get_parameter("vgl_pose_timeout_s").value)
        self.tracking_timeout = float(self.get_parameter("tracking_timeout_s").value)
        self.status_timeout = float(self.get_parameter("status_timeout_s").value)
        self.required_tracking = int(
            self.get_parameter("tracking_samples_required").value
        )
        self.enable_surround = bool(
            self.get_parameter("enable_surround_cameras").value
        )
        positive = (
            self.max_attempts,
            self.camera_warmup,
            self.obstruction_clearance,
            self.retry_period,
            self.pose_timeout,
            self.tracking_timeout,
            self.status_timeout,
            self.required_tracking,
        )
        if any(value <= 0 for value in positive):
            raise ValueError("all localization recovery limits must be positive")

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.ready_publisher = self.create_publisher(
            Bool, "/localization/ready", latched
        )
        self.camera_publisher = self.create_publisher(
            Bool, "/vgl/cameras_enabled", latched
        )
        self.state_publisher = self.create_publisher(
            String, "/localization/recovery_state", latched
        )
        self.create_subscription(
            VisualSlamStatus, "/visual_slam/status", self.on_status, reliable
        )
        self.create_subscription(
            Bool, "/vgl_pose_relay/accepted", self.on_relay, reliable
        )
        self.trigger_client = self.create_client(
            Trigger, "/visual_localization/trigger_localization"
        )
        self.force_service = self.create_service(
            Trigger, "/localization/force_relocalization", self.on_force
        )

        self.state = RecoveryState.CAMERA_WARMUP
        self.ready = False
        self.cameras_enabled = self.enable_surround
        self.reason = "startup"
        self.failure_reason = ""
        self.transition_sequence = 0
        self.recovery_count = 0
        self.attempts = 0
        self.relay_accepted = False
        self.consecutive_tracking = 0
        self.last_status_wall: float | None = None
        self.state_started_wall = time.monotonic()
        self.next_action_wall = self.state_started_wall + self.camera_warmup
        self.pending_trigger = None
        self.last_heartbeat_wall = 0.0
        self.publish_outputs(force=True)
        self.timer = self.create_timer(
            0.05, self.on_timer, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def state_payload(self) -> dict[str, object]:
        return {
            "state": self.state,
            "navigation_ready": self.ready,
            "surround_cameras_enabled": self.cameras_enabled,
            "attempt": self.attempts,
            "max_attempts": self.max_attempts,
            "recovery_count": self.recovery_count,
            "reason": self.reason,
            "failure_reason": self.failure_reason,
            "transition_sequence": self.transition_sequence,
        }

    def publish_outputs(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_heartbeat_wall < 1.0:
            return
        self.ready_publisher.publish(Bool(data=self.ready))
        self.camera_publisher.publish(Bool(data=self.cameras_enabled))
        self.state_publisher.publish(
            String(data=json.dumps(self.state_payload(), sort_keys=True))
        )
        self.last_heartbeat_wall = now

    def transition(self, state: str) -> None:
        if state == self.state:
            return
        previous = self.state
        self.state = state
        self.state_started_wall = time.monotonic()
        self.transition_sequence += 1
        self.publish_outputs(force=True)
        self.get_logger().info(
            f"Localization recovery {previous} -> {state}: "
            f"{json.dumps(self.state_payload(), sort_keys=True)}"
        )

    def begin_cycle(self, reason: str, *, count_recovery: bool) -> None:
        if count_recovery:
            self.recovery_count += 1
        self.reason = reason
        self.failure_reason = ""
        self.ready = False
        self.cameras_enabled = self.enable_surround
        self.attempts = 0
        self.relay_accepted = False
        self.consecutive_tracking = 0
        self.pending_trigger = None
        wait_s = self.obstruction_clearance if count_recovery else self.camera_warmup
        self.next_action_wall = time.monotonic() + wait_s
        self.transition(RecoveryState.CAMERA_WARMUP)
        self.publish_outputs(force=True)

    def schedule_retry(self, reason: str) -> None:
        if self.attempts >= self.max_attempts:
            self.failure_reason = reason
            self.ready = False
            self.cameras_enabled = self.enable_surround
            self.transition(RecoveryState.FAILED_SAFE)
            self.publish_outputs(force=True)
            self.get_logger().error(
                f"Localization recovery exhausted {self.max_attempts} attempts: {reason}"
            )
            return
        self.relay_accepted = False
        self.consecutive_tracking = 0
        self.pending_trigger = None
        self.next_action_wall = time.monotonic() + self.retry_period
        self.transition(RecoveryState.CAMERA_WARMUP)

    def on_force(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.begin_cycle("forced_relocalization", count_recovery=True)
        response.success = True
        response.message = "safe relocalization cycle started"
        return response

    def on_relay(self, message: Bool) -> None:
        if not message.data:
            if self.state == RecoveryState.WAITING_FOR_VGL_POSE:
                self.schedule_retry("cuVGL pose rejected by innovation gate")
            return
        if self.state == RecoveryState.NAVIGATION_READY:
            return
        self.relay_accepted = True
        self.consecutive_tracking = 0
        self.transition(RecoveryState.WAITING_FOR_TRACKING)

    def on_status(self, message: VisualSlamStatus) -> None:
        self.last_status_wall = time.monotonic()
        tracking = int(message.vo_state) == 1
        if tracking:
            self.consecutive_tracking += 1
        else:
            self.consecutive_tracking = 0
            if self.ready or self.state == RecoveryState.NAVIGATION_READY:
                # Guard receives this same unhealthy status and also stops
                # immediately; readiness is withdrawn here in the same callback.
                self.begin_cycle("visual_slam_tracking_lost", count_recovery=True)
                return
        if (
            self.relay_accepted
            and tracking
            and self.consecutive_tracking >= self.required_tracking
        ):
            self.ready = True
            self.cameras_enabled = False
            self.failure_reason = ""
            self.transition(RecoveryState.NAVIGATION_READY)
            self.publish_outputs(force=True)

    def request_vgl(self) -> None:
        if self.attempts >= self.max_attempts:
            self.schedule_retry("attempt limit reached before trigger")
            return
        if not self.trigger_client.wait_for_service(timeout_sec=0.02):
            return
        self.attempts += 1
        self.pending_trigger = self.trigger_client.call_async(Trigger.Request())
        self.transition(RecoveryState.TRIGGERING_VGL)
        self.get_logger().info(
            f"Triggered {'four-direction' if self.enable_surround else 'front-stereo'} "
            f"cuVGL attempt {self.attempts}/{self.max_attempts}"
        )

    def on_timer(self) -> None:
        now = time.monotonic()
        self.publish_outputs()
        if (
            self.state == RecoveryState.NAVIGATION_READY
            and self.last_status_wall is not None
            and now - self.last_status_wall > self.status_timeout
        ):
            self.begin_cycle("visual_slam_status_stale", count_recovery=True)
            return
        if self.state == RecoveryState.CAMERA_WARMUP:
            if now >= self.next_action_wall:
                self.request_vgl()
            return
        if self.state == RecoveryState.TRIGGERING_VGL:
            if self.pending_trigger is None or not self.pending_trigger.done():
                return
            response = self.pending_trigger.result()
            self.pending_trigger = None
            if response is None or not response.success:
                self.schedule_retry("cuVGL trigger service rejected request")
                return
            self.transition(RecoveryState.WAITING_FOR_VGL_POSE)
            return
        if (
            self.state == RecoveryState.WAITING_FOR_VGL_POSE
            and now - self.state_started_wall > self.pose_timeout
        ):
            self.schedule_retry("timed out waiting for cuVGL pose")
            return
        if (
            self.state == RecoveryState.WAITING_FOR_TRACKING
            and now - self.state_started_wall > self.tracking_timeout
        ):
            self.schedule_retry("cuVSLAM did not regain stable tracking")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LocalizationRecoveryManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.ready = False
            node.cameras_enabled = False
            node.publish_outputs(force=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
