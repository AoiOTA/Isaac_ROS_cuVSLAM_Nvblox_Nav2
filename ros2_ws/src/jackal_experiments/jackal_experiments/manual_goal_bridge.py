"""Forward RViz ``2D Goal Pose`` messages to a readiness-gated Nav2 action."""

from __future__ import annotations

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class ManualGoalBridge(Node):
    """Turn the latest ``/goal_pose`` message into a Nav2 action goal.

    RViz's standard 2D Goal Pose tool publishes a ``PoseStamped`` rather than
    calling a Nav2 action. A newly clicked pose cancels the active manual goal
    and is sent as soon as cancellation completes. Goals remain queued until
    cuVGL has anchored the current cuVSLAM odometry in the saved map.
    """

    def __init__(self) -> None:
        super().__init__("manual_goal_bridge")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("action_topic", "/navigate_to_pose_resilient")
        self.declare_parameter("required_frame", "map")
        self.declare_parameter("ready_topic", "/localization/ready")
        self.declare_parameter("require_ready", True)

        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.action_topic = str(self.get_parameter("action_topic").value)
        self.required_frame = str(self.get_parameter("required_frame").value)
        self.ready_topic = str(self.get_parameter("ready_topic").value)
        self.require_ready = bool(self.get_parameter("require_ready").value)

        self.client = ActionClient(self, NavigateToPose, self.action_topic)
        self.subscription = self.create_subscription(
            PoseStamped, self.goal_topic, self.on_goal_pose, 10
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_subscription = self.create_subscription(
            Bool, self.ready_topic, self.on_ready, latched
        )
        self.pending: PoseStamped | None = None
        self.active_handle = None
        self.active_message: PoseStamped | None = None
        self.sent_message: PoseStamped | None = None
        self.send_in_flight = False
        self.cancel_requested = False
        self.ready = not self.require_ready
        self.generation = 0
        self.sent_generation = 0
        self.timer = self.create_timer(0.5, self.pump)
        self.get_logger().info(
            f"Waiting for RViz 2D Goal Pose on {self.goal_topic}; "
            f"forwarding to {self.action_topic}"
        )

    def on_ready(self, message: Bool) -> None:
        was_ready = self.ready
        self.ready = bool(message.data) or not self.require_ready
        if self.ready != was_ready:
            self.get_logger().info(f"Manual navigation localization-ready={self.ready}")
        if not self.ready and self.active_handle is not None:
            if self.pending is None and self.active_message is not None:
                self.pending = self.active_message
            self.cancel_active("Localization became unready; pausing the manual goal")
        elif self.ready:
            self.pump()

    def cancel_active(self, reason: str) -> None:
        if self.active_handle is None or self.cancel_requested:
            return
        self.cancel_requested = True
        self.get_logger().info(reason)
        self.active_handle.cancel_goal_async()

    def valid_pose(self, message: PoseStamped) -> bool:
        frame = message.header.frame_id.lstrip("/")
        required = self.required_frame.lstrip("/")
        if frame != required:
            self.get_logger().error(
                f"Ignoring manual goal in frame '{message.header.frame_id}'; "
                f"RViz Fixed Frame must be '{self.required_frame}'"
            )
            return False
        values = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("Ignoring manual goal with non-finite pose values")
            return False
        return True

    def on_goal_pose(self, message: PoseStamped) -> None:
        if not self.valid_pose(message):
            return
        self.generation += 1
        self.pending = message
        self.get_logger().info(
            "Received RViz manual goal "
            f"#{self.generation}: x={message.pose.position.x:.3f}, "
            f"y={message.pose.position.y:.3f}"
        )
        if not self.ready:
            self.get_logger().info(
                "Goal queued until cuVGL/cuvSLAM localization is navigation-ready"
            )
        self.pump()

    def pump(self) -> None:
        if self.pending is None:
            return
        if self.active_handle is not None:
            self.cancel_active(
                "Canceling the active manual goal before sending the newer pose"
            )
            return
        if self.send_in_flight:
            return
        if not self.ready:
            return
        if not self.client.server_is_ready():
            return

        message = self.pending
        self.pending = None
        self.sent_generation = self.generation
        goal = NavigateToPose.Goal()
        goal.pose = message
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.send_in_flight = True
        self.sent_message = message
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future: object) -> None:
        self.send_in_flight = False
        handle = future.result()
        sent_message = self.sent_message
        self.sent_message = None
        if handle is None or not handle.accepted:
            self.get_logger().error("The navigation server rejected the manual goal")
            self.pump()
            return
        self.active_handle = handle
        self.active_message = sent_message
        self.cancel_requested = False
        self.get_logger().info(f"Manual goal #{self.sent_generation} accepted")
        result = handle.get_result_async()
        result.add_done_callback(self.on_result)
        if not self.ready:
            if self.pending is None and self.active_message is not None:
                self.pending = self.active_message
            self.cancel_active("Localization is not ready; canceling the accepted goal")
        elif self.pending is not None:
            self.pump()

    def on_feedback(self, message: object) -> None:
        feedback = message.feedback
        self.get_logger().debug(
            f"Manual goal distance remaining: {feedback.distance_remaining:.3f} m"
        )

    def on_result(self, future: object) -> None:
        wrapped = future.result()
        self.active_handle = None
        self.active_message = None
        self.cancel_requested = False
        if wrapped is None:
            self.get_logger().error("Manual goal completed without an action result")
        elif wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Manual goal reached successfully")
        elif wrapped.status == GoalStatus.STATUS_CANCELED and self.pending is not None:
            self.get_logger().info("Previous manual goal canceled; sending newer pose")
        else:
            self.get_logger().error(
                "Manual goal did not succeed: "
                f"status={wrapped.status}, error_code={wrapped.result.error_code}, "
                f"detail={wrapped.result.error_msg}"
            )
        self.pump()

    def destroy_node(self) -> None:
        self.client.destroy()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ManualGoalBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
