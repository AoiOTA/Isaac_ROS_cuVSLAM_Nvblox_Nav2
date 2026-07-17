"""NavigateToPose proxy that pauses, relocalizes, and resumes the same goal."""

from __future__ import annotations

import json
import threading
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class ResilientNavigation(Node):
    """Expose ``/navigate_to_pose_resilient`` over the stock Nav2 action."""

    def __init__(self) -> None:
        super().__init__("resilient_navigation")
        self.declare_parameter("max_resume_attempts", 3)
        self.declare_parameter("recovery_timeout_s", 40.0)
        self.declare_parameter("goal_timeout_s", 300.0)
        self.max_resume_attempts = int(
            self.get_parameter("max_resume_attempts").value
        )
        self.recovery_timeout = float(
            self.get_parameter("recovery_timeout_s").value
        )
        self.goal_timeout = float(self.get_parameter("goal_timeout_s").value)
        if min(
            self.max_resume_attempts, self.recovery_timeout, self.goal_timeout
        ) <= 0:
            raise ValueError("resilient navigation limits must be positive")

        self.callback_group = ReentrantCallbackGroup()
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready = False
        self.active = False
        self.active_lock = threading.Lock()
        self.current_downstream = None
        self.status_publisher = self.create_publisher(
            String, "/navigation/resilient_status", latched
        )
        self.create_subscription(
            Bool,
            "/localization/ready",
            self.on_ready,
            latched,
            callback_group=self.callback_group,
        )
        self.client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=self.callback_group,
        )
        self.server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose_resilient",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.callback_group,
        )
        self.publish_status("idle", 0, "")

    def on_ready(self, message: Bool) -> None:
        self.ready = message.data

    def on_goal(self, _request: NavigateToPose.Goal) -> GoalResponse:
        with self.active_lock:
            if self.active:
                return GoalResponse.REJECT
            self.active = True
        return GoalResponse.ACCEPT

    def on_cancel(self, _goal_handle: object) -> CancelResponse:
        downstream = self.current_downstream
        if downstream is not None:
            downstream.cancel_goal_async()
        return CancelResponse.ACCEPT

    def publish_status(self, state: str, resumes: int, detail: str) -> None:
        self.status_publisher.publish(
            String(
                data=json.dumps(
                    {"state": state, "resume_count": resumes, "detail": detail},
                    sort_keys=True,
                )
            )
        )

    @staticmethod
    def wait_future(future: object, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.02)
        return future.done()

    def wait_localization(self, goal_handle: object, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            if self.ready:
                return True
            time.sleep(0.02)
        return False

    def downstream_feedback(self, goal_handle: object, message: object) -> None:
        if goal_handle.is_active:
            goal_handle.publish_feedback(message.feedback)

    def failure_result(self, detail: str) -> NavigateToPose.Result:
        result = NavigateToPose.Result()
        if hasattr(NavigateToPose.Result, "UNKNOWN"):
            result.error_code = NavigateToPose.Result.UNKNOWN
        result.error_msg = detail
        return result

    def execute(self, goal_handle: object) -> NavigateToPose.Result:
        started = time.monotonic()
        resumes = 0
        try:
            while resumes <= self.max_resume_attempts:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self.failure_result("goal canceled by caller")
                self.publish_status("waiting_for_localization", resumes, "")
                if not self.wait_localization(goal_handle, self.recovery_timeout):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return self.failure_result("goal canceled during recovery")
                    goal_handle.abort()
                    return self.failure_result("localization recovery timed out")
                if time.monotonic() - started > self.goal_timeout:
                    goal_handle.abort()
                    return self.failure_result("resilient goal exceeded total timeout")
                if not self.client.wait_for_server(timeout_sec=2.0):
                    time.sleep(0.25)
                    continue

                goal = goal_handle.request
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                self.publish_status("forwarding_goal", resumes, "")
                send = self.client.send_goal_async(
                    goal,
                    feedback_callback=lambda message: self.downstream_feedback(
                        goal_handle, message
                    ),
                )
                if not self.wait_future(send, 10.0):
                    resumes += 1
                    self.publish_status("send_timeout", resumes, "")
                    continue
                downstream = send.result()
                if downstream is None or not downstream.accepted:
                    goal_handle.abort()
                    return self.failure_result("Nav2 rejected forwarded goal")
                self.current_downstream = downstream
                result_future = downstream.get_result_async()
                self.publish_status("navigating", resumes, "")

                interrupted = False
                while not result_future.done():
                    if goal_handle.is_cancel_requested:
                        downstream.cancel_goal_async()
                        goal_handle.canceled()
                        return self.failure_result("goal canceled by caller")
                    if not self.ready:
                        cancel = downstream.cancel_goal_async()
                        self.wait_future(cancel, 3.0)
                        interrupted = True
                        resumes += 1
                        self.publish_status(
                            "paused_for_relocalization", resumes, "localization not ready"
                        )
                        break
                    if time.monotonic() - started > self.goal_timeout:
                        downstream.cancel_goal_async()
                        goal_handle.abort()
                        return self.failure_result("resilient goal exceeded total timeout")
                    time.sleep(0.02)
                self.current_downstream = None
                if interrupted:
                    continue

                wrapped = result_future.result()
                if wrapped is None:
                    resumes += 1
                    continue
                if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                    goal_handle.succeed()
                    self.publish_status("succeeded", resumes, "")
                    return wrapped.result
                # A transform failure may race readiness withdrawal.  Retry only
                # while recovery is active; ordinary planner failures propagate.
                if not self.ready:
                    resumes += 1
                    continue
                goal_handle.abort()
                self.publish_status(
                    "nav2_failed", resumes, f"status={wrapped.status}"
                )
                return wrapped.result

            goal_handle.abort()
            return self.failure_result(
                f"exhausted {self.max_resume_attempts} resume attempts"
            )
        finally:
            self.current_downstream = None
            with self.active_lock:
                self.active = False

    def destroy_node(self) -> None:
        self.server.destroy()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ResilientNavigation()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
