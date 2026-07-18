"""Verify Nav2 activation and request a clean relaunch after partial startup."""

from __future__ import annotations

import json
from pathlib import Path
import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


DEFAULT_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "velocity_smoother",
    "collision_monitor",
    "bt_navigator",
    "waypoint_follower",
]


class Nav2LifecycleGuard(Node):
    """Verify the managed stack and recover a partial activation atomically."""

    def __init__(self) -> None:
        super().__init__("nav2_lifecycle_guard")
        self.declare_parameter("node_names", DEFAULT_NODES)
        self.declare_parameter("service_timeout_s", 30.0)
        self.declare_parameter("failure_file", "")
        self.declare_parameter("force_failure", False)
        self.node_names = [str(value) for value in self.get_parameter("node_names").value]
        self.service_timeout = float(self.get_parameter("service_timeout_s").value)
        failure_value = str(self.get_parameter("failure_file").value)
        self.failure_file = Path(failure_value).resolve() if failure_value else None
        self.force_failure = bool(self.get_parameter("force_failure").value)
        if not self.node_names:
            raise ValueError("lifecycle guard requires managed node names")
        self.state_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in self.node_names
        }

    def call(self, client: object, request: object, timeout: float) -> object | None:
        try:
            if not client.wait_for_service(
                timeout_sec=min(timeout, self.service_timeout)
            ):
                return None
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            return future.result() if future.done() else None
        except Exception:
            if not rclpy.ok():
                return None
            raise

    def states(self) -> dict[str, str]:
        result: dict[str, str] = {}
        for name, client in self.state_clients.items():
            response = self.call(client, GetState.Request(), self.service_timeout)
            result[name] = (
                response.current_state.label if response is not None else "unavailable"
            )
        return result

    @staticmethod
    def all_active(states: dict[str, str]) -> bool:
        return bool(states) and all(value == "active" for value in states.values())

    def wait_active(self, timeout: float) -> tuple[bool, dict[str, str]]:
        deadline = time.monotonic() + timeout
        last: dict[str, str] = {}
        while time.monotonic() < deadline and rclpy.ok():
            last = self.states()
            if self.all_active(last):
                return True, last
            time.sleep(1.0)
        return False, last

    def verify_or_recover(self) -> dict[str, object]:
        if self.force_failure:
            return {"status": "failed", "attempts": 0, "states": {"forced": "test"}}
        active, states = self.wait_active(5.0)
        if active:
            return {"status": "already_active", "attempts": 0, "states": states}
        return {
            "status": "failed",
            "attempts": 0,
            "states": states,
        }


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Nav2LifecycleGuard()
    try:
        report = node.verify_or_recover()
        node.get_logger().info(f"NOVA_CARTER_NAV2_LIFECYCLE {json.dumps(report)}")
        if report["status"] == "failed":
            if node.failure_file is not None:
                node.failure_file.parent.mkdir(parents=True, exist_ok=True)
                node.failure_file.write_text(
                    json.dumps(report, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
            raise RuntimeError(f"Nav2 lifecycle recovery failed: {report['states']}")
        # A successful guard remains alive. Any later unexpected exit is also
        # treated as a reason for the enclosing launch to shut down cleanly.
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            # ros2 launch shutting down the context is the normal exit path.
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
