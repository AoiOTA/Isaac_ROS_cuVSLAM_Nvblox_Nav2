from jackal_teleop.safety import DeadmanCommand
from jackal_teleop.keyboard_teleop import KeyboardTeleop


def test_motion_key_and_deadman_stop() -> None:
    state = DeadmanCommand(timeout_s=0.18, linear_speed=0.35, angular_speed=0.8)
    assert state.apply("w", 1.0)
    assert state.linear == 0.35
    assert not state.update(1.18)
    assert state.update(1.181)
    assert state.linear == state.angular == 0.0


def test_space_stops_immediately() -> None:
    state = DeadmanCommand()
    state.apply("a", 1.0)
    assert state.apply(" ", 1.01)
    assert state.linear == state.angular == 0.0


def test_shutdown_stop_does_not_publish_after_rcl_context_closes(monkeypatch) -> None:
    node = object.__new__(KeyboardTeleop)
    node.state = DeadmanCommand()
    node.state.apply("w", 1.0)
    published = []
    node.publish = lambda: published.append(True)
    monkeypatch.setattr("jackal_teleop.keyboard_teleop.rclpy.ok", lambda: False)
    node.stop_now()
    assert node.state.linear == node.state.angular == 0.0
    assert published == []
