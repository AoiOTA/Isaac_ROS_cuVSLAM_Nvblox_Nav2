from jackal_teleop.safety import DeadmanCommand
import pytest

from jackal_teleop.keyboard_teleop import KeyboardTeleop, speed_for_level


def test_motion_key_and_deadman_stop() -> None:
    state = DeadmanCommand(timeout_s=0.18, linear_speed=0.55, angular_speed=1.0)
    assert state.apply("w", 1.0)
    assert state.linear == 0.55
    assert not state.update(1.18)
    assert state.update(1.181)
    assert state.linear == state.angular == 0.0


def test_space_stops_immediately() -> None:
    state = DeadmanCommand()
    state.apply("a", 1.0)
    assert state.apply(" ", 1.01)
    assert state.linear == state.angular == 0.0


def test_mapping_speed_levels_are_faster_by_default_and_bounded() -> None:
    assert speed_for_level(0.55, 1.0, 3) == (0.55, 1.0)
    assert speed_for_level(0.55, 1.0, 5) == pytest.approx((0.7425, 1.35))
    with pytest.raises(ValueError, match="speed_level"):
        speed_for_level(0.55, 1.0, 6)


def test_speed_change_updates_next_motion_command() -> None:
    state = DeadmanCommand()
    state.set_speeds(0.7425, 1.35)
    state.apply("d", 1.0)
    assert state.linear == 0.0
    assert state.angular == -1.35


def test_speed_level_change_stops_before_applying_new_limit() -> None:
    class Logger:
        def info(self, _message: str) -> None:
            pass

    node = object.__new__(KeyboardTeleop)
    node.base_linear_speed = 0.55
    node.base_angular_speed = 1.0
    node.speed_level = 3
    node.state = DeadmanCommand()
    node.state.apply("w", 1.0)
    node.get_logger = lambda: Logger()
    assert node.set_speed_level(5)
    assert node.state.linear == node.state.angular == 0.0
    assert node.state.linear_speed == pytest.approx(0.7425)
    assert node.state.angular_speed == pytest.approx(1.35)


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
