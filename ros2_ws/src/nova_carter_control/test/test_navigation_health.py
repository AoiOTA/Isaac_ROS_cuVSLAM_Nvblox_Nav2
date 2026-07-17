from nova_carter_control.command_guard import CommandGuard


def guard() -> CommandGuard:
    node = CommandGuard.__new__(CommandGuard)
    node.require_navigation_health = True
    node.localization_ready = True
    node.visual_slam_tracking = True
    node.last_visual_slam_ns = 9_800_000_000
    node.last_depth_ns = 9_900_000_000
    node.last_map_slice_ns = 9_600_000_000
    node.visual_slam_timeout = 0.75
    node.depth_timeout = 0.35
    node.map_slice_timeout = 0.75
    return node


def test_navigation_health_accepts_fresh_complete_dataflow() -> None:
    assert guard().navigation_health_failure(10_000_000_000) is None


def test_navigation_health_identifies_each_safety_failure() -> None:
    node = guard()
    node.localization_ready = False
    assert node.navigation_health_failure(10_000_000_000) == "localization_not_ready"
    node = guard()
    node.visual_slam_tracking = False
    assert node.navigation_health_failure(10_000_000_000) == "visual_slam_not_tracking"
    node = guard()
    node.last_depth_ns = 9_000_000_000
    assert node.navigation_health_failure(10_000_000_000) == "depth_stale"
    node = guard()
    node.last_map_slice_ns = None
    assert node.navigation_health_failure(10_000_000_000) == "map_slice_missing"
