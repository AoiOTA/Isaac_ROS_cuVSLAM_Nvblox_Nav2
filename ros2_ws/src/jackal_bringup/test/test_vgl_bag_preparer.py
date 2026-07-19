import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "tools"))

from prepare_vgl_sensor_bag import (  # noqa: E402
    PoseSample,
    SynchronizedGroup,
    maximum_gap_seconds,
    select_keyframe_groups,
    synchronize_stamps,
)


def quaternion(yaw_degrees: float) -> tuple[float, float, float, float]:
    half = math.radians(yaw_degrees) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def pose(stamp_s: float, x: float, yaw_degrees: float) -> PoseSample:
    return PoseSample(
        round(stamp_s * 1_000_000_000),
        (x, 0.0, 0.0),
        quaternion(yaw_degrees),
    )


def group(stamp_s: float) -> SynchronizedGroup:
    stamp_ns = round(stamp_s * 1_000_000_000)
    return SynchronizedGroup(stamp_ns, {f"camera_{index}": stamp_ns for index in range(8)})


def test_synchronizer_emits_only_complete_eight_camera_groups() -> None:
    stamps = {
        f"camera_{index}": [100_000_000, 200_000_000, 300_000_000]
        for index in range(8)
    }
    stamps["camera_7"] = [100_000_000, 300_000_000]
    synchronized = synchronize_stamps(stamps, threshold_ns=40_000_000)
    assert [item.stamp_ns for item in synchronized] == [100_000_000, 300_000_000]
    assert all(len(item.image_stamps) == 8 for item in synchronized)


def test_keyframe_selection_trims_initialization_and_bounds_stationary_gap() -> None:
    groups = [group(index * 0.1) for index in range(41)]
    poses = [
        pose(index * 0.1, 0.0 if index < 11 else 0.11, 0.0)
        for index in range(41)
    ]
    selected, removed = select_keyframe_groups(
        groups,
        poses,
        minimum_translation_m=0.1,
        minimum_rotation_degrees=2.0,
        maximum_interval_s=0.5,
    )
    assert removed == 11
    assert selected[0].stamp_ns == 1_100_000_000
    assert maximum_gap_seconds(selected) <= 0.5
    assert selected[-1].stamp_ns == groups[-1].stamp_ns


def test_rotation_selects_dense_groups_without_translation() -> None:
    groups = [group(index * 0.1) for index in range(8)]
    poses = [pose(index * 0.1, 0.0, index * 3.0) for index in range(8)]
    selected, removed = select_keyframe_groups(
        groups,
        poses,
        minimum_translation_m=0.1,
        minimum_rotation_degrees=2.0,
        maximum_interval_s=0.5,
    )
    assert removed == 1
    assert len(selected) == 7
