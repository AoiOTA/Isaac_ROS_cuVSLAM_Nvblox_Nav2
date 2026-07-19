from types import SimpleNamespace

import pytest

from jackal_experiments.visual_map_saver import (
    DEFAULT_MAXIMUM_3D_TO_PLANAR_RATIO,
    DEFAULT_MINIMUM_PLANAR_PATH_LENGTH_M,
    pose_record,
    trajectory_metrics,
    tum_line,
)


def message(timestamp: float, x: float, y: float, z: float) -> SimpleNamespace:
    seconds = int(timestamp)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=int((timestamp - seconds) * 1e9)),
            frame_id="map",
        ),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def test_trajectory_metrics_measure_closed_planar_route() -> None:
    records = [
        pose_record(message(3.0, 0.0, 0.0, 0.0)),
        pose_record(message(1.0, 0.0, 0.0, 0.0)),
        pose_record(message(2.0, 1.0, 0.0, 0.0)),
    ]
    metrics = trajectory_metrics(records)
    assert metrics["pose_count"] == 3
    assert metrics["duration_s"] == pytest.approx(2.0)
    assert metrics["planar_path_length_m"] == pytest.approx(2.0)
    assert metrics["path_length_3d_to_planar_ratio"] == pytest.approx(1.0)
    assert metrics["closure_error_m"] == pytest.approx(0.0)
    assert metrics["vertical_range_m"] == pytest.approx(0.0)
    assert metrics["frame_ids"] == ["map"]


def test_tum_line_preserves_timestamp_and_quaternion_order() -> None:
    record = pose_record(message(7.25, 1.0, -2.0, 0.5))
    assert tum_line(record) == (
        "7.250000000 1.000000000 -2.000000000 0.500000000 "
        "0.000000000 0.000000000 0.000000000 1.000000000"
    )


def test_pose_record_rejects_non_finite_values() -> None:
    with pytest.raises(ValueError, match="non-finite"):
        pose_record(message(1.0, float("nan"), 0.0, 0.0))


def test_empty_get_all_poses_frame_is_preserved_for_version_policy() -> None:
    item = message(1.0, 0.0, 0.0, 0.0)
    item.header.frame_id = ""
    assert trajectory_metrics([pose_record(item)])["frame_ids"] == [""]


def test_short_planar_capture_is_rejected_by_distance_not_z_jitter_ratio() -> None:
    records = [
        pose_record(message(float(index), index * 0.01, 0.0, (index % 2) * 0.002))
        for index in range(29)
    ]
    metrics = trajectory_metrics(records)
    assert metrics["planar_path_length_m"] < DEFAULT_MINIMUM_PLANAR_PATH_LENGTH_M
    assert (
        metrics["path_length_3d_to_planar_ratio"]
        <= DEFAULT_MAXIMUM_3D_TO_PLANAR_RATIO
    )
