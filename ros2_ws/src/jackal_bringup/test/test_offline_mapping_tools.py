from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "tools"))

from prepare_native_depth_fusion import (  # noqa: E402
    depth_to_millimetres,
    load_selected_frames,
    nearest_target_index,
    safe_relative_image_path,
)
from check_visual_map_stage import require_same_pose  # noqa: E402
from run_offline_nvblox_fusion import gflags  # noqa: E402
from write_optimized_frames_report import analyze_frames_metadata  # noqa: E402


def test_native_depth_conversion_is_metric_and_range_gated() -> None:
    values = np.asarray([[0.2, 0.4, 1.234], [8.0, 8.1, np.nan]], dtype=np.float32)
    message = SimpleNamespace(
        encoding="32FC1",
        is_bigendian=False,
        step=values.shape[1] * values.dtype.itemsize,
        width=values.shape[1],
        height=values.shape[0],
        data=values.tobytes(),
    )
    depth, metrics = depth_to_millimetres(message, 0.4, 8.0)
    assert depth.tolist() == [[0, 400, 1234], [8000, 0, 0]]
    assert metrics["valid_fraction"] == pytest.approx(0.5)


def test_keyframe_matching_and_paths_are_bounded() -> None:
    timestamps = [100, 200, 300]
    assert nearest_target_index(timestamps, 249) == 1
    assert nearest_target_index(timestamps, 251) == 2
    assert safe_relative_image_path("front/100.jpg", ".png") == Path(
        "front/100.png"
    )
    with pytest.raises(ValueError):
        safe_relative_image_path("../escape.jpg", ".png")


def test_protobuf_default_camera_zero_is_accepted_when_field_is_omitted(
    tmp_path: Path,
) -> None:
    metadata = tmp_path / "frames_meta.json"
    metadata.write_text(
        '{"camera_params_id_to_camera_params":{"0":{"sensor_meta_data":'
        '{"sensor_name":"camera_zero"}}},"keyframes_metadata":['
        '{"timestamp_microseconds":"100","image_name":"zero/100.jpg"}]}'
    )
    _, camera_id, frames = load_selected_frames(metadata, "camera_zero")
    assert camera_id == "0"
    assert len(frames) == 1


def test_nvblox_gflags_preserve_false_values() -> None:
    arguments = gflags(
        {
            "visualization": False,
            "use_2d_esdf_mode": True,
            "voxel_size": 0.05,
        }
    )
    assert "--visualization=false" in arguments
    assert "--use_2d_esdf_mode" in arguments
    assert "--voxel_size=0.05" in arguments


def test_shared_optimized_frames_require_complete_camera_groups() -> None:
    parameters = {
        str(index): {"sensor_meta_data": {"sensor_name": f"camera_{index}"}}
        for index in range(8)
    }
    frames = [
        {
            "camera_params_id": str(index),
            "timestamp_microseconds": "100",
            "image_name": f"camera_{index}/100.jpg",
            "camera_to_world": {},
        }
        for index in range(8)
    ]
    summary = analyze_frames_metadata(
        {
            "camera_params_id_to_camera_params": parameters,
            "keyframes_metadata": frames,
        }
    )
    assert summary["camera_streams"] == 8
    assert summary["synchronized_groups"] == 1
    assert summary["frame_rows"] == 8

    with pytest.raises(RuntimeError, match="incomplete"):
        analyze_frames_metadata(
            {
                "camera_params_id_to_camera_params": parameters,
                "keyframes_metadata": frames[:-1],
            }
        )


def test_pose_comparison_normalizes_axis_angle_quaternions() -> None:
    pose = {
        "camera_to_world": {
            "translation": {"x": 1.0, "y": 2.0, "z": 3.0},
            "axis_angle": {
                "x": 0.391294238491,
                "y": -0.761829403124,
                "z": 0.516278399713,
                "angle_degrees": 137.123456789,
            },
        }
    }
    require_same_pose(pose, pose)
