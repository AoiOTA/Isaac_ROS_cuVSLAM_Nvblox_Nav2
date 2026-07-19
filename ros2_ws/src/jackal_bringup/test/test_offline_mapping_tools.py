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
from run_offline_nvblox_fusion import gflags  # noqa: E402


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
