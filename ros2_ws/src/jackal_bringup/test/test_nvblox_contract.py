from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]


def test_phase6_nvblox_configuration_contract() -> None:
    path = ROOT / "ros2_ws/src/jackal_bringup/config/nvblox.yaml"
    params = yaml.safe_load(path.read_text(encoding="utf-8"))["nvblox_node"][
        "ros__parameters"
    ]
    assert params["mapping_type"] == "static_tsdf"
    assert params["global_frame"] == "map"
    assert params["num_cameras"] == 1
    assert params["use_tf_transforms"] is True
    assert params["use_depth"] is True
    assert params["use_color"] is True
    assert params["use_lidar"] is False
    assert params["voxel_size"] == 0.05
    assert params["integrate_depth_rate_hz"] == 10.0
    assert params["integrate_color_rate_hz"] == 3.0
    assert params["update_esdf_rate_hz"] == 10.0
    assert params["update_mesh_rate_hz"] == 1.0
    assert params["esdf_mode"] == "2d"
    assert params["static_mapper"]["esdf_slice_min_height"] == 0.09
    assert params["static_mapper"]["esdf_slice_max_height"] == 0.65


def test_nvblox_launch_uses_project_sensor_contract() -> None:
    launch = (
        ROOT / "ros2_ws/src/jackal_bringup/launch/nvblox.launch.py"
    ).read_text(encoding="utf-8")
    for value in (
        'plugin="nvblox::NvbloxNode"',
        'name="nvblox_node"',
        '"/front_stereo_camera/depth/image_raw"',
        '"/front_stereo_camera/depth/camera_info"',
        '"/front_stereo_camera/left/image_raw_rgb"',
        '"/front_stereo_camera/left/camera_info"',
    ):
        assert value in launch
