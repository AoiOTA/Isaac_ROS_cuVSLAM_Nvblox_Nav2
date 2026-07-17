from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]


def test_visual_slam_owns_the_main_tf_chain() -> None:
    config = yaml.safe_load(
        (ROOT / "ros2_ws/src/nova_carter_bringup/config/visual_slam.yaml").read_text(
            encoding="utf-8"
        )
    )["visual_slam_node"]["ros__parameters"]
    assert config["tracking_mode"] == 1
    assert config["num_cameras"] == 2
    assert config["base_frame"] == "base_link"
    assert config["map_frame"] == "map"
    assert config["odom_frame"] == "odom"
    assert config["publish_map_to_odom_tf"] is True
    assert config["publish_odom_to_base_tf"] is True


def test_visual_slam_uses_front_stereo_and_imu() -> None:
    launch = (
        ROOT / "ros2_ws/src/nova_carter_bringup/launch/visual_slam.launch.py"
    ).read_text(encoding="utf-8")
    for topic in (
        "/front_stereo_camera/left/image_raw",
        "/front_stereo_camera/right/image_raw",
        "/front_stereo_camera/left/camera_info",
        "/front_stereo_camera/right/camera_info",
        "/front_stereo_imu/imu",
    ):
        assert topic in launch
    assert "use_intra_process_comms" in launch
