from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]


def test_visual_slam_profiles_share_saved_rig_and_tf_contract() -> None:
    root = ROOT / "ros2_ws/src/jackal_bringup/config"
    mapping = yaml.safe_load((root / "visual_slam_mapping_8cam.yaml").read_text())[
        "visual_slam_node"
    ]["ros__parameters"]
    navigation = yaml.safe_load(
        (root / "visual_slam_navigation_6cam.yaml").read_text()
    )["visual_slam_node"]["ros__parameters"]
    assert mapping["num_cameras"] == mapping["min_num_images"] == 8
    assert navigation["num_cameras"] == 8
    assert navigation["min_num_images"] == 2
    assert mapping["camera_optical_frames"] == navigation["camera_optical_frames"]
    for config in (mapping, navigation):
        assert config["tracking_mode"] == 1
        assert config["base_frame"] == "base_link"
        assert config["map_frame"] == "map"
        assert config["odom_frame"] == "odom"
        assert config["publish_map_to_odom_tf"] is True
        assert config["publish_odom_to_base_tf"] is True
        assert config["override_publishing_stamp"] is False


def test_live_nav2_overrides_cuvslam_output_stamp_only_in_phase8() -> None:
    visual_slam_launch = (
        ROOT / "ros2_ws/src/jackal_bringup/launch/visual_slam.launch.py"
    ).read_text(encoding="utf-8")
    phase8_launch = (
        ROOT / "ros2_ws/src/jackal_bringup/launch/phase8.launch.py"
    ).read_text(encoding="utf-8")
    assert '"override_publishing_stamp",\n                default_value="false"' in visual_slam_launch
    assert '"override_publishing_stamp": "true"' in phase8_launch


def test_visual_slam_remaps_all_cardinal_stereo_pairs_and_front_imu() -> None:
    launch = (
        ROOT / "ros2_ws/src/jackal_bringup/launch/visual_slam.launch.py"
    ).read_text(encoding="utf-8")
    for topic in [
        f"/{pair}_stereo_camera/{side}/{leaf}"
        for pair in ("front", "left", "right", "back")
        for side in ("left", "right")
        for leaf in ("image_raw", "camera_info")
    ] + ["/front_stereo_imu/imu"]:
        assert topic in launch
    assert "use_intra_process_comms" in launch
