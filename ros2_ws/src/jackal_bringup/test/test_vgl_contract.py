from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]


def test_vgl_profiles_use_eight_for_mapping_and_six_for_navigation() -> None:
    root = ROOT / "ros2_ws/src/jackal_bringup/config"
    mapping = yaml.safe_load((root / "vgl_mapping_8cam.yaml").read_text())[
        "visual_global_localization_node"
    ]["ros__parameters"]
    navigation = yaml.safe_load((root / "vgl_navigation_6cam.yaml").read_text())[
        "visual_global_localization_node"
    ]["ros__parameters"]
    assert mapping["num_cameras"] == 8
    assert mapping["stereo_localizer_cam_ids"] == "0,1,2,3,4,5,6,7"
    assert navigation["num_cameras"] == 6
    assert navigation["stereo_localizer_cam_ids"] == "0,1,2,3,4,5"
    assert len(mapping["camera_optical_frames"]) == 8
    assert len(navigation["camera_optical_frames"]) == 6
    for config in (mapping, navigation):
        assert config["localization_precision_level"] == 2
        assert config["enable_continuous_localization"] is False
        assert config["publish_map_to_base_tf"] is False
        assert config["publish_map_to_odom_tf"] is False


def test_mapping_topics_match_runtime_contract() -> None:
    config = yaml.safe_load(
        (ROOT / "ros2_ws/src/jackal_bringup/config/mapping_topics_8cam.yaml").read_text()
    )
    cameras = config["stereo_cameras"]
    assert [item["name"] for item in cameras] == [
        "front_stereo_camera",
        "left_stereo_camera",
        "right_stereo_camera",
        "back_stereo_camera",
    ]
    assert len([item[side] for item in cameras for side in ("left", "right")]) == 8
