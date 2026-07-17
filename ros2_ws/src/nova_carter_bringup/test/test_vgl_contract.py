from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]


def test_vgl_has_single_front_stereo_and_does_not_publish_tf() -> None:
    config = yaml.safe_load(
        (ROOT / "ros2_ws/src/nova_carter_bringup/config/vgl.yaml").read_text()
    )["visual_global_localization_node"]["ros__parameters"]
    assert config["num_cameras"] == 2
    assert config["stereo_localizer_cam_ids"] == "0,1"
    assert config["localization_precision_level"] == 2
    assert config["enable_continuous_localization"] is False
    assert config["publish_map_to_base_tf"] is False
    assert config["publish_map_to_odom_tf"] is False


def test_mapping_topics_match_runtime_contract() -> None:
    config = yaml.safe_load(
        (ROOT / "ros2_ws/src/nova_carter_bringup/config/mapping_topics.yaml").read_text()
    )
    assert config == {
        "stereo_cameras": [
            {
                "name": "front_stereo_camera",
                "left": "/front_stereo_camera/left/image_raw",
                "left_camera_info": "/front_stereo_camera/left/camera_info",
                "right": "/front_stereo_camera/right/image_raw",
                "right_camera_info": "/front_stereo_camera/right/camera_info",
            }
        ]
    }
