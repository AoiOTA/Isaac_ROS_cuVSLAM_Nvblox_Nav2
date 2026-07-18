from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[4]


def test_sensor_topics_frames_and_rates_are_fixed() -> None:
    config = yaml.safe_load((ROOT / "config/sensors.yaml").read_text(encoding="utf-8"))
    front = config["front_stereo"]
    assert (front["image_width"], front["image_height"]) == (1280, 800)
    assert (front["depth_width"], front["depth_height"]) == (640, 400)
    assert front["depth_min_range_m"] == 0.40
    assert front["image_rate_hz"] == 10.0
    assert front["imu_rate_hz"] == 120.0
    assert front["navigation_projection"] == "pinhole"
    assert len(set(config["topics"].values())) == len(config["topics"])
    assert config["frames"]["base"] == "base_link"
    assert config["extrinsics"]["stereo_baseline_m"] == 0.15
    assert list(config["surround_stereo"]["cameras"]) == ["left", "right", "back"]
    assert config["surround_stereo"]["image_rate_hz"] == 10.0


def test_front_depth_excludes_jackal_self_returns_at_the_camera() -> None:
    source = (ROOT / "isaac_sim/jackal_sim/sensors.py").read_text(encoding="utf-8")
    assert "_configure_front_depth_near_clip(stage, sensor)" in source
    assert "GetClippingRangeAttr" in source
    assert "Gf.Vec2f(minimum" in source


def test_xacro_contains_the_usd_extracted_sensor_chain() -> None:
    path = ROOT / "ros2_ws/src/jackal_bringup/urdf/jackal.urdf.xacro"
    expanded = subprocess.run(
        ["xacro", str(path)], check=True, capture_output=True, text=True
    ).stdout
    root = ET.fromstring(expanded)
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    expected = {
        "front_stereo_mount",
        "front_stereo_left_optical_joint",
        "front_stereo_right_optical_joint",
        "front_stereo_imu_joint",
        "left_stereo_mount",
        "left_stereo_left_optical_joint",
        "left_stereo_right_optical_joint",
        "right_stereo_mount",
        "right_stereo_left_optical_joint",
        "right_stereo_right_optical_joint",
        "back_stereo_mount",
        "back_stereo_left_optical_joint",
        "back_stereo_right_optical_joint",
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    }
    assert expected <= set(joints)
    assert joints["front_stereo_mount"].find("origin").attrib["xyz"] == "0 0 0.42"
    assert joints["front_stereo_left_optical_joint"].find("origin").attrib["xyz"] == "0 0.075 0"
    assert joints["front_stereo_right_optical_joint"].find("origin").attrib["xyz"] == "0 -0.075 0"
    assert joints["left_stereo_mount"].find("origin").attrib["rpy"].endswith(
        "1.57079632679"
    )
    assert joints["right_stereo_mount"].find("origin").attrib["rpy"].endswith(
        "-1.57079632679"
    )
