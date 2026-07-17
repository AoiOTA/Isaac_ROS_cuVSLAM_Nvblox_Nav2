from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[4]


def test_sensor_topics_frames_and_rates_are_fixed() -> None:
    config = yaml.safe_load((ROOT / "config/sensors.yaml").read_text(encoding="utf-8"))
    front = config["front_stereo"]
    assert (front["image_width"], front["image_height"]) == (1280, 800)
    assert (front["depth_width"], front["depth_height"]) == (640, 400)
    assert front["image_rate_hz"] == 30.0
    assert front["imu_rate_hz"] == 120.0
    assert len(set(config["topics"].values())) == len(config["topics"])
    assert config["frames"]["base"] == "base_link"
    assert config["extrinsics"]["stereo_baseline_m"] == 0.15


def test_xacro_contains_the_usd_extracted_sensor_chain() -> None:
    path = ROOT / "ros2_ws/src/nova_carter_bringup/urdf/nova_carter.urdf.xacro"
    root = ET.parse(path).getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    expected = {
        "front_stereo_mount",
        "front_stereo_left_optical_joint",
        "front_stereo_right_optical_joint",
        "front_stereo_imu_joint",
        "joint_wheel_left",
        "joint_wheel_right",
        "joint_caster_base",
        "joint_swing_left",
        "joint_swing_right",
        "joint_caster_left",
        "joint_caster_right",
    }
    assert expected <= set(joints)
    assert joints["front_stereo_mount"].find("origin").attrib["xyz"] == "0.1003 -0.000002 0.3459"
    assert joints["front_stereo_left_optical_joint"].find("origin").attrib["xyz"] == "0 0.075 0"
    assert joints["front_stereo_right_optical_joint"].find("origin").attrib["xyz"] == "0 -0.075 0"
