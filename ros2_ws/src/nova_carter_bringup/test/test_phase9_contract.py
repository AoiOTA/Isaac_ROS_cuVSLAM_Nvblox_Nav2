from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[4]
BRINGUP = ROOT / "ros2_ws/src/nova_carter_bringup"


def test_surround_hawk_contract_and_fixed_order() -> None:
    sensors = yaml.safe_load((ROOT / "config/sensors.yaml").read_text())
    surround = sensors["surround_stereo"]
    assert surround["image_rate_hz"] == 10.0
    assert (surround["image_width"], surround["image_height"]) == (1280, 800)
    assert surround["enable_topic"] == "/vgl/cameras_enabled"
    assert list(surround["cameras"]) == ["left", "right", "back"]
    for name, pair in surround["cameras"].items():
        assert pair["left_camera_prim"].endswith(f"/{name}_hawk/left/camera_left")
        assert pair["right_camera_prim"].endswith(f"/{name}_hawk/right/camera_right")

    vgl = yaml.safe_load((BRINGUP / "config/vgl_4way.yaml").read_text())[
        "visual_global_localization_node"
    ]["ros__parameters"]
    assert vgl["num_cameras"] == 8
    assert vgl["stereo_localizer_cam_ids"] == "0,1,2,3,4,5,6,7"
    assert vgl["camera_optical_frames"] == [
        "front_stereo_camera_left_optical",
        "front_stereo_camera_right_optical",
        "left_stereo_camera_left_optical",
        "left_stereo_camera_right_optical",
        "right_stereo_camera_left_optical",
        "right_stereo_camera_right_optical",
        "back_stereo_camera_left_optical",
        "back_stereo_camera_right_optical",
    ]
    assert vgl["publish_map_to_base_tf"] is False
    assert vgl["publish_map_to_odom_tf"] is False


def test_four_way_mapping_and_runtime_remaps_are_complete() -> None:
    mapping = yaml.safe_load((BRINGUP / "config/mapping_topics_4way.yaml").read_text())
    assert [item["name"] for item in mapping["stereo_cameras"]] == [
        "front_stereo_camera",
        "left_stereo_camera",
        "right_stereo_camera",
        "back_stereo_camera",
    ]
    launch = (BRINGUP / "launch/vgl.launch.py").read_text()
    for index, camera in enumerate(
        ("front/left", "front/right", "left/left", "left/right", "right/left", "right/right", "back/left", "back/right")
    ):
        body, side = camera.split("/")
        assert f'("visual_localization/image_{index}", "/{body}_stereo_camera/{side}/image_raw")' in launch
        assert f'"visual_localization/camera_info_{index}"' in launch
    vslam_launch = (BRINGUP / "launch/visual_slam.launch.py").read_text()
    for index, camera in enumerate(
        ("front/left", "front/right", "left/left", "left/right", "right/left", "right/right", "back/left", "back/right")
    ):
        body, side = camera.split("/")
        assert f'("/visual_slam/image_{index}", "/{body}_stereo_camera/{side}/image_raw")' in vslam_launch


def test_side_and_back_frames_are_in_robot_description() -> None:
    root = ET.parse(BRINGUP / "urdf/nova_carter.urdf.xacro").getroot()
    joints = {item.attrib["name"] for item in root.findall("joint")}
    for camera in ("left", "right", "back"):
        assert f"{camera}_stereo_mount" in joints
        assert f"{camera}_stereo_left_optical_joint" in joints
        assert f"{camera}_stereo_right_optical_joint" in joints


def test_dynamic_nvblox_and_combined_nav2_wiring() -> None:
    dynamic = yaml.safe_load((BRINGUP / "config/nvblox_dynamic.yaml").read_text())[
        "nvblox_node"
    ]["ros__parameters"]
    assert dynamic["mapping_type"] == "dynamic"
    assert dynamic["decay_dynamic_occupancy_rate_hz"] >= 10.0
    assert dynamic["dynamic_mapper"]["occupied_region_decay_probability"] > 0.0
    phase9 = (BRINGUP / "launch/phase9.launch.py").read_text()
    localization = (BRINGUP / "launch/phase9_localization.launch.py").read_text()
    assert '"/nvblox_node/combined_map_slice"' in phase9
    assert '"map_slice_topic": "/nvblox_node/combined_map_slice"' in localization
    assert '"nvblox_dynamic.launch.py"' in phase9
    assert '"anchor_pose_topic": "/vgl_pose_relay/pose"' in phase9
    assert '"anchor_ready_topic": "/localization/ready"' in phase9
    assert '"odometry_topic": "/odometry/filtered"' in phase9
    assert 'package="robot_localization"' in localization
    assert '"publish_odom_to_base_tf": "false"' in localization
    ekf = yaml.safe_load((BRINGUP / "config/visual_wheel_ekf.yaml").read_text())[
        "visual_wheel_ekf"
    ]["ros__parameters"]
    assert ekf["world_frame"] == "odom"
    assert ekf["odom0"] == "/wheel/odometry"
    assert "odom1" not in ekf
    assert 'executable="localization_recovery_manager"' in localization
    assert 'executable="resilient_navigation"' in localization


def test_dynamic_scenario_has_heterogeneous_movers() -> None:
    scenarios = yaml.safe_load((ROOT / "config/scenarios.yaml").read_text())
    obstacles = scenarios["dynamic_profiles"]["warehouse_crossing"]["obstacles"]
    assert {item["kind"] for item in obstacles} == {
        "existing_forklift",
        "box",
        "capsule",
    }
    assert any(item.get("prim_path") == "/World/Forklift" for item in obstacles)
    assert all(item["period_s"] > 0.0 for item in obstacles)


def test_phase9_runtime_uses_front_stereo_and_strict_sync() -> None:
    script = (ROOT / "scripts/run_phase9_navigation.sh").read_text()
    assert "prepare_vgl_runtime_config.py" in script
    assert "--max-sync-us 3000" in script
    assert 'vgl_config_dir:="${RUNTIME_CONFIG_DIR}"' in script
    full = (ROOT / "scripts/run_phase9.sh").read_text()
    assert "--enable-surround-cameras" not in full
    assert '"${PHASE9_FRONT_IMAGE_RATE_HZ:-10}"' in full
    assert "--reliable-sensor-qos" in full
    assert "require_surround_cameras:=false" in full
    mapping_entry = (ROOT / "scripts/run_phase9_mapping.sh").read_text()
    assert "--four-way" not in mapping_entry


def test_phase9_front_mapping_is_lossless_and_four_way_remains_optional() -> None:
    mapping = (ROOT / "scripts/run_mapping.sh").read_text()
    phase9 = (ROOT / "scripts/run_phase9_mapping.sh").read_text()
    assert "--front-rate-hz 10 --reliable-sensor-qos" in phase9
    assert '--front-image-rate-hz "${FRONT_RATE_HZ}"' in mapping
    assert "--surround-image-rate-hz 10" in mapping
    assert "--reliable-sensor-qos" in mapping
    assert "image_qos:=DEFAULT" in mapping
    assert "--storage-preset-profile fastwrite" in mapping
    assert "expected_camera_info_rate_hz:=8.0" in mapping
    assert "min_depth_integration_rate_hz:=6.0" in mapping
    assert "min_color_integration_rate_hz:=1.0" in mapping
    offline = (ROOT / "scripts/create_vgl_map.sh").read_text()
    assert '--sample_sync_threshold_microseconds="${MAX_SYNC_US}"' in offline


def test_four_way_mapping_preserves_validated_front_resolution() -> None:
    script = (ROOT / "scripts/run_mapping.sh").read_text()
    assert "--front-image-width 1280 --front-image-height 800" in script
    assert "front_image_width:=1280 front_image_height:=800" in script
    assert "--surround-image-width 1280 --surround-image-height 800" in script
    visual_slam = (BRINGUP / "launch/visual_slam.launch.py").read_text()
    assert 'DeclareLaunchArgument("front_image_width", default_value="1280")' in visual_slam
    phase9 = (BRINGUP / "launch/phase9_localization.launch.py").read_text()
    assert '"front_image_width": "1280"' in phase9
    assert '"front_image_height": "800"' in phase9


def test_stage9_front_map_and_optional_four_way_map_preserve_alignment() -> None:
    params = yaml.safe_load((BRINGUP / "config/visual_slam_4way.yaml").read_text())[
        "visual_slam_node"
    ]["ros__parameters"]
    assert params["num_cameras"] == 8
    assert params["min_num_images"] == 2
    assert params["camera_optical_frames"] == [
        "front_stereo_camera_left_optical",
        "front_stereo_camera_right_optical",
        "left_stereo_camera_left_optical",
        "left_stereo_camera_right_optical",
        "right_stereo_camera_left_optical",
        "right_stereo_camera_right_optical",
        "back_stereo_camera_left_optical",
        "back_stereo_camera_right_optical",
    ]
    mapping = (ROOT / "scripts/run_mapping.sh").read_text()
    assert 'cp -a "${MAP_DIR}/online_cuvslam"' not in mapping
    phase9 = (BRINGUP / "launch/phase9_localization.launch.py").read_text()
    assert 'share / "config/visual_slam_phase9.yaml"' in phase9
    assert 'share / "config/vgl.yaml"' in phase9
    assert '"enable_surround_cameras": False' in phase9

    runtime = yaml.safe_load(
        (BRINGUP / "config/visual_slam_phase9.yaml").read_text()
    )["visual_slam_node"]["ros__parameters"]
    assert runtime["num_cameras"] == 2
    assert runtime["image_jitter_threshold_ms"] >= 50.0
    assert runtime["image_qos"] == "DEFAULT"
    assert runtime["slam_throttling_time_ms"] >= 10000
    assert runtime["enable_ground_constraint_in_odometry"] is False


def test_rviz_exposes_dynamic_and_on_demand_sources() -> None:
    text = (BRINGUP / "rviz/navigation.rviz").read_text()
    for topic in (
        "/nvblox_node/dynamic_esdf_pointcloud",
        "/nvblox_node/combined_esdf_pointcloud",
        "/nvblox_node/dynamic_points",
        "/left_stereo_camera/left/image_raw",
        "/right_stereo_camera/left/image_raw",
        "/back_stereo_camera/left/image_raw",
    ):
        assert topic in text


def test_phase9_full_runner_owns_a_local_discovery_server() -> None:
    text = (ROOT / "scripts/run_phase9.sh").read_text()
    assert "fastdds discovery" in text
    assert 'ROS_DISCOVERY_SERVER="127.0.0.1:${DISCOVERY_PORT}"' in text
    assert "unset ROS_LOCALHOST_ONLY" in text
    assert 'stop_group "${DISCOVERY_PID}"' in text
