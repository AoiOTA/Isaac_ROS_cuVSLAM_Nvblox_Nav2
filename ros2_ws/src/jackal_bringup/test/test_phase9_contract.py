from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[4]
BRINGUP = ROOT / "ros2_ws/src/jackal_bringup"

CAMERA_FRAMES = [
    f"{pair}_stereo_camera_{side}_optical"
    for pair in ("front", "left", "right", "back")
    for side in ("left", "right")
]


def parameters(name: str, node: str) -> dict:
    return yaml.safe_load((BRINGUP / "config" / name).read_text())[node][
        "ros__parameters"
    ]


def test_mapping_uses_all_eight_images_and_navigation_omits_rear_publishers() -> None:
    mapping = parameters("visual_slam_mapping_8cam.yaml", "visual_slam_node")
    navigation = parameters("visual_slam_navigation_6cam.yaml", "visual_slam_node")
    assert mapping["num_cameras"] == mapping["min_num_images"] == 8
    assert mapping["camera_optical_frames"] == CAMERA_FRAMES
    # The saved cuVSLAM rig remains an eight-camera calibration.  Runtime can
    # track from the six front/side publishers because min_num_images is two.
    assert navigation["num_cameras"] == 8
    assert navigation["min_num_images"] == 2
    assert navigation["camera_optical_frames"] == CAMERA_FRAMES
    assert mapping["image_jitter_threshold_ms"] >= 100.0
    assert navigation["image_jitter_threshold_ms"] >= 100.0

    mapping_vgl = parameters(
        "vgl_mapping_8cam.yaml", "visual_global_localization_node"
    )
    navigation_vgl = parameters(
        "vgl_navigation_6cam.yaml", "visual_global_localization_node"
    )
    assert mapping_vgl["num_cameras"] == 8
    assert mapping_vgl["stereo_localizer_cam_ids"] == "0,1,2,3,4,5,6,7"
    assert mapping_vgl["camera_optical_frames"] == CAMERA_FRAMES
    assert navigation_vgl["num_cameras"] == 6
    assert navigation_vgl["stereo_localizer_cam_ids"] == "0,1,2,3,4,5"
    assert navigation_vgl["camera_optical_frames"] == CAMERA_FRAMES[:6]


def test_mapping_topic_order_is_front_left_right_back() -> None:
    mapping = yaml.safe_load(
        (BRINGUP / "config/mapping_topics_8cam.yaml").read_text()
    )
    cameras = mapping["stereo_cameras"]
    assert [item["name"] for item in cameras] == [
        "front_stereo_camera",
        "left_stereo_camera",
        "right_stereo_camera",
        "back_stereo_camera",
    ]
    images = [item[key] for item in cameras for key in ("left", "right")]
    assert len(images) == len(set(images)) == 8


def test_ros_launches_remap_eight_inputs_but_gate_rear_normalizers() -> None:
    visual_slam = (BRINGUP / "launch/visual_slam.launch.py").read_text()
    vgl = (BRINGUP / "launch/vgl.launch.py").read_text()
    for index, (pair, side) in enumerate(
        (pair, side)
        for pair in ("front", "left", "right", "back")
        for side in ("left", "right")
    ):
        topic = f"/{pair}_stereo_camera/{side}/image_raw"
        assert f'("/visual_slam/image_{index}", "{topic}")' in visual_slam
        assert f'("visual_localization/image_{index}", "{topic}")' in vgl
    assert visual_slam.count("rear_only=True") == 2
    sensors_launch = (BRINGUP / "launch/sensors.launch.py").read_text()
    assert sensors_launch.count("rear_only=True") == 2
    assert "' == 'mapping_8cam'" in sensors_launch


def test_mapping_workflow_is_manual_eight_camera_and_discards_raw_capture() -> None:
    script = (ROOT / "scripts/run_mapping.sh").read_text()
    for token in (
        "--interactive",
        "[[ -t 0 ]]",
        "--gui --duration 0 --camera-profile mapping_8cam",
        "ros2 run jackal_teleop keyboard_teleop",
        "for pair in front left right back",
        "mapping_topics_8cam.yaml",
        "ros2 bag record --storage mcap",
        'rm -rf -- "${BAG_ROOT}"',
    ):
        assert token in script
    assert "--dynamic-profile" not in script


def test_navigation_is_locked_to_six_camera_static_profile() -> None:
    phase8 = (BRINGUP / "launch/phase8.launch.py").read_text()
    assert 'choices=["navigation_6cam"]' in phase8
    assert "visual_slam_navigation_6cam.yaml" in phase8
    assert "vgl_navigation_6cam.yaml" in phase8
    script = (ROOT / "scripts/run_navigation.sh").read_text()
    assert "check_map_manifest.py" in script
    assert "camera_profile:=navigation_6cam" in script
    simulator = (ROOT / "isaac_sim/navigation_sim.py").read_text()
    assert "dynamic obstacle profiles are not supported" in simulator
    assert 'report["dynamic_obstacles"] = {"enabled": False}' in simulator


def test_simulator_explicitly_applies_the_reference_physics_timestep() -> None:
    simulator = (ROOT / "isaac_sim/navigation_sim.py").read_text()
    articulation = (ROOT / "isaac_sim/jackal_sim/articulation_runtime.py").read_text()
    assert "SimulationManager.set_physics_dt" in simulator
    assert "SimulationManager.get_physics_dt" in simulator
    assert "SimulationManager.step(steps=1, update_fabric=False)" in simulator
    assert "robot_runtime.set_world_pose" in simulator
    assert "def set_world_pose" in articulation
    assert 'report["physics_timing"]' in simulator
    assert "explicit_simulation_manager_configuration" in simulator
    assert "reference_reset_lifecycle" in simulator


def test_differential_drive_runs_on_every_physics_step() -> None:
    graphs = (ROOT / "isaac_sim/jackal_sim/graphs.py").read_text()
    differential_drive = graphs.split(
        "def _create_differential_drive_graph", 1
    )[1].split("def _create_joint_state_graph", 1)[0]
    assert "OnPhysicsStep" in differential_drive
    assert "GRAPH_PIPELINE_STAGE_ONDEMAND" in differential_drive
    assert '"evaluator_name": "execution"' not in differential_drive
    assert (
        '"OnPhysicsStep.outputs:deltaSimulationTime",\n'
        '                    "DifferentialController.inputs:dt"'
    ) in differential_drive


def test_nvblox_consumes_only_front_native_simulated_depth() -> None:
    launch = (BRINGUP / "launch/nvblox.launch.py").read_text()
    assert '"/front_stereo_camera/depth/image_raw"' in launch
    assert '"/front_stereo_camera/depth/camera_info"' in launch
    for pair in ("left", "right", "back"):
        assert f'"/{pair}_stereo_camera/depth/' not in launch
    config = parameters("nvblox.yaml", "nvblox_node")
    assert config["num_cameras"] == 1
    assert config["use_depth"] is True
    assert config["use_lidar"] is False
    sensors = yaml.safe_load((ROOT / "config/sensors.yaml").read_text())
    assert sensors["lidar_enabled"] is False
    sensor_source = (ROOT / "isaac_sim/jackal_sim/sensors.py").read_text()
    assert "Jackal LiDAR must remain disabled" in sensor_source
