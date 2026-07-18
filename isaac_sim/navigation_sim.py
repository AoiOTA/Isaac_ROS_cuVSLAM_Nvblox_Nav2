#!/usr/bin/env python3
"""Isaac Sim 6.0.1 entry point for the Kujiale, Jackal and Hawk stack."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import traceback

import yaml

from jackal_sim.process_lock import SimulatorAlreadyRunning, SimulatorProcessLock


PROJECT_ROOT = Path(__file__).resolve().parents[1]
STOP_REQUESTED = False


def request_stop(_signum: int, _frame: object) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


def load_config(path: Path) -> dict[str, object]:
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"simulation config must contain a mapping: {path}")
    stage = config.get("stage", {})
    if not isinstance(stage, dict):
        raise ValueError("stage config entry must be a mapping")
    if stage.get("robot_prim") != "/World/Jackal":
        raise ValueError("simulation config must use robot prim /World/Jackal")
    if stage.get("environment_default_prim") != "/Root":
        raise ValueError("Kujiale source must use default prim /Root")
    if stage.get("compose_in_session_layer") is not True:
        raise ValueError("stage composition must remain in the session layer")
    if stage.get("save_composed_stage") is not False:
        raise ValueError("saving the composed official stage is forbidden")
    if stage.get("stage_open_count") != 1:
        raise ValueError("the Kujiale stage must be opened exactly once")
    return config


def load_mapping(path: Path, label: str) -> dict[str, object]:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain a mapping: {path}")
    return value


def parse_args() -> argparse.Namespace:
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "config/simulation.yaml")
    pre_args, _ = pre_parser.parse_known_args()
    if not pre_args.config.is_file():
        pre_parser.error(f"simulation config does not exist: {pre_args.config}")
    try:
        config = load_config(pre_args.config)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        pre_parser.error(str(exc))

    try:
        runtime = config["runtime"]
        spawn = config["spawn_search"]
        resolution = runtime["resolution"]
        preferred_xy = spawn["preferred_xy_m"]
        if len(resolution) != 2 or len(preferred_xy) != 2:
            raise ValueError("resolution and preferred_xy_m must each contain two values")
        lock_path = Path(runtime["project_process_lock"])
    except (KeyError, TypeError, ValueError) as exc:
        pre_parser.error(f"invalid simulation config: {exc}")
    if not lock_path.is_absolute():
        lock_path = PROJECT_ROOT / lock_path

    parser = argparse.ArgumentParser(
        description="Open Kujiale once and compose Jackal plus four Hawk pairs in memory"
    )
    parser.add_argument("--config", type=Path, default=pre_args.config)
    parser.add_argument(
        "--control-config", type=Path, default=PROJECT_ROOT / "config/control.yaml"
    )
    parser.add_argument(
        "--sensor-config", type=Path, default=PROJECT_ROOT / "config/sensors.yaml"
    )
    parser.add_argument(
        "--front-image-width",
        type=int,
        default=None,
        help="optional front RGB width override used by four-way offline mapping",
    )
    parser.add_argument(
        "--front-image-height",
        type=int,
        default=None,
        help="optional front RGB height override used by four-way offline mapping",
    )
    parser.add_argument(
        "--front-image-rate-hz",
        type=float,
        default=None,
        help="optional front camera rate override for deterministic map capture",
    )
    parser.add_argument(
        "--reliable-sensor-qos",
        action="store_true",
        help="use reliable sensor writers while recording calibration maps",
    )
    parser.add_argument(
        "--scenario-config", type=Path, default=PROJECT_ROOT / "config/scenarios.yaml"
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--headless", action="store_true")
    mode.add_argument("--gui", action="store_true")
    parser.add_argument(
        "--environment-usd", type=Path, default=Path(os.environ.get("KUJIALE_USD", ""))
    )
    parser.add_argument(
        "--robot-usd", type=Path, default=Path(os.environ.get("JACKAL_USD", ""))
    )
    parser.add_argument(
        "--hawk-usd",
        type=Path,
        default=Path(os.environ.get("HAWK_USD", "")),
    )
    parser.add_argument(
        "--duration", type=float, default=0.0, help="wall seconds; 0 runs until stopped"
    )
    parser.add_argument(
        "--benchmark-performance",
        action="store_true",
        help="run adaptive wall-time performance sampling with Isaac's official recorders",
    )
    parser.add_argument(
        "--performance-start-file",
        type=Path,
        default=None,
        help="start adaptive warmup only after the external ROS workload creates this file",
    )
    parser.add_argument("--performance-min-warmup-s", type=float, default=10.0)
    parser.add_argument("--performance-max-warmup-s", type=float, default=90.0)
    parser.add_argument("--performance-window-s", type=float, default=5.0)
    parser.add_argument("--performance-stable-windows", type=int, default=3)
    parser.add_argument("--performance-max-mean-change", type=float, default=0.03)
    parser.add_argument("--performance-max-cv", type=float, default=0.10)
    parser.add_argument("--performance-min-sample-s", type=float, default=30.0)
    parser.add_argument("--performance-max-sample-s", type=float, default=180.0)
    parser.add_argument(
        "--stop-file",
        type=Path,
        default=None,
        help="optional runtime sentinel; exit cleanly when this file appears",
    )
    parser.add_argument("--physics-hz", type=int, default=int(runtime["physics_hz"]))
    parser.add_argument("--update-hz", type=float, default=float(runtime["update_hz"]))
    parser.add_argument(
        "--spawn-clearance", type=float, default=float(spawn["footprint_clearance_m"])
    )
    parser.add_argument("--spawn-x", type=float, default=None)
    parser.add_argument("--spawn-y", type=float, default=None)
    parser.add_argument(
        "--spawn-yaw",
        type=float,
        default=math.radians(float(spawn.get("yaw_deg", 180.0))),
        help="initial yaw in radians",
    )
    parser.add_argument("--width", type=int, default=int(resolution[0]))
    parser.add_argument("--height", type=int, default=int(resolution[1]))
    parser.add_argument(
        "--report", type=Path, default=PROJECT_ROOT / "data/logs/stage4/latest.json"
    )
    parser.add_argument(
        "--disable-ros-control",
        action="store_true",
        help="run the composed stage without Phase 3 runtime ROS control graphs",
    )
    parser.add_argument(
        "--disable-sensors",
        action="store_true",
        help="run without the Phase 4 front stereo, depth, and IMU graphs",
    )
    parser.add_argument(
        "--camera-profile",
        choices=("mapping_8cam", "navigation_6cam"),
        default="navigation_6cam",
        help="mapping publishes all four stereo pairs; navigation omits the rear pair",
    )
    parser.add_argument(
        "--dynamic-profile",
        default="",
        help="name under scenarios.yaml dynamic_profiles; empty disables obstacles",
    )
    parser.add_argument(
        "--static-profile",
        default="",
        help="name under scenarios.yaml static_profiles; empty disables static obstacles",
    )
    parser.add_argument(
        "--disable-follow-camera",
        action="store_true",
        help="keep the default GUI viewport camera instead of the smooth robot follower",
    )
    parser.add_argument(
        "--lock-file", type=Path, default=lock_path
    )
    parser.add_argument(
        "--lock-wait-seconds",
        type=float,
        default=30.0,
        help="wait briefly for this project's previous simulator to release its lock",
    )
    args = parser.parse_args()
    if args.duration < 0.0:
        parser.error("--duration must be non-negative")
    if args.benchmark_performance and args.duration != 0.0:
        parser.error("adaptive performance sampling requires --duration 0")
    if args.lock_wait_seconds < 0.0:
        parser.error("--lock-wait-seconds must be non-negative")
    if args.physics_hz <= 0 or args.update_hz <= 0.0:
        parser.error("physics and update frequencies must be positive")
    args.renderer = str(runtime["renderer"])
    args.spawn_grid_resolution = float(spawn["grid_resolution_m"])
    args.spawn_height = float(spawn["height_above_floor_m"])
    args.spawn_obstacle_height = float(spawn["obstacle_height_m"])
    args.spawn_preferred_xy = tuple(float(value) for value in preferred_xy)
    if (args.spawn_x is None) != (args.spawn_y is None):
        parser.error("--spawn-x and --spawn-y must be supplied together")
    if args.spawn_x is not None:
        args.spawn_preferred_xy = (args.spawn_x, args.spawn_y)
    for label, path in (
        ("Kujiale", args.environment_usd),
        ("Jackal", args.robot_usd),
        ("Hawk", args.hawk_usd),
    ):
        if not path.is_file():
            parser.error(f"{label} USD does not exist: {path}")
    if not args.control_config.is_file():
        parser.error(f"control config does not exist: {args.control_config}")
    try:
        args.control = load_mapping(args.control_config, "control config")
        for key in ("kinematics", "limits", "topics", "frames"):
            if not isinstance(args.control.get(key), dict):
                raise ValueError(f"control config entry {key!r} must be a mapping")
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    if not args.sensor_config.is_file():
        parser.error(f"sensor config does not exist: {args.sensor_config}")
    try:
        args.sensor = load_mapping(args.sensor_config, "sensor config")
        for key in ("front_stereo", "topics", "frames", "extrinsics"):
            if not isinstance(args.sensor.get(key), dict):
                raise ValueError(f"sensor config entry {key!r} must be a mapping")
        if args.sensor.get("lidar_enabled") is not False:
            raise ValueError("Jackal LiDAR must be explicitly disabled")
        if (args.front_image_width is None) != (args.front_image_height is None):
            raise ValueError(
                "--front-image-width and --front-image-height must be supplied together"
            )
        if args.front_image_width is not None:
            if min(args.front_image_width, args.front_image_height) <= 0:
                raise ValueError("front image dimensions must be positive")
            args.sensor["front_stereo"]["image_width"] = args.front_image_width
            args.sensor["front_stereo"]["image_height"] = args.front_image_height
        if args.front_image_rate_hz is not None:
            if args.front_image_rate_hz <= 0.0:
                raise ValueError("front image rate must be positive")
            args.sensor["front_stereo"]["image_rate_hz"] = args.front_image_rate_hz
        args.sensor["front_stereo"]["image_rate_hz"] = 10.0
        args.sensor["surround_stereo"]["image_rate_hz"] = 10.0
        args.sensor["qos_reliability"] = (
            "reliable" if args.reliable_sensor_qos else "bestEffort"
        )
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    if args.dynamic_profile:
        parser.error("dynamic obstacle profiles are not supported by the Kujiale static scope")
    if args.static_profile:
        if not args.scenario_config.is_file():
            parser.error(f"scenario config does not exist: {args.scenario_config}")
        try:
            scenarios = load_mapping(args.scenario_config, "scenario config")
            static_profiles = scenarios.get("static_profiles", {})
            if args.static_profile and (
                not isinstance(static_profiles, dict)
                or args.static_profile not in static_profiles
            ):
                raise ValueError(
                    f"static profile {args.static_profile!r} is not configured"
                )
            args.static_scenario = (
                static_profiles[args.static_profile] if args.static_profile else None
            )
            args.dynamic_scenario = None
            if args.static_scenario is not None and not isinstance(args.static_scenario, dict):
                raise ValueError("selected static profile must be a mapping")
            if args.dynamic_scenario is not None and not isinstance(args.dynamic_scenario, dict):
                raise ValueError("selected dynamic profile must be a mapping")
        except (OSError, ValueError, yaml.YAMLError) as exc:
            parser.error(str(exc))
    else:
        args.static_scenario = None
        args.dynamic_scenario = None
    return args


def write_report(path: Path, report: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def run(args: argparse.Namespace) -> int:
    # SimulationApp must exist before importing any other Isaac Sim or pxr module.
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": args.headless,
            "hide_ui": args.headless,
            # Headless sensor render products do not depend on the editor
            # viewport.  Leaving its updates enabled renders an additional
            # 1280x720 view every application frame for no ROS consumer.
            "disable_viewport_updates": args.headless,
            "renderer": args.renderer,
            "width": args.width,
            "height": args.height,
        }
    )

    import omni.timeline

    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.utils.extensions import enable_extension

    from jackal_sim.graphs import create_control_graphs
    from jackal_sim.contact_monitor import RobotContactMonitor
    from jackal_sim.follow_camera import FollowCamera, activate_viewport_camera
    from jackal_sim.dynamic_obstacles import DynamicObstacleManager
    from jackal_sim.static_obstacles import StaticObstacleManager
    from jackal_sim.articulation_runtime import (
        ArticulationRuntime,
        articulation_physics_config_from_mapping,
    )
    from jackal_sim.idle_brake import IdleBrake
    from jackal_sim.skid_steer_motion_assist import SkidSteerMotionAssist
    from jackal_sim.sensors import create_sensor_graphs
    from jackal_sim.runtime import (
        stage_identity,
        unexpected_robot_overlaps,
        world_translation,
    )
    from jackal_sim.stage import (
        ROBOT_PRIM_PATH,
        compose_robot,
        fingerprint,
        open_environment_once,
    )

    timeline = None
    follow_camera = None
    dynamic_obstacles = None
    static_obstacles = None
    follow_camera_bindings = 0
    contact_monitor = None
    performance_benchmark = None
    ros_runtime_node = None
    rclpy_module = None
    owns_rclpy_context = False
    idle_brake = None
    motion_assist = None
    idle_brake_updates = 0
    motion_assist_updates = 0
    report: dict[str, object] = {
        "status": "failed",
        "mode": "headless" if args.headless else "gui",
        "pid": os.getpid(),
        "stage_open_count": 0,
        "config": str(args.config.resolve()),
        "headless_viewport_updates_disabled": bool(args.headless),
    }
    environment_before = fingerprint(args.environment_usd)
    robot_before = fingerprint(args.robot_usd)
    hawk_before = fingerprint(args.hawk_usd)
    report["assets_before"] = {
        "environment": asdict(environment_before),
        "robot": asdict(robot_before),
        "hawk": asdict(hawk_before),
    }
    started_wall = time.monotonic()
    try:
        if not args.disable_ros_control or not args.disable_sensors:
            if os.environ.get("ROS_DISTRO") != "jazzy":
                raise RuntimeError(
                    "ROS 2 Jazzy must be sourced before enabling runtime ROS graphs"
                )
            enable_extension("isaacsim.ros2.bridge")
            enable_extension("isaacsim.robot.wheeled_robots")
            enable_extension("isaacsim.sensors.physics.nodes")
            enable_extension("isaacsim.sensors.camera")
            if args.benchmark_performance:
                enable_extension("isaacsim.benchmark.services")
            app.update()
        stage = open_environment_once(app, args.environment_usd)
        report["stage_open_count"] = 1
        active_stage_identity = stage_identity()
        _, spawn, composition = compose_robot(
            stage,
            args.robot_usd,
            args.hawk_usd,
            args.control,
            physics_hz=args.physics_hz,
            spawn_clearance=args.spawn_clearance,
            spawn_grid_resolution=args.spawn_grid_resolution,
            spawn_height=args.spawn_height,
            spawn_obstacle_height=args.spawn_obstacle_height,
            spawn_preferred_xy=args.spawn_preferred_xy,
            spawn_yaw=args.spawn_yaw,
        )
        report.update(
            {
                "composition": composition,
                "camera_profile": args.camera_profile,
                "active_image_streams": (
                    8 if args.camera_profile == "mapping_8cam" else 6
                ),
            }
        )
        # Match the known-good Jackal runtime from the reference project.  An
        # authored PhysX timeStepsPerSecond value alone does not update Isaac
        # Sim's active SimulationManager clock; the motion assist would then
        # integrate with a different dt than the physics engine.
        physics_scene_path = str(composition["physics_scene"])
        requested_physics_dt = 1.0 / float(args.physics_hz)
        SimulationManager.set_physics_dt(
            requested_physics_dt,
            physics_scene=physics_scene_path,
        )
        effective_physics_dt = float(
            SimulationManager.get_physics_dt(physics_scene=physics_scene_path)
        )
        if not math.isclose(
            effective_physics_dt,
            requested_physics_dt,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise RuntimeError(
                "Isaac SimulationManager physics dt mismatch: "
                f"requested={requested_physics_dt} effective={effective_physics_dt}"
            )
        report["physics_timing"] = {
            "scene": physics_scene_path,
            "requested_hz": float(args.physics_hz),
            "requested_dt_s": requested_physics_dt,
            "effective_dt_s": effective_physics_dt,
            "explicit_simulation_manager_configuration": True,
            "reference": "codex/kujiale-navigation-mapping@caae0c08",
        }
        app.update()
        if stage_identity() != active_stage_identity:
            raise RuntimeError("active stage changed after Jackal composition")

        if args.disable_ros_control:
            report["control_graphs"] = {"enabled": False}
        else:
            graph_summary = create_control_graphs(stage, args.control)
            app.update()
            report["control_graphs"] = {
                "enabled": True,
                **graph_summary.to_dict(),
                "official_ros_sample_loaded": False,
            }
            if stage_identity() != active_stage_identity:
                raise RuntimeError("active stage changed while creating Phase 3 graphs")

        if args.disable_sensors:
            report["sensor_graphs"] = {"enabled": False}
        else:
            sensor_summary = create_sensor_graphs(
                stage,
                args.sensor,
                camera_profile=args.camera_profile,
            )
            app.update()
            report["sensor_graphs"] = {
                "enabled": True,
                **sensor_summary.to_dict(),
                "official_ros_sample_loaded": False,
            }
            if stage_identity() != active_stage_identity:
                raise RuntimeError("active stage changed while creating Phase 4 graphs")

        if args.static_scenario is not None:
            static_obstacles = StaticObstacleManager(
                stage,
                args.static_profile,
                args.static_scenario,
                (spawn.x, spawn.y, spawn.z),
            )
            app.update()
            report["static_obstacles"] = static_obstacles.summary()
        else:
            report["static_obstacles"] = {"enabled": False}

        if args.dynamic_scenario is not None:
            dynamic_obstacles = DynamicObstacleManager(
                stage,
                args.dynamic_profile,
                args.dynamic_scenario,
                (spawn.x, spawn.y, spawn.z),
            )
            app.update()
            report["dynamic_obstacles"] = dynamic_obstacles.summary()
        else:
            report["dynamic_obstacles"] = {"enabled": False}

        # Build the contact report-pair whitelist after optional session-layer
        # obstacles exist so static acceptance obstacles cannot be omitted.
        contact_monitor = RobotContactMonitor(stage, ROBOT_PRIM_PATH)
        app.update()

        if args.gui and not args.disable_follow_camera:
            follow_camera = FollowCamera(stage, f"{ROBOT_PRIM_PATH}/base_link")
            app.update()
            activate_viewport_camera()
            report["follow_camera"] = {
                "enabled": True,
                "prim": "/World/FollowCameraRig",
                "viewport_active": True,
                "distance_m": 3.0,
                "height_m": 1.8,
            }
        else:
            report["follow_camera"] = {
                "enabled": False,
                "headless": args.headless,
                "viewport_dependency_created": False,
            }

        timeline = omni.timeline.get_timeline_interface()
        # Imported static USD stages have no authored time range, so Kit's
        # default timeline is [0, 0] with looping enabled.  Playing that range
        # repeatedly resets simulation time and every time-stamped ROS topic.
        timeline.set_start_time(0.0)
        timeline.set_end_time(86_400.0)
        timeline.set_looping(False)
        timeline.set_time_codes_per_second(float(args.physics_hz))
        timeline.set_target_framerate(float(args.update_hz))
        timeline.commit()
        report["timeline"] = {
            "start_time_s": float(timeline.get_start_time()),
            "end_time_s": float(timeline.get_end_time()),
            "looping": bool(timeline.is_looping()),
            "time_codes_per_second": float(
                timeline.get_time_codes_per_seconds()
            ),
            "target_framerate_hz": float(args.update_hz),
        }
        # Mirror the proven reference lifecycle: stop, warm exactly two
        # physics updates, then initialize and configure the articulation while
        # paused.  Starting the tensor articulation after a longer live settle
        # leaves stale wheel contacts and produces severe skid-steer understeer.
        timeline.stop()
        app.update()
        timeline.play()
        app.update()
        app.update()
        timeline.pause()
        app.update()

        if not args.disable_ros_control:
            import rclpy
            rclpy_module = rclpy
            if not rclpy.ok():
                rclpy.init(args=[])
                owns_rclpy_context = True
            ros_runtime_node = rclpy.create_node("jackal_sim_runtime")
            articulation_settings = articulation_physics_config_from_mapping(
                args.control
            )
            robot_runtime = ArticulationRuntime(
                ROBOT_PRIM_PATH,
                f"{ROBOT_PRIM_PATH}/base_link",
                app,
            )
            robot_runtime.initialize()
            robot_runtime.configure_stability(articulation_settings)
            command_topic = str(args.control["topics"]["command_to_sim"])
            simulation_clock = lambda: float(  # noqa: E731 - injected callback
                SimulationManager.get_simulation_time()
            )
            idle_brake = IdleBrake(
                ros_runtime_node,
                robot_runtime,
                articulation_settings,
                topic_name=command_topic,
                clock=simulation_clock,
            )
            motion_assist = SkidSteerMotionAssist(
                ros_runtime_node,
                robot_runtime,
                articulation_settings,
                physics_dt=1.0 / float(args.physics_hz),
                topic_name=command_topic,
                clock=simulation_clock,
            )
            # Preserve the reference project's proven reset lifecycle.  The
            # USD transform selects the authored spawn before physics starts;
            # this paused tensor reset then makes the same pose authoritative
            # for the live floating articulation and advances one physics step
            # before accepting velocity commands.
            robot_runtime.set_world_pose(
                [spawn.x, spawn.y, spawn.z],
                [
                    math.cos(spawn.yaw_radians * 0.5),
                    0.0,
                    0.0,
                    math.sin(spawn.yaw_radians * 0.5),
                ],
            )
            robot_runtime.zero_all_velocities()
            motion_assist.reset()
            SimulationManager.step(steps=1, update_fabric=False)
            timeline.play()
            app.update()
            report["skid_steer_runtime"] = {
                "enabled": True,
                "command_topic": command_topic,
                "dof_names": robot_runtime.get_dof_names(),
                "idle_brake_timeout_s": (
                    articulation_settings.idle_brake_command_timeout_sec
                ),
                "motion_assist_enabled": articulation_settings.motion_assist_enabled,
                "motion_assist_timeout_s": (
                    articulation_settings.motion_assist_command_timeout_sec
                ),
                "effective_wheel_separation_m": float(
                    args.control["kinematics"]["wheel_separation_m"]
                ),
                "reference_reset_lifecycle": True,
            }
        else:
            report["skid_steer_runtime"] = {"enabled": False}
            timeline.play()
            app.update()

        if follow_camera is not None:
            activate_viewport_camera()
            follow_camera_bindings += 1

        start_simulation_time = float(timeline.get_current_time())
        last_simulation_time = start_simulation_time
        maximum_simulation_time = start_simulation_time
        start_pose = world_translation(stage, f"{ROBOT_PRIM_PATH}/base_link")
        initial_overlaps = unexpected_robot_overlaps(
            robot_prim_path=ROBOT_PRIM_PATH,
            footprint_aabb=spawn.footprint_aabb,
            floor_z=spawn.z - args.spawn_height,
        )
        if initial_overlaps:
            raise RuntimeError(f"PhysX found initial obstacle overlap: {initial_overlaps}")
        ready_fields = (
            f"mode={report['mode']} spawn=({spawn.x:.3f},{spawn.y:.3f},{spawn.z:.3f}) "
            f"control={not args.disable_ros_control} sensors={not args.disable_sensors} "
            f"camera_profile={args.camera_profile} streams={8 if args.camera_profile == 'mapping_8cam' else 6} "
            "lidar=false"
        )
        print(f"JACKAL_CONTROL_READY {ready_fields}", flush=True)
        print(f"JACKAL_SENSORS_READY {ready_fields}", flush=True)

        if args.benchmark_performance:
            from jackal_sim.performance import (
                AdaptiveOfficialBenchmark,
                AdaptiveSamplingConfig,
            )

            performance_benchmark = AdaptiveOfficialBenchmark(
                AdaptiveSamplingConfig(
                    minimum_warmup_s=args.performance_min_warmup_s,
                    maximum_warmup_s=args.performance_max_warmup_s,
                    stability_window_s=args.performance_window_s,
                    stable_windows_required=args.performance_stable_windows,
                    maximum_mean_change_ratio=args.performance_max_mean_change,
                    maximum_coefficient_of_variation=args.performance_max_cv,
                    minimum_sample_s=args.performance_min_sample_s,
                    maximum_sample_s=args.performance_max_sample_s,
                ),
                camera_profile=args.camera_profile,
                start_file=args.performance_start_file,
            )
            print(
                "JACKAL_PERFORMANCE_READY "
                f"profile={args.camera_profile} fixed_frames=none",
                flush=True,
            )

        run_started = time.monotonic()
        deadline = run_started + args.duration if args.duration > 0.0 else None
        frames = 0
        sampled_time_regressions = 0
        significant_time_regressions = 0
        maximum_time_regression_s = 0.0
        time_regression_tolerance_s = 1.5 / float(args.physics_hz)
        while app.is_running() and not STOP_REQUESTED:
            frame_started = time.monotonic()
            app.update()
            if ros_runtime_node is not None:
                rclpy_module.spin_once(ros_runtime_node, timeout_sec=0.0)
                if idle_brake.update():
                    idle_brake_updates += 1
                elif motion_assist.update():
                    motion_assist_updates += 1
            if dynamic_obstacles is not None:
                dynamic_obstacles.update(float(timeline.get_current_time()))
            if follow_camera is not None:
                follow_camera.update(1.0 / args.update_hz)
                if frames % 30 == 0:
                    activate_viewport_camera()
                    follow_camera_bindings += 1
            frames += 1
            current_simulation_time = float(timeline.get_current_time())
            if current_simulation_time + 1.0e-9 < last_simulation_time:
                sampled_time_regressions += 1
                regression_s = last_simulation_time - current_simulation_time
                maximum_time_regression_s = max(maximum_time_regression_s, regression_s)
                if regression_s > time_regression_tolerance_s:
                    significant_time_regressions += 1
            last_simulation_time = current_simulation_time
            maximum_simulation_time = max(maximum_simulation_time, current_simulation_time)
            if not timeline.is_playing():
                raise RuntimeError("timeline stopped before shutdown was requested")
            if stage_identity() != active_stage_identity:
                raise RuntimeError("active stage changed during the simulation loop")
            if deadline is not None and time.monotonic() >= deadline:
                break
            if args.stop_file is not None and args.stop_file.exists():
                break
            if performance_benchmark is not None:
                performance_benchmark.tick()
                if performance_benchmark.completed:
                    break
            remaining = 1.0 / args.update_hz - (time.monotonic() - frame_started)
            if remaining > 0.0:
                time.sleep(remaining)

        if frames == 0:
            raise RuntimeError("simulation loop completed without advancing any frames")
        elapsed_run_wall = time.monotonic() - run_started
        stopped_by_file = args.stop_file is not None and args.stop_file.exists()
        if (
            args.duration > 0.0
            and elapsed_run_wall + 0.05 < args.duration
            and not STOP_REQUESTED
            and not stopped_by_file
        ):
            raise RuntimeError(
                f"application stopped before the requested duration: "
                f"requested={args.duration:.3f}s actual={elapsed_run_wall:.3f}s"
            )
        if maximum_simulation_time <= start_simulation_time:
            raise RuntimeError("simulation time did not advance")
        if significant_time_regressions:
            raise RuntimeError(
                "simulation time regressed by more than "
                f"{time_regression_tolerance_s:.6f}s on "
                f"{significant_time_regressions} frames"
            )

        final_pose = world_translation(stage, f"{ROBOT_PRIM_PATH}/base_link")
        final_overlaps = unexpected_robot_overlaps(
            robot_prim_path=ROBOT_PRIM_PATH,
            footprint_aabb=spawn.footprint_aabb,
            floor_z=spawn.z - args.spawn_height,
        )
        if final_overlaps:
            raise RuntimeError(f"PhysX found final obstacle overlap: {final_overlaps}")
        if abs(final_pose[2] - start_pose[2]) > 0.15:
            raise RuntimeError(
                f"robot vertical pose drifted unexpectedly: start={start_pose}, final={final_pose}"
            )
        report.update(
            {
                "status": "passed",
                "runtime": {
                    "frames": frames,
                    "wall_seconds": elapsed_run_wall,
                    "simulation_time_start": start_simulation_time,
                    "simulation_time_end": maximum_simulation_time,
                    "simulation_time_last_sample": last_simulation_time,
                    "simulation_time_delta": maximum_simulation_time - start_simulation_time,
                    "sampled_time_regressions": sampled_time_regressions,
                    "significant_time_regressions": significant_time_regressions,
                    "maximum_time_regression_s": maximum_time_regression_s,
                    "time_regression_tolerance_s": time_regression_tolerance_s,
                    "timeline_playing_during_probe": True,
                    "start_chassis_translation": start_pose,
                    "final_chassis_translation": final_pose,
                    "initial_unexpected_overlaps": initial_overlaps,
                    "final_unexpected_overlaps": final_overlaps,
                },
            }
        )
        if performance_benchmark is not None:
            if not performance_benchmark.completed:
                performance_benchmark.stop_incomplete()
            report["performance"] = performance_benchmark.report()
            if not report["performance"]["completed"]:
                raise RuntimeError("adaptive performance sample did not complete")
        if follow_camera is not None:
            report["follow_camera"].update(
                {
                    "viewport_bindings": follow_camera_bindings,
                    **follow_camera.state(),
                }
            )
        if report["skid_steer_runtime"]["enabled"]:
            report["skid_steer_runtime"].update(
                {
                    "idle_brake_updates": idle_brake_updates,
                    "motion_assist_updates": motion_assist_updates,
                }
            )
        print(
            "JACKAL_SIM_COMPLETED "
            f"mode={report['mode']} spawn=({spawn.x:.3f},{spawn.y:.3f},{spawn.z:.3f}) "
            f"control={not args.disable_ros_control} sensors={not args.disable_sensors} "
            f"frames={frames} "
            f"sim_delta={maximum_simulation_time - start_simulation_time:.3f}",
            flush=True,
        )
        return 0
    except Exception as exc:  # noqa: BLE001 - persist complete standalone diagnostics
        report["status"] = "failed"
        if performance_benchmark is not None and "performance" not in report:
            performance_benchmark.stop_incomplete()
            report["performance"] = performance_benchmark.report()
        report["error"] = str(exc)
        report["traceback"] = traceback.format_exc()
        print(report["traceback"], file=sys.stderr, flush=True)
        return 1
    finally:
        if report.get("skid_steer_runtime", {}).get("enabled"):
            report["skid_steer_runtime"].update(
                {
                    "idle_brake_updates": idle_brake_updates,
                    "motion_assist_updates": motion_assist_updates,
                }
            )
        if ros_runtime_node is not None:
            ros_runtime_node.destroy_node()
        if (
            owns_rclpy_context
            and rclpy_module is not None
            and rclpy_module.ok()
        ):
            rclpy_module.shutdown()
        if dynamic_obstacles is not None:
            report["dynamic_obstacles"] = dynamic_obstacles.summary()
            dynamic_obstacles.close()
        if contact_monitor is not None:
            report["robot_contacts"] = contact_monitor.summary()
            contact_monitor.close()
        if timeline is not None:
            timeline.stop()
            app.update()
            report["timeline_stopped_on_exit"] = not timeline.is_playing()
        environment_after = fingerprint(args.environment_usd)
        robot_after = fingerprint(args.robot_usd)
        hawk_after = fingerprint(args.hawk_usd)
        report["assets_after"] = {
            "environment": asdict(environment_after),
            "robot": asdict(robot_after),
            "hawk": asdict(hawk_after),
        }
        report["official_assets_unchanged"] = (
            environment_before == environment_after
            and robot_before == robot_after
            and hawk_before == hawk_after
        )
        report["total_wall_seconds"] = time.monotonic() - started_wall
        if not report["official_assets_unchanged"]:
            report["status"] = "failed"
            report["error"] = "an official USD asset changed during the run"
        write_report(args.report, report)
        print(f"SIM_REPORT {args.report.resolve()}", flush=True)
        app.close(exit_code=0 if report["status"] == "passed" else 1)


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        with SimulatorProcessLock(args.lock_file, args.lock_wait_seconds):
            status = run(args)
            if status == 0:
                persisted = json.loads(args.report.read_text(encoding="utf-8"))
                if persisted.get("status") != "passed":
                    print(
                        f"error: persisted simulation report is not passed: {args.report}",
                        file=sys.stderr,
                    )
                    return 1
            return status
    except SimulatorAlreadyRunning as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 73


if __name__ == "__main__":
    raise SystemExit(main())
