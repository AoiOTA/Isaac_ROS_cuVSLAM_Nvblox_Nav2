#!/usr/bin/env python3
"""Isaac Sim 6.0.1 standalone entry point for the fixed warehouse and Nova Carter."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import signal
import sys
import time
import traceback

import yaml

from nova_carter_sim.process_lock import SimulatorAlreadyRunning, SimulatorProcessLock


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
    variants = config.get("variants", {})
    if not isinstance(stage, dict) or not isinstance(variants, dict):
        raise ValueError("stage and variants config entries must be mappings")
    if stage.get("robot_prim") != "/World/NovaCarter":
        raise ValueError("simulation config must use robot prim /World/NovaCarter")
    if stage.get("compose_in_session_layer") is not True:
        raise ValueError("stage composition must remain in the session layer")
    if stage.get("save_composed_stage") is not False:
        raise ValueError("saving the composed official stage is forbidden")
    if stage.get("stage_open_count") != 1:
        raise ValueError("the warehouse stage must be opened exactly once")
    expected_variants = {
        "Physics": "physx",
        "Sensors": "All_Sensors",
        "ROS": "Disabled",
    }
    if variants != expected_variants:
        raise ValueError(f"simulation variants must remain fixed: {expected_variants}")
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
        description="Open the official warehouse once and compose Nova Carter in memory"
    )
    parser.add_argument("--config", type=Path, default=pre_args.config)
    parser.add_argument(
        "--control-config", type=Path, default=PROJECT_ROOT / "config/control.yaml"
    )
    parser.add_argument(
        "--sensor-config", type=Path, default=PROJECT_ROOT / "config/sensors.yaml"
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--headless", action="store_true")
    mode.add_argument("--gui", action="store_true")
    parser.add_argument(
        "--warehouse-usd", type=Path, default=Path(os.environ.get("WAREHOUSE_USD", ""))
    )
    parser.add_argument(
        "--robot-usd", type=Path, default=Path(os.environ.get("NOVA_CARTER_USD", ""))
    )
    parser.add_argument(
        "--forbidden-ros-sample-usd",
        type=Path,
        default=Path(os.environ.get("NOVA_CARTER_ROS_SAMPLE_USD", "")),
    )
    parser.add_argument(
        "--duration", type=float, default=0.0, help="wall seconds; 0 runs until stopped"
    )
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
    parser.add_argument("--spawn-yaw", type=float, default=0.0, help="initial yaw in radians")
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
        "--disable-follow-camera",
        action="store_true",
        help="keep the default GUI viewport camera instead of the smooth robot follower",
    )
    parser.add_argument(
        "--lock-file", type=Path, default=lock_path
    )
    args = parser.parse_args()
    if args.duration < 0.0:
        parser.error("--duration must be non-negative")
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
        ("warehouse", args.warehouse_usd),
        ("Nova Carter", args.robot_usd),
        ("forbidden ROS sample", args.forbidden_ros_sample_usd),
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
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))
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
            "renderer": args.renderer,
            "width": args.width,
            "height": args.height,
        }
    )

    import omni.timeline

    from isaacsim.core.utils.extensions import enable_extension

    from nova_carter_sim.graphs import create_control_graphs
    from nova_carter_sim.follow_camera import FollowCamera, activate_viewport_camera
    from nova_carter_sim.sensors import create_sensor_graphs
    from nova_carter_sim.runtime import (
        stage_identity,
        unexpected_robot_overlaps,
        world_translation,
    )
    from nova_carter_sim.stage import (
        ROBOT_PRIM_PATH,
        compose_robot,
        fingerprint,
        open_warehouse_once,
    )

    timeline = None
    follow_camera = None
    report: dict[str, object] = {
        "status": "failed",
        "mode": "headless" if args.headless else "gui",
        "pid": os.getpid(),
        "stage_open_count": 0,
        "config": str(args.config.resolve()),
    }
    warehouse_before = fingerprint(args.warehouse_usd)
    robot_before = fingerprint(args.robot_usd)
    report["assets_before"] = {
        "warehouse": asdict(warehouse_before),
        "robot": asdict(robot_before),
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
            app.update()
        stage = open_warehouse_once(app, args.warehouse_usd)
        report["stage_open_count"] = 1
        active_stage_identity = stage_identity()
        _, spawn, composition = compose_robot(
            stage,
            args.robot_usd,
            args.forbidden_ros_sample_usd,
            physics_hz=args.physics_hz,
            spawn_clearance=args.spawn_clearance,
            spawn_grid_resolution=args.spawn_grid_resolution,
            spawn_height=args.spawn_height,
            spawn_obstacle_height=args.spawn_obstacle_height,
            spawn_preferred_xy=args.spawn_preferred_xy,
            spawn_yaw=args.spawn_yaw,
        )
        app.update()
        if stage_identity() != active_stage_identity:
            raise RuntimeError("active stage changed after Nova Carter composition")

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
            sensor_summary = create_sensor_graphs(stage, args.sensor)
            app.update()
            report["sensor_graphs"] = {
                "enabled": True,
                **sensor_summary.to_dict(),
                "official_ros_sample_loaded": False,
            }
            if stage_identity() != active_stage_identity:
                raise RuntimeError("active stage changed while creating Phase 4 graphs")

        if args.gui and not args.disable_follow_camera:
            follow_camera = FollowCamera(stage, f"{ROBOT_PRIM_PATH}/chassis_link")
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
        timeline.set_time_codes_per_second(float(args.physics_hz))
        timeline.play()
        for _ in range(10):
            app.update()

        start_simulation_time = float(timeline.get_current_time())
        last_simulation_time = start_simulation_time
        start_pose = world_translation(stage, f"{ROBOT_PRIM_PATH}/chassis_link")
        initial_overlaps = unexpected_robot_overlaps(
            robot_prim_path=ROBOT_PRIM_PATH,
            footprint_aabb=spawn.footprint_aabb,
            floor_z=spawn.z - args.spawn_height,
        )
        if initial_overlaps:
            raise RuntimeError(f"PhysX found initial obstacle overlap: {initial_overlaps}")
        ready_fields = (
            f"mode={report['mode']} spawn=({spawn.x:.3f},{spawn.y:.3f},{spawn.z:.3f}) "
            f"control={not args.disable_ros_control} sensors={not args.disable_sensors}"
        )
        print(f"NOVA_CARTER_CONTROL_READY {ready_fields}", flush=True)
        print(f"NOVA_CARTER_SENSORS_READY {ready_fields}", flush=True)

        run_started = time.monotonic()
        deadline = run_started + args.duration if args.duration > 0.0 else None
        frames = 0
        time_regressions = 0
        while app.is_running() and not STOP_REQUESTED:
            frame_started = time.monotonic()
            app.update()
            if follow_camera is not None:
                follow_camera.update(1.0 / args.update_hz)
            frames += 1
            current_simulation_time = float(timeline.get_current_time())
            if current_simulation_time + 1.0e-9 < last_simulation_time:
                time_regressions += 1
            last_simulation_time = current_simulation_time
            if not timeline.is_playing():
                raise RuntimeError("timeline stopped before shutdown was requested")
            if stage_identity() != active_stage_identity:
                raise RuntimeError("active stage changed during the simulation loop")
            if deadline is not None and time.monotonic() >= deadline:
                break
            if args.stop_file is not None and args.stop_file.exists():
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
        if last_simulation_time <= start_simulation_time:
            raise RuntimeError("simulation time did not advance")
        if time_regressions:
            raise RuntimeError(f"simulation time regressed on {time_regressions} frames")

        final_pose = world_translation(stage, f"{ROBOT_PRIM_PATH}/chassis_link")
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
                "composition": composition,
                "runtime": {
                    "frames": frames,
                    "wall_seconds": elapsed_run_wall,
                    "simulation_time_start": start_simulation_time,
                    "simulation_time_end": last_simulation_time,
                    "simulation_time_delta": last_simulation_time - start_simulation_time,
                    "time_regressions": time_regressions,
                    "timeline_playing_during_probe": True,
                    "start_chassis_translation": start_pose,
                    "final_chassis_translation": final_pose,
                    "initial_unexpected_overlaps": initial_overlaps,
                    "final_unexpected_overlaps": final_overlaps,
                },
            }
        )
        print(
            "NOVA_CARTER_SIM_COMPLETED "
            f"mode={report['mode']} spawn=({spawn.x:.3f},{spawn.y:.3f},{spawn.z:.3f}) "
            f"control={not args.disable_ros_control} sensors={not args.disable_sensors} "
            f"frames={frames} "
            f"sim_delta={last_simulation_time - start_simulation_time:.3f}",
            flush=True,
        )
        return 0
    except Exception as exc:  # noqa: BLE001 - persist complete standalone diagnostics
        report["error"] = str(exc)
        report["traceback"] = traceback.format_exc()
        print(report["traceback"], file=sys.stderr, flush=True)
        return 1
    finally:
        if timeline is not None:
            timeline.stop()
            app.update()
            report["timeline_stopped_on_exit"] = not timeline.is_playing()
        warehouse_after = fingerprint(args.warehouse_usd)
        robot_after = fingerprint(args.robot_usd)
        report["assets_after"] = {
            "warehouse": asdict(warehouse_after),
            "robot": asdict(robot_after),
        }
        report["official_assets_unchanged"] = (
            warehouse_before == warehouse_after and robot_before == robot_after
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
        with SimulatorProcessLock(args.lock_file):
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
