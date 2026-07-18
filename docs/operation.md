# Operation

Available through Phase 11:

```bash
./scripts/build.sh
./scripts/bootstrap_baremetal.sh --preflight
./scripts/smoke_ros_bridge.sh
python3 tools/check_stage1.py
./scripts/smoke_isaac_ros_nodes.sh
./scripts/run_sim.sh --headless
./scripts/run_control.sh
./scripts/run_phase3_tests.sh
./scripts/run_sensors.sh
./scripts/run_phase4_tests.sh
./scripts/run_visual_slam.sh
./scripts/run_phase5_tests.sh
./scripts/run_nvblox.sh
./scripts/run_phase6.sh
./scripts/save_nvblox_map.sh
./scripts/run_phase6_tests.sh
./scripts/run_mapping.sh --map warehouse_v1
./scripts/run_phase7_tests.sh warehouse_v1
./scripts/run_navigation.sh --map warehouse_v1 --rviz
./scripts/run_all.sh --map warehouse_v1 --headless --rviz
./scripts/run_phase8_tests.sh warehouse_v1
./scripts/run_phase9_mapping.sh --map warehouse_v2_front
./scripts/run_phase9_navigation.sh --map warehouse_v2_front --rviz
./scripts/run_phase9.sh --map warehouse_v2_front --headless --rviz
./scripts/run_phase9_tests.sh warehouse_v2_front
./scripts/run_phase10_trial.sh --class dynamic --seed 11000 --goal-index 0 --record-bag
./scripts/run_phase10_hardening.sh warehouse_v2_front
./scripts/run_phase10_preacceptance.sh --map warehouse_v2_front
./scripts/run_phase10_tests.sh warehouse_v2_front
./scripts/prepare_stage11_reference.sh --map warehouse_v2_front
./scripts/run_stage11_trial.sh --class heterogeneous --seed 41000 --goal-index 5 --record-bag
./scripts/run_stage11_smoke.sh warehouse_v2_front
./scripts/run_stage11_tests.sh warehouse_v2_front
./scripts/run_acceptance.sh --matrix-id phase11-formal-20260718 --record-bag
```

`smoke_ros_bridge.sh` starts only its own Isaac Sim process group, receives real `/clock`, image, and CameraInfo messages, and then terminates that process group. It never uses `killall` or `pkill`.

`smoke_isaac_ros_nodes.sh` loads cuVSLAM, nvblox, and VGL one at a time. A component passes only if it remains alive in its waiting-for-input state for the complete smoke interval; an early exit is reported with its log.

## Phase 2 standalone simulator

The implemented simulator entry points are:

```bash
./scripts/run_sim.sh --headless
./scripts/run_sim.sh --gui
```

Both commands derive the repository root, load `config/environment.env`, validate the three asset paths, and invoke Isaac Sim with the configured Conda interpreter. Add `--duration SECONDS` for a bounded test. Without it, the program runs until Ctrl-C or the GUI closes.

The simulator performs these operations in order:

1. Acquire `data/locks/navigation_sim.lock`. The advisory lock only prevents duplicate simulator instances from this repository.
2. Create `SimulationApp` before importing Isaac Sim runtime modules.
3. Open the fixed Warehouse USD exactly once.
4. Inspect collision-floor and obstacle bounds and choose a padded, floor-supported spawn.
5. Add `/World/NovaCarter` as a reference to the main robot USD in the anonymous session layer.
6. Select `Physics=physx`, `Sensors=All_Sensors`, and `ROS=Disabled`.
7. Verify the wheel joints, chassis, cameras, absence of robot OmniGraph, and absence of `Nova_Carter_ROS.usd` from the layer stack.
8. Start the timeline and monitor stage identity, monotonic simulation time, chassis height, and PhysX overlaps.
9. Stop the timeline, write a JSON report, and close `SimulationApp`.

The composed stage is never saved. A successful report can be checked independently:

```bash
python3 tools/check_stage2_report.py data/logs/stage2/latest.json
```

## Phase 3 differential drive

`run_sim.sh` now enables the ROS 2 Bridge, Wheeled Robots, physics sensor nodes, and four project-owned runtime graphs after composing the Phase 2 stage. It publishes `/clock`, `/joint_states`, and `/ground_truth/odometry`, and subscribes to `/cmd_vel_sim`.

In a second terminal, run:

```bash
./scripts/run_control.sh
```

This launches the Command Guard and `/wheel/odometry`. A manual smoke command can then be sent with:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 topic pub --rate 20 /cmd_vel_safe geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

Use the automated suite for acceptance instead of judging motion by eye:

```bash
./scripts/run_phase3_tests.sh
```

The suite starts the simulator and ROS nodes in process groups it owns, uses ROS domain 43 by default for test isolation, records JSON and logs, and shuts down through a simulator stop sentinel. Override only the temporary test domain with `PHASE3_TEST_ROS_DOMAIN_ID`; the public project domain remains 42.

Mapping, relocalization, static/dynamic navigation, experiment hardening and formal front-stereo acceptance now have public entries through Stage 11. Lighting and color randomization are intentionally outside the accepted Stage 11 scope.

## Phase 4 visual sensor data flow

`run_sim.sh` now also authors `FrontStereo`, `FrontDepth`, and `FrontImu` under `/World/Graphs`. In a separate terminal, start the below-`base_link` TF tree and Isaac ROS mono conversion with:

```bash
./scripts/run_sensors.sh
```

The complete Phase 4 bringup used by automation is `ros2 launch nova_carter_bringup phase4.launch.py`; it includes Phase 3 control, robot_state_publisher, and two GPU `ImageFormatConverterNode` components. The acceptance entry is:

```bash
./scripts/run_phase4_tests.sh
```

It uses isolated domain 44 by default. Override only for test isolation with `PHASE4_TEST_ROS_DOMAIN_ID`; normal project commands remain on domain 42. Reports are placed under `data/reports/phase4` and logs under `data/logs/stage4`.

## Phase 5 continuous visual localization

Start the Standalone simulator first, then start only the ROS visual-localization side in another terminal:

```bash
./scripts/run_sim.sh --headless
./scripts/run_visual_slam.sh
```

For the complete control plus localization bringup, use:

```bash
ros2 launch nova_carter_bringup phase5.launch.py
```

The launch composes the two GPU mono converters and `VisualSlamNode` in one multithreaded component container. cuVSLAM runs in stereo+IMU VIO mode and is the only publisher of `map→odom` and `odom→base_link`. Its public map interfaces are:

```bash
ros2 service call /visual_slam/save_map \
  isaac_ros_visual_slam_interfaces/srv/FilePath \
  "{file_path: '/absolute/path/to/cuvslam_map'}"

ros2 service call /visual_slam/load_map \
  isaac_ros_visual_slam_interfaces/srv/FilePath \
  "{file_path: '/absolute/path/to/cuvslam_map'}"

ros2 service call /visual_slam/get_all_poses \
  isaac_ros_visual_slam_interfaces/srv/GetAllPoses \
  "{max_count: 10000}"
```

Use the reproducible acceptance instead of manual driving:

```bash
./scripts/run_phase5_tests.sh
```

The suite uses isolated domain 45, starts and cleans only its own process groups, commands a repeated forward/reverse S-course for at least 122 simulation seconds, audits tracking state and TF ownership, compares direction and metric scale against ground truth, and exercises all three map interfaces. Reports are written under `data/reports/phase5`; detailed evidence is in `docs/phase5_validation.md`.

For a clean-machine manual setup, camera migration, parameter-by-parameter tuning, map services, and troubleshooting, follow [`docs/cuvslam_configuration.md`](cuvslam_configuration.md).

## Phase 6 nvblox reconstruction

Start the simulator in one terminal and the complete control, cuVSLAM, and nvblox stack in another:

```bash
./scripts/run_sim.sh --headless
./scripts/run_phase6.sh
```

`run_nvblox.sh` starts only the nvblox component and assumes the sensor streams and `odom→base_link→front_stereo_camera_left_optical` TF are already live. Phase 6 remaps one native 32FC1 depth stream and the front-left RGB stream into `nvblox::NvbloxNode`. The mapper uses `static_tsdf`, 5 cm voxels, `global_frame=odom`, 2D ESDF, no lidar, and SensorData QoS.

Save the current map, PLY, rates, and timings without RViz interaction:

```bash
./scripts/save_nvblox_map.sh \
  data/maps/warehouse_v1/nvblox \
  warehouse
```

Run the full isolated acceptance:

```bash
./scripts/run_phase6_tests.sh
```

The suite audits the effective runtime parameters, rate-limits a 60-second bidirectional S course to 20 Hz control, verifies healthy cuVSLAM tracking and nonempty TSDF/Mesh/ESDF/map-slice outputs, invokes all four save services, and checks the simulator for unexpected PhysX overlap. Results go under `data/reports/phase6`; see [`docs/phase6_validation.md`](phase6_validation.md).

## Stage 7 mapping and relocalization

```bash
./scripts/run_mapping.sh --map warehouse_v1
python3 tools/check_phase7_maps.py data/maps/warehouse_v1
./scripts/run_phase7_tests.sh warehouse_v1
```

The mapping entry owns only its own process groups, records MCAP, persists the online reconstruction, creates aligned cuVSLAM/cuVGL products, caches TensorRT engines, writes a manifest, and exits without GUI interaction. The acceptance entry starts five fresh simulator/ROS graphs, triggers cuVGL, relays each pose to cuVSLAM, and requires tracking recovery without a failed-localization hint. See [`docs/phase7_validation.md`](phase7_validation.md).

The mapper is intentionally in 2D ESDF mode. Do not call `get_esdf_and_gradient` in this mode on nvblox 4.5; it is a 3D-only service and terminates the node. Use `static_esdf_pointcloud` and `static_map_slice` for 2D validation.

For installation on another computer, input/TF contracts, complete copyable YAML and launch files, persistence, parameter tuning, dynamic-mode migration, Nav2 integration, and troubleshooting, follow [`docs/nvblox_configuration.md`](nvblox_configuration.md).

## Stage 8 navigation

The clean-terminal end-to-end entry is:

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/run_all.sh --map warehouse_v1 --headless --rviz
```

It starts the simulator and ROS launch in process groups owned by this repository, waits for sensor readiness, triggers cuVGL, relays the pose to cuVSLAM, waits for all nine Nav2 lifecycle nodes, executes three `NavigateToPose` goals, validates the data flow, writes a JSON report, and shuts down without signaling processes from other projects. `--no-rviz` tests the same core pipeline without the visualization process.

To run the ROS side against an already running project simulator:

```bash
./scripts/run_navigation.sh --map warehouse_v1 --rviz
```

The full regression is:

```bash
./scripts/run_phase8_tests.sh warehouse_v1
```

It additionally runs all package tests and a real GUI follow-camera smoke. Runtime products are stored in ignored directories under `data/logs/stage8`, `data/reports/phase8`, and `data/logs/stage4`.

Useful controlled overrides are:

```bash
PHASE8_SIM_UPDATE_HZ=120 ./scripts/run_all.sh --map warehouse_v1 --headless --no-rviz
PHASE8_GOAL_TIMEOUT_S=240 ./scripts/run_all.sh --map warehouse_v1 --headless --rviz
PHASE8_GOAL_POSES='[1.0, 0.0, 0.0]' ./scripts/run_all.sh --map warehouse_v1 --headless --no-rviz
```

The default simulator update cap is 120 Hz and the physics rate remains 120 Hz. `PHASE8_GOAL_POSES` is intended for bounded tuning runs; the persisted Stage 8 checker requires the default three-goal report. Full interfaces and measured results are in [`docs/phase8_validation.md`](phase8_validation.md).

## Stage 9 front-stereo dynamic navigation

The default one-command run is:

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/run_phase9.sh --map warehouse_v2_front --headless --rviz
```

It owns a loopback-only Fast DDS discovery server, simulator, ROS launch and
test runner. The simulator creates only `FrontStereo`, `FrontDepth` and
`FrontImu` sensor graphs, publishes the front pair at 10 Hz and IMU at 120 Hz,
and drives the `warehouse_crossing` dynamic profile. The ROS stack starts
front-only cuVSLAM/cuVGL, wheel-local EKF, dynamic nvblox, recovery manager,
resilient Nav2 action, complete Nav2, and optional RViz.

To run only the ROS half against this repository's already running simulator:

```bash
./scripts/run_phase9_navigation.sh --map warehouse_v2_front --rviz
```

To regenerate a front-only map using the same map pipeline as Stage 7:

```bash
./scripts/run_phase9_mapping.sh --map warehouse_v2_front
```

For reproducible regression, use:

```bash
PHASE9_TRIALS=3 ./scripts/run_phase9_tests.sh warehouse_v2_front
```

The first trial includes RViz and later trials omit it. Each trial injects one
forced relocalization during an active goal and requires a safe stop plus
automatic goal resume. Runtime reports are under `data/reports/phase9`; logs
are under `data/logs/stage9`. These directories are intentionally ignored by
Git. Full pass criteria and measured data are in
[`docs/phase9_validation.md`](phase9_validation.md).

## Stage 10 automation and hardening

Stage 10 retains the Stage 9 front-stereo-only runtime and explicitly leaves
lighting and color unchanged. `run_phase10_trial.sh` is the atomic experiment
entry: it creates a deterministic scenario, assigns an isolated ROS domain and
Fast DDS server, launches Isaac Sim and the complete Phase 10 ROS graph,
records compact MCAP/trajectory/commands/GPU/contact evidence, computes the
result, and cleans only its own process groups.

Before the recorder or goal runner starts, the script waits for
`nav2_lifecycle_guard` to report every managed Nav2 node active.  If the first
launch is only partially activated, the navigation wrapper destroys that
complete launch and creates a fresh one; no acceptance goal is allowed to
straddle the restart.  Matrix, trial and run-directory `flock` locks reject
duplicate invocations before they can share Isaac Sim or overwrite an MCAP.

Use `run_phase10_hardening.sh` for real relocalization/depth/map-slice fault
injection. Use `run_phase10_preacceptance.sh` for the 20 static plus 20 dynamic
matrix. The umbrella `run_phase10_tests.sh` runs the build, all automated tests,
hardening and the complete matrix. See
[`docs/phase10_validation.md`](phase10_validation.md) for thresholds, frozen
parameters and measured evidence, and [`docs/experiments.md`](experiments.md)
for artifact handling and retune workflows.

## Stage 11 formal acceptance

Generate the independent path reference from the actual official USD before a
new matrix:

```bash
./scripts/prepare_stage11_reference.sh --map warehouse_v2_front
```

The extractor reads authored `UsdPhysics.CollisionAPI` geometry with the Isaac
Sim 6.0.1 USD runtime. The planner then applies the padded asymmetric Nova
Carter footprint and runs an eight-heading SE(2) A* at 5 cm resolution. The
result is stored under `data/reference/warehouse_usd_005/`; no official USD is
saved or edited.

Run one completely automated trial with one of six fixed goals:

```bash
./scripts/run_stage11_trial.sh \
  --class static --seed 21000 --goal-index 3 --no-bag

./scripts/run_stage11_trial.sh \
  --class heterogeneous --seed 41000 --goal-index 5 --record-bag
```

`static`, `dynamic`, and `heterogeneous` respectively require zero, three, and
at least six moving actors. Goal indices 3–5 exercise long-distance warehouse
navigation. A trial is passed only by `tools/finalize_stage11_trial.py`, which
joins the simulator, navigation, command, data-age, GPU, contact, theoretical
path and optional MCAP evidence.

The formal command reads the default 10/10/10 counts from
`config/stage11.yaml`:

```bash
./scripts/run_acceptance.sh \
  --matrix-id phase11-formal-20260718 --record-bag
```

If the host or terminal is interrupted, do not delete completed runs. Resume
the exact identity set with:

```bash
./scripts/run_acceptance.sh \
  --matrix-id phase11-formal-20260718 \
  --resume --skip-build --record-bag
```

The resume path accepts a completed passed **or failed** report only when its
class, seed, goal index and Stage number match and `navigation.json/goals` proves
that a goal was actually attempted. This preserves real failures in the formal
denominator; only infrastructure-only empty runs are retried. The final machine report is
`data/reports/phase11/acceptance/<matrix-id>/summary.json`; see
[`docs/phase11_validation.md`](phase11_validation.md) for the complete metric
definitions and [`docs/user_manual.md`](user_manual.md) for day-to-day use.
