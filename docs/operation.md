# Operation

Available through Phase 3:

```bash
./scripts/build.sh
./scripts/bootstrap_baremetal.sh --preflight
./scripts/smoke_ros_bridge.sh
python3 tools/check_stage1.py
./scripts/smoke_isaac_ros_nodes.sh
./scripts/run_sim.sh --headless
./scripts/run_control.sh
./scripts/run_phase3_tests.sh
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

Mapping, navigation, and final acceptance entry points will be added only when their corresponding phases are implemented and verified.
