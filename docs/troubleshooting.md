# Troubleshooting

## Phase-0 build failure

- Confirm `/opt/ros/jazzy/setup.bash` exists.
- Confirm `/home/lyb/miniconda3/envs/isaacsim/bin/python` exists.
- Confirm `nvidia-smi` can access the RTX 4090.
- Confirm the two paths in `config/assets.yaml` exist.

`scripts/build.sh` is read-only with respect to system packages and official USD assets. It does not use sudo and does not terminate any running process.

## Bare-metal installation interruption

Rerun the same command:

```bash
./scripts/bootstrap_baremetal.sh --install
```

Repository and rosdep files are installed idempotently. Each run writes its APT simulation and installed-package manifest under `data/logs/stage1/`.

## APT proposes removals

The installer stops before the real target install. Inspect the newest `data/logs/stage1/apt-simulate-*.log`; do not bypass this check. Existing ROS 2 Jazzy, Nav2, OpenCV, and NVIDIA driver packages must remain installed.

## ROS Bridge smoke failure

Inspect the newest `data/logs/stage1/ros-bridge-*.log`. The smoke script sources the system Jazzy environment and uses the project DDS settings before launching Isaac Sim. It controls only the PID/PGID it created.

## `pip check` reports PyNaCl/cffi

Ubuntu's `python3-nacl` package declares the compiled `python3-cffi-backend` Debian dependency, while its Python package metadata still names `cffi`. This can make `pip check` report `pynacl 1.5.0 requires cffi`. On this host, both `nacl` and `nacl.bindings` import successfully; this warning is unrelated to the Isaac ROS runtime.

## Simulator says this project already has a running instance

`navigation_sim.py` uses an advisory lock at `data/locks/navigation_sim.lock`. If another instance from this repository is active, stop that instance normally with Ctrl-C. The lock does not inspect, signal, or stop Isaac Sim processes from other projects. A stale text file after a crash is harmless because ownership is maintained by the kernel lock, not by file existence.

## GUI mode does not create a window

Check the graphical session before starting:

```bash
echo "${DISPLAY}"
test -S /tmp/.X11-unix/X0
```

The verified workstation uses X11 with `DISPLAY=:0`. Headless mode does not require a visible viewport.

## Stage-2 report says an overlap was found

Inspect `composition.spawn` and `runtime.*_unexpected_overlaps` in `data/logs/stage2/latest.json`. The first is the conservative USD collision-bound search; the second is the PhysX runtime query. Do not bypass either check by hard-coding a pose. Fix the footprint, clearance, or collision classification and rerun.

## Phase-3 topics collide with another simulator

The public environment deliberately uses ROS domain 42. If another project on the workstation uses the same domain, do not terminate it. `run_phase3_tests.sh` uses isolated domain 43 by default and starts every tested process with that value. For a one-off alternate domain:

```bash
PHASE3_TEST_ROS_DOMAIN_ID=44 ./scripts/run_phase3_tests.sh
```

## Command Guard reports timeout

The final command must be refreshed faster than 0.25 seconds in simulation time. A timeout is a safety stop, not a simulator failure. Nav2's later velocity smoother/collision monitor chain must continuously publish while motion is authorized.

## Wheel odometry diverges during aggressive tests

Inspect the segment-level errors in `data/reports/phase3/latest.json`. Large error during only one maneuver can indicate wheel slip or a collision rather than a wrong differential formula. The accepted suite automatically returns arc and S-curve tests toward the collision-free test center before continuous sharp turns; do not concatenate arbitrary open-loop paths through warehouse geometry when evaluating pure kinematics.

## Stage 8 waits for Nav2 action or lifecycle activation

Inspect the newest `data/logs/stage8/*/navigation.log`. Phase 8 deliberately starts nvblox three seconds after localization and Nav2 twelve seconds after localization, then waits another four seconds before activating the navigation lifecycle manager. This avoids concurrent TensorRT, cuVSLAM, nvblox, and Nav2 allocation spikes. Do not remove the delays merely because all processes have appeared in `ps`; an action server can exist before every managed node is active.

If another project is automatically relaunching Isaac Sim, stop it through that project's own supervisor. This repository never uses `killall` or cross-project `pkill` and only cleans process groups that it created.

## Robot creeps or does not move during Stage 8

Read the five-second `Goal progress` lines in `test-runner.log`. They show map pose, ground truth, Guard state, Collision Monitor action, and all four command stages. Interpret them in order:

- A nonzero `/cmd_vel_nav_raw` that becomes zero at `/cmd_vel_safe` indicates Collision Monitor intervention or stale raw depth.
- A nonzero `/cmd_vel_safe` that becomes zero at `/cmd_vel_sim` identifies the exact blocked health state in `guard=`.
- A low `/cmd_vel_nav_raw` with all later stages matching is an MPPI critic/trajectory issue, not a differential-controller issue.
- A moving ground truth pose with a frozen map pose is a localization/TF problem and must stop navigation.

The accepted tuning uses a 0.15 temperature, 0.30 linear sampling deviation, stronger Goal/PathFollow critics, an 0.08 m/s/rad/s deadband critic, and bounded acceleration/jerk downstream. Keep footprint collision checking and Collision Monitor enabled when changing these weights.

## SmacPlanner2D prints an inflation error while planning succeeds

ROS 2 Jazzy Nav2 1.3.12 `SmacPlanner2D` constructs its 2D collision checker with `radius=true` and `possible_collision_cost=0`. In this release, `GridCollisionChecker::setFootprint()` logs the generic non-circular inflation error before it evaluates the radius fast path. The project global costmap does contain a 1.0 m InflationLayer, and both real three-goal runs completed. Treat the single configure-time message as an upstream 2D false positive; do not hide real repeated planning, collision, or costmap errors.

## RViz reports occasional base_link message-filter queue drops

The tested RViz configuration loads successfully and renders the occupancy map, both costmaps, paths, robot, TF, images, depth, nvblox outputs, scan, and collision polygons. Under simultaneous RTX rendering, cuVSLAM, nvblox, VGL, and RViz, an occasional old `base_link` visualization message can be dropped because the display queue is full. Navigation uses the synchronized odom pointcloud and is unaffected. Persistent drops together with missing current displays indicate TF or GPU starvation and should fail a fresh `run_phase8_tests.sh` run.

## Stage 9 front-stereo VGL repeatedly rejects poses

Inspect `/localization/recovery_state`, `/vgl_pose_relay/accepted`, and the
`vgl_pose_relay` log together. Confirm that `warehouse_v2_front/cuvslam` and
`warehouse_v2_front/cuvgl` came from the same offline bag and pose solution.
Do not loosen the innovation gate first: wait for a moving foreground object to
clear, verify the 3 ms stereo synchronization window, then improve map coverage.
The recovery manager retries at most three times and intentionally leaves the
robot stopped in `failed_safe` after exhaustion.

## Stage 9 map pose is stable but local odometry jumps near movers

Large moving foregrounds can contaminate front-only visual odometry even while
cuVSLAM reports tracking. The Stage 9 launch deliberately disables cuVSLAM's
direct TF output, publishes `odom→base_link` from `visual_wheel_ekf`, and uses
cuVGL to anchor `map→odom`. Confirm exactly one publisher for each edge and
that the EKF input is `/wheel/odometry`; ground truth must never appear as an
input. cuVSLAM status remains a hard Command Guard prerequisite, so do not
remove the visual-health gate merely because wheel-local prediction is smooth.

## Stage 9 dynamic obstacle collides with a stopped robot

Read `dynamic_obstacles.robot_contact_pairs` in the simulator JSON. Kinematic
actors do not yield when Collision Monitor stops the robot, so their physical
swept volume must stay outside the robot footprint while their nvblox/inflation
envelope can still intersect the route. For generated USD shapes, author the
translate op before scale; reversing them scales the waypoint and silently
moves the obstacle into the robot. Never disable the PhysX contact assertion to
make a run pass.

## Stage 9 guard alternates active and timeout under full RViz load

First distinguish the final 250 ms command watchdog from navigation-health
freshness. Phase 9 keeps the hard command watchdog unchanged, but allows 2 s
for front depth, cuVSLAM status and combined map slice because GPU rendering,
dynamic nvblox and RViz share one RTX 4090. Persistent gaps longer than this
mean the 10 Hz front stream or DDS discovery is unhealthy; verify the local
Fast DDS server, topic rates and GPU load instead of increasing timeouts again.
