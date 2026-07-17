# Phase 3 Validation

Phase 3 was executed on 2026-07-17 against the actual Isaac Sim 6.0.1 warehouse and Nova Carter assets. The official `Nova_Carter_ROS.usd` graph was inspected only as an interface reference and was never loaded by the project simulator.

## Implemented control and state chain

- Standalone Python creates `/World/Graphs/Clock`, `DifferentialDrive`, `JointState`, and `GroundTruth` in the anonymous session layer.
- DifferentialDrive subscribes to `/cmd_vel_sim`, uses wheel radius 0.14 m and measured separation 0.4132 m, and commands exactly `joint_wheel_left` and `joint_wheel_right`.
- The five caster/swing joints are read through JointState but never included in the command array.
- JointState uses `isaacsim.sensors.physics.IsaacReadJointState`, avoiding the deprecated direct-target publisher path.
- Ground truth publishes `/ground_truth/odometry` with `sim_world/base_link` frame IDs and no TF.
- The ROS Command Guard converts `/cmd_vel_safe` to `/cmd_vel_sim`, rejects non-finite data, zeros non-planar components, clamps speed, smooths ordinary transitions, and applies a 0.25 s watchdog.
- Wheel odometry integrates the two actual wheel velocities into `/wheel/odometry` and does not publish TF.

## Reproduction command

From a clean terminal:

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/build.sh
./scripts/run_phase3_tests.sh
```

The final run passed all 16 checks. The persisted local evidence is `data/reports/phase3/latest.json` and `data/reports/phase3/simulator-latest.json`. A later short `run_sim.sh` smoke can safely replace `data/logs/stage3/latest.json` without losing the full-course evidence.

## Recorded motion results

| Test | Ground-truth result | Wheel-odometry error |
|---|---:|---:|
| Forward straight | 2.0151 m; lateral 0.49 mm; heading 0.00041 rad | 0.78 mm / 0.00062 rad |
| Reverse straight | 2.0165 m; lateral 1.43 mm | 2.23 mm / 0.00125 rad |
| In-place turn | 3.1800 rad; center drift 2.69 cm | 1.14 cm / 0.01043 rad |
| 0.75 m-radius arc | 1.5903 rad; chord 0.9201 m | 2.11 cm / 0.03294 rad |
| S-curve | 1.3418 m displacement | 3.74 cm / 0.03231 rad |
| Eight continuous sharp turns | 1.3810 m displacement | 1.13 cm / 0.00907 rad |

The ordinary-command acceleration P99 values were 1.60 m/s² linear and 3.00 rad/s² angular, matching the configured deceleration envelopes. Steady wheel-speed RMSE against the commanded differential-drive solution was 0.249 rad/s.

## Recorded safety and simulator results

| Check | Result |
|---|---:|
| Maximum guarded linear command | 1.0 m/s |
| Maximum guarded angular command | 1.2 rad/s |
| Watchdog zero latency | 0.25 s |
| NaN command zero latency | same observed simulation tick |
| Ground-truth samples | 2744 |
| JointState samples | 2917 |
| Wheel-odometry samples | 2915 |
| Simulation-time regressions | 0 |
| Initial/final unexpected PhysX overlaps | 0 / 0 |
| Official asset changes | none |

The final simulator report also confirms four runtime graphs, one warehouse stage open, only two commanded wheel joints, and `official_ros_sample_loaded=false`.
