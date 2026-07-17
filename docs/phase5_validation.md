# Phase 5 Validation

Phase 5 was executed on 2026-07-17 with Isaac Sim 6.0.1, Isaac ROS Visual SLAM 4.5.0, cuVSLAM 15.0.0, ROS 2 Jazzy, and the actual official Nova Carter and Warehouse assets. The official ROS sample USD was not loaded; all simulator graphs were created at runtime by Standalone Python.

## Implemented localization path

- The front Hawk stereo pair is rendered at 1280×800 and published with synchronized stereo CameraInfo.
- Two Isaac ROS GPU `ImageFormatConverterNode` components produce `mono8` in the same component container as cuVSLAM.
- cuVSLAM runs in VIO mode with the front stereo IMU, mapping enabled, SensorData QoS, and simulation time.
- cuVSLAM exclusively publishes `map→odom` and `odom→base_link`; robot_state_publisher owns only descendants of `base_link`.
- Tracking odometry and status are recorded continuously.
- `/visual_slam/save_map`, `/visual_slam/get_all_poses`, and `/visual_slam/load_map` are all exercised by automation.

## Isaac Sim 6.0 Hawk projection correction

The front Hawk camera prims in the official asset author `cameraProjectionType=fisheyePolynomial` and rational-polynomial distortion. In Isaac Sim 6.0.1 this legacy projection generates deprecation warnings, while the ROS stereo CameraInfo helper publishes `plumb_bob` with all-zero distortion. Feeding those rendered images to cuVSLAM as rectified images produced a measured 2.41–3.30× one-second translation scale and poor translation direction.

The project now sets only the two front navigation camera prims to zero-distortion `pinhole` in the anonymous session layer before render products are created. It does not edit or save the official USD. A 35-second A/B verification then measured a median scale ratio of 0.994, a windowed path ratio of 0.984, a translation direction cosine of 0.9998, and 100% rotation-sign agreement.

## Reproduction

From a clean terminal:

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/build.sh
./scripts/run_phase5_tests.sh
```

The test uses ROS domain 45 by default, owns only its simulator and bringup process groups, and stops through the repository's simulator sentinel. It repeatedly commands forward-left, forward-right, reverse-right, reverse-left, and settle segments for at least 122 simulation seconds. Ground truth is subscribed only by the experiment runner and never enters localization or control.

## Final measured result

| Check | Result |
|---|---:|
| continuous successful tracking | 123.10 s |
| failures after first lock | 0 |
| tracking status samples | 1191 |
| maximum successful-status gap | 0.800 s |
| mean / P99 node callback time | 1.49 / 10.69 ms |
| `map→odom` samples | 1191 |
| `odom→base_link` samples | 1190 |
| TF timestamp regressions | 0 |
| competing parent for `odom` or `base_link` | none |
| ground-truth path length | 31.499 m |
| median one-second visual/ground-truth scale | 0.9980 |
| aligned translation direction cosine | 0.9966 |
| rotation direction agreement | 99.91% |
| one-second windowed path ratio | 1.2239 |
| map save / load | success / success |
| optimized poses returned | 1216 |
| saved map artifact | `data.mdb`, non-empty |
| official USD changes | none |

The raw short-interval visual path ratio is also reported for transparency, but acceptance uses one-second displacement windows and median scale so frame-to-frame estimator noise is not counted as physical travel. The persisted local result is `data/reports/phase5/latest.json`; generated reports, logs, and maps are intentionally ignored by Git.
