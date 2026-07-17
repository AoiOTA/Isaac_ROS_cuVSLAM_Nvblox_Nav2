# Phase 4 Validation

Phase 4 was executed on 2026-07-17 with the actual Isaac Sim 6.0.1 Nova Carter and Warehouse assets. The simulator opens the Warehouse once, references the Nova Carter main USD in the anonymous session layer, and creates all sensor graphs at runtime. `Nova_Carter_ROS.usd` is not loaded.

## Implemented data path

- `/World/Graphs/FrontStereo` publishes synchronized 1280×800 RGB stereo images and stereo CameraInfo at 30 Hz.
- `/World/Graphs/FrontDepth` publishes 640×400 `32FC1` metric depth and matching CameraInfo at 30 Hz from the front-left Hawk camera.
- `/World/Graphs/FrontImu` publishes the front Hawk IMU on every 120 Hz physics step.
- `/World/Graphs/Clock` was moved to the same physics-step trigger, so `/clock` is 120 Hz instead of being limited by the render tick.
- Two Isaac ROS `ImageFormatConverterNode` components convert the left and right RGB streams to `mono8`. Both input and output use SensorData QoS.
- `robot_state_publisher` publishes only the tree below `base_link`. It uses USD-extracted front Hawk, wheel, and passive-caster transforms.
- Ground truth and wheel odometry remain outside the main TF tree; future cuVSLAM retains exclusive ownership of `map→odom→base_link`.

## Reproduction

From a clean terminal:

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/build.sh
./scripts/run_phase4_tests.sh
```

The test uses ROS domain 44, owns and cleans only its own process groups, and does not touch other Isaac Sim projects. It first captures a synchronized full stereo/depth payload, then audits the sustained streams while commanding stationary, left-arc, right-arc, spin, and settle phases.

## Final measured result

| Signal or check | Result |
|---|---:|
| `/clock` | 120.000 Hz |
| left/right stereo render cadence | 30.000 / 30.000 Hz |
| depth render cadence | 30.000 Hz |
| front Hawk IMU | 120.000 Hz |
| stereo RGB stamp equality | exact |
| mono image stamps present in matching CameraInfo sequence | 100% |
| depth valid-pixel fraction | 96.325% |
| sampled depth range | 0.218–12.333 m |
| timestamp regressions | 0 on every stream |
| static front sensor TF edges | complete |
| dynamic wheel/caster TF edges | complete |
| `/tf` and `/tf_static` publishers | exactly one each |
| motion phases | all five exercised |
| simulator graph count | 4 control + 3 sensor graphs |
| official USD changes | none |

The sustained audit records CameraInfo as the authoritative render cadence because images use Best-Effort SensorData QoS: a Python audit subscriber may sample large 1280×800 payloads without imposing reliable backpressure. Full RGB/depth content, encoding, dimensions, frame IDs, synchronized stereo stamp, non-constant image content, and finite metric depth are checked separately by the payload probe. Mono samples must retain stamps from the complete paired CameraInfo sequence.

Persisted local evidence is written to `data/reports/phase4/latest.json`, `data/reports/phase4/simulator-latest.json`, and `data/logs/stage4/latest.json`.
