# Architecture

## Runtime composition

The simulator opens the fixed warehouse USD once, references the official Nova Carter main USD into the session layer, and creates project-owned OmniGraphs at runtime. The official ROS sample USD is never loaded.

The intended pipeline is:

```text
Isaac Sim Standalone
  -> stereo / depth / IMU / clock
  -> cuVSLAM
  -> nvblox
  -> Visual Global Localization
  -> Nav2
  -> guarded cmd_vel
  -> two-wheel differential controller
```

## TF ownership

```text
map -> odom -> base_link -> sensors and wheel frames
```

- cuVSLAM owns `map -> odom` and `odom -> base_link`.
- robot_state_publisher owns the tree below `base_link`.
- VGL, wheel odometry, and ground truth do not publish competing TF edges.

## Initial integration order

The project first targets a front-stereo static-map MVP. Four-direction VGL, dynamic nvblox, lighting variation, and statistical acceptance are added only after the end-to-end mapping/localization/navigation loop works.

## Implemented Phase 3 control boundary

```text
/cmd_vel_safe
  -> Command Guard (finite check, planar projection, slew limits, watchdog)
  -> /cmd_vel_sim
  -> runtime DifferentialController
  -> runtime IsaacArticulationController
  -> [joint_wheel_left, joint_wheel_right]
```

The runtime graphs are `/World/Graphs/Clock`, `DifferentialDrive`, `JointState`, and `GroundTruth`. JointState uses the Isaac Sim 6.0 `IsaacReadJointState` sensor-output path. Ground truth publishes only an Odometry message in `sim_world`; it never enters the navigation TF tree. The ROS wheel odometry node integrates only the two driven wheels and also publishes no TF.

## Implemented Phase 4 sensor boundary

```text
front Hawk left/right cameras
  -> runtime FrontStereo (RGB + stereo CameraInfo, 1280x800 @ 30 Hz)
  -> Isaac ROS ImageFormatConverterNode x2
  -> mono8 left/right

front Hawk left camera -> runtime FrontDepth (32FC1 meters, 640x400 @ 30 Hz)
front Hawk IMU -> runtime FrontImu (physics step @ 120 Hz)
```

`Clock` and `FrontImu` use on-demand physics-step graphs; camera and depth graphs use render products at the camera's authored 30 Hz tick rate. All sensor publishers use Best-Effort SensorData QoS. The simplified Xacro contains the USD-extracted 0.15 m stereo baseline, optical-frame rotations, IMU mount, active wheels, and passive caster chain. Only robot_state_publisher owns TF below `base_link`.

## Implemented Phase 5 localization boundary

```text
front stereo RGB -> ImageFormatConverterNode x2 -> mono8
mono8 + stereo CameraInfo + front IMU -> cuVSLAM VIO
cuVSLAM -> map→odom→base_link + tracking odometry/status + map services
```

The official Hawk assets use the legacy `fisheyePolynomial` projection. Isaac Sim 6.0.1 renders that distortion but its stereo CameraInfo helper reports zero distortion, which is not a valid `rectified_images=true` input. The simulator therefore overrides only the two front navigation cameras to zero-distortion pinhole in the anonymous session layer before creating render products. The source USD remains unchanged. The 0.15 m stereo baseline remains encoded by the right CameraInfo projection matrix and the robot_state_publisher transform.

`visual_slam.launch.py` composes both GPU converters and cuVSLAM in a single multithreaded container with intra-process communication for normalized images. cuVSLAM uses the front stereo pair and IMU, publishes both main TF edges, enables mapping, and exposes save/load/get-all-poses services. Wheel odometry and simulator ground truth remain observation-only and never enter the main TF tree.

## Implemented Phase 6 reconstruction boundary

```text
native 32FC1 depth + depth CameraInfo ─┐
front-left RGB + color CameraInfo ─────┼→ nvblox static TSDF in odom
cuVSLAM/robot-state-publisher TF ──────┘     ├→ TSDF/color layers
                                              ├→ Mesh
                                              ├→ static 2D ESDF
                                              ├→ static_map_slice
                                              └→ .nvblx / .ply
```

`nvblox.launch.py` creates a project-owned multithreaded component container and loads `nvblox::NvbloxNode`. It uses one camera, 5 cm voxels, no lidar, native simulated depth, 30 Hz configured depth integration, 5 Hz color integration, 10 Hz ESDF update and 1 Hz mesh/layer output. `global_frame=odom` matches the future Nav2 local rolling costmap and avoids loop-closure discontinuities inside the reconstruction frame.

The 2D ESDF spans 0.09–0.65 m and is published as both a pointcloud and `DistanceMapSlice`. In nvblox 4.5 the dense `get_esdf_and_gradient` service is 3D-only, so it is deliberately not called in this configuration. Map, PLY, rates and timings are saved through actual nvblox services.
