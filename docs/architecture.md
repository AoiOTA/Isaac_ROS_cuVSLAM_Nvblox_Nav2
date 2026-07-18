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

- Phases 5–8 use cuVSLAM-derived main TF edges as described below.
- Phase 9 navigation makes `navigation_tf_bridge` the sole `map -> odom`
  publisher and `visual_wheel_ekf` the sole `odom -> base_link` publisher.
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

## Implemented Phase 8 navigation boundary

```text
cuVGL pose -> /visual_slam/initial_pose -> cuVSLAM map localization
cuVSLAM slam_path + tracking odometry -> current-time map->odom adapter
native depth -> raw LaserScan -> Collision Monitor
native depth -> TF-synchronized odom PointCloud2 -> local ObstacleLayer
nvblox static_map_slice -> local NvbloxCostmapLayer
occupancy map -> global StaticLayer

SmacPlanner2D -> MPPI(DiffDrive) -> /cmd_vel_nav_raw
  -> Velocity Smoother -> /cmd_vel_smoothed
  -> Collision Monitor -> /cmd_vel_safe
  -> navigation-health Command Guard -> /cmd_vel_sim
  -> runtime two-wheel differential OmniGraph
```

Phase 8 disables cuVSLAM's direct `map→odom` TF output and makes `navigation_tf_bridge` the sole publisher of that edge. The adapter computes it only from cuVSLAM's map-frame SLAM path and odom-frame tracking odometry, then republishes at current simulation time. Ground truth and wheel odometry are never inputs. cuVSLAM remains the sole `odom→base_link` source, while robot_state_publisher owns all robot-fixed edges.

The raw scan stays in `base_link` and feeds Collision Monitor without waiting for localization. `scan_timestamp_relay` separately releases a scan only when a matching cuVSLAM transform exists and publishes `/front_depth/points_odom`; this prevents future-dated depth from poisoning the rolling costmap while preserving a low-latency emergency stop path.

Nav2 uses a map-frame global costmap with Static and Inflation layers, and an odom-frame rolling local costmap with Nvblox, visual PointCloud2 Obstacle, and Inflation layers. SmacPlanner2D supplies the global path. MPPI runs a DiffDrive motion model at 20 Hz and publishes the controller-local transformed path. The final guard checks localization readiness, cuVSLAM tracking freshness, depth freshness, nvblox slice freshness, finite commands, planar motion, bounds, acceleration/jerk, and a 250 ms command watchdog.

## Implemented Phase 9 front-stereo dynamic-navigation boundary

Phase 9 deliberately keeps the validated front Hawk stereo pair as the only
runtime visual-localization rig. Side and rear cameras are not created by the
default simulator entry and are not required by mapping, recovery, RViz, or
acceptance.

```text
front stereo + IMU -> cuVSLAM status/tracking health ───────────────┐
front stereo -> triggered cuVGL -> innovation-gated map pose ──────┤
wheel odometry twist -> robot_localization EKF -> odom->base_link ─┤
VGL anchor + filtered odom -> navigation_tf_bridge -> map->odom ───┘

native front depth -> nvblox dynamic mapper
  -> dynamic ESDF/map slice + combined ESDF/map slice
  -> local NvbloxCostmapLayer

RViz 2D Goal Pose -> /goal_pose -> manual_goal_bridge
  -> /navigate_to_pose_resilient
  -> stock /navigate_to_pose
  -> SmacPlanner2D + MPPI(DiffDrive)
  -> Velocity Smoother -> Collision Monitor -> Command Guard
```

The daily Phase 9 runner uses this manual path and does not embed a goal list.
The bounded acceptance runner calls the resilient action directly only when
`run_phase9.sh --auto` is selected.

Moving foreground features in a front-only warehouse view can momentarily
perturb raw visual odometry. Phase 9 therefore separates global visual
observability from smooth local prediction: VGL anchors `map`; cuVSLAM health
remains a hard permission for motion; the EKF integrates only the two-wheel
odometry twist between visual global corrections. Ground truth is never an
input. A lost or stale cuVSLAM status immediately withdraws localization
readiness, so wheel prediction can never authorize blind navigation.

The recovery manager waits for a potentially occluding foreground object to
clear, triggers front-stereo VGL, rejects excessive translation/yaw innovation,
injects an accepted pose into cuVSLAM, and requires 20 consecutive healthy
tracking samples. It retries at most three times and otherwise remains in
`failed_safe`. The resilient action proxy cancels the active Nav2 goal while
unready and submits the same goal after recovery.

The simulator moves an official scene forklift plus project-owned box and
capsule shapes using session-layer kinematic transforms. PhysX contact reports
are part of acceptance: moving actors must enter the local cost envelope but
their physical swept volumes must not ram a correctly stopped robot.

## Implemented Phase 10 experiment and safety boundary

Phase 10 keeps the same front-stereo perception graph and adds an automation
boundary around it. Every trial owns an isolated DDS discovery server,
simulator, ROS launch, test runner, optional MCAP recorder and GPU sampler. A
single finalizer joins navigation, simulation, contact and resource evidence;
ground truth remains metrics-only.

Kinematic obstacle actors retain collision and perception geometry while
yielding. Ordinary actors retreat along their configured trajectory in the
direction that increases robot clearance, then resume after release hysteresis.
For a latched refuge, every bounded motion step chooses among both route
endpoints and the refuge while retaining the current position as a candidate;
the actor is therefore never commanded to reduce its current center clearance
from the robot. This is necessary because the visual map can be rotated from
the USD world and a fixed world-frame refuge may temporarily lie beyond the
robot. It avoids both an actor ramming a correctly stopped robot and the old
stationary-obstacle deadlock. All motion, yield events, final positions and
PhysX contacts remain acceptance evidence.

Test-only SetBool services can suppress depth or combined-map health refreshes
when explicitly enabled by `phase10.launch.py`. Normal Phase 9 launch behavior
is unchanged. The final Guard must enter the corresponding blocked state and
publish zero motion before the injected fault can count as recovered.

Nav2 lifecycle activation is checked once after staged startup. A partial
activation shuts down the complete ROS launch and the wrapper starts a fresh
stack, up to three attempts; in-process lifecycle reset is deliberately avoided
because the nvblox costmap plugin cannot be safely reconfigured in place. Run,
matrix and physical-GPU locks make report ownership unambiguous.

## Implemented Phase 11 long-distance and formal-acceptance boundary

Stage 11 retains the front-stereo-only runtime and makes the global-path
reference independent of Nav2. The reference extractor opens the fixed
official Warehouse USD with Isaac Sim's USD runtime and selects actual
`UsdPhysics.CollisionAPI` geometry in the robot vertical band. An eight-heading
SE(2) A* evaluates the padded asymmetric Nova Carter footprint at 5 cm
resolution. Ground-truth poses are used only after a run to measure the actual
path; neither the USD reference nor ground truth is sent to Nav2.

The local ObstacleLayer consumes `/front_depth/scan` in the true optical sensor
frame. This is essential beyond the startup neighborhood: a PointCloud2 that
has already been transformed into `odom` contains no separate raytrace origin,
so Nav2 otherwise treats `odom (0,0)` as the sensor and eventually places the
origin outside the rolling costmap. `/front_depth/points_odom` remains a
visualization and metrics product, not an ObstacleLayer input.

Dynamic nvblox uses a rolling 8 m map-clearing radius at 1 Hz. MapServer owns
the fixed global occupancy map, while nvblox owns only the bounded local 3D
TSDF/ESDF and combined slice. This keeps long-distance slice publication above
the safety contract without discarding global static structure.

```text
actual USD CollisionAPI -> padded-footprint SE(2) reference ─┐
ground-truth metrics-only trajectory -------------------------┤
ROS navigation/data age/latency/smoothness -------------------┼-> trial finalizer
PhysX contacts + actor motion + simulator graph identity ------┤
GPU/RTF + optional compact MCAP -------------------------------┘
                                                               -> 10/10/10 summary
```

Each trial owns its DDS server, simulator, ROS stack, recorder, sampler and run
directory. The final matrix validates the complete class/seed/goal identity set
before computing static, dynamic, heterogeneous and long-distance success
rates. Lighting and color remain unchanged by explicit Stage 11 scope.
