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
