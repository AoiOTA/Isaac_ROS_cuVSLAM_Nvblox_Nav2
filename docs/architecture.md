# Architecture

## Runtime composition

The simulator will open the fixed warehouse USD once, reference the official Nova Carter main USD into the session layer, and create project-owned OmniGraphs at runtime. The official ROS sample USD is never loaded.

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
