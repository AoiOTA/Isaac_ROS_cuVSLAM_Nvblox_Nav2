# 架构与所有权

## 仿真组合

`isaac_sim/navigation_sim.py` 是正式 Standalone 入口。它先创建 `SimulationApp`，再执行：

1. 只打开一次酷家乐 `/Root` stage；
2. 在匿名 session layer 引用 `/World/Jackal`；
3. 在 Jackal `base_link` 下引用 front、left、right、back 四组 Hawk；
4. 写入轮地物理、碰撞与求解迭代 overlay；
5. 创建项目拥有的控制、Clock、JointState、GroundTruth 和传感器图；
6. 启动 contact monitor、idle brake 与有界 skid-steer motion assist；
7. 检查 stage identity、仿真时间单调性、重叠、碰撞和三个官方资产指纹；
8. 写 JSON 报告后关闭。

组合 stage 永不保存，源 USD 永不修改。固定 USD 出生点为 `[2.9, -0.2, 0.0635]`、yaw `180°`；视觉地图启动坐标为 map `[0, 0, 0]`。

## 相机 profile

| Profile | front | left | right | back | RGB 总数 | 用途 |
|---|---:|---:|---:|---:|---:|---|
| `mapping_8cam` | 2 | 2 | 2 | 2 | 8 | 手动建图、离线 cuVSLAM/cuVGL |
| `navigation_6cam` | 2 | 2 | 2 | 0 | 6 | 定位与导航 |

导航 profile 不为 back Hawk 创建 render product，因而不是“发布后丢弃”或“ROS 侧不订阅”。front 原生 depth 与 IMU 在两个 profile 中均启用。

## TF 所有权

```text
map -> odom -> base_link -> four Hawk optical frames + front IMU + wheels
```

- `robot_state_publisher` 独占 `base_link` 以下固定 TF。
- cuVSLAM 发布跟踪里程计；导航时关闭其直接 `map -> odom`，由 `navigation_tf_bridge` 组合 VGL map anchor 与 cuVSLAM odom 后独占发布。
- wheel odometry 与 `/ground_truth/odometry` 不发布主 TF，也不作为正式定位输入。
- nvblox 在 `odom` 中重建，occupancy map 在 `map` 中供全局规划。

## 感知与定位

```text
8 RGB mapping ──> ImageFormatConverter x8 ──> cuVSLAM map
                                         └─> offline cuVGL map

6 RGB navigation ──> cuVSLAM tracking
                  └─> cuVGL relocalization ──> map anchor

front 32FC1 depth ──> nvblox static TSDF/ESDF/static_map_slice
                   └─> raw LaserScan ──> Collision Monitor
                   └─> TF-aligned scan/points ──> Nav2 obstacle layers
```

所有 Hawk 相机在 session layer 中使用与 `rectified_images=true` 一致的 pinhole 投影。源 Hawk USD 保持不变。图像为 `1280×800 @ 10 Hz`，front depth 为 `640×400 @ 10 Hz`，front IMU 为 `120 Hz`。

nvblox 配置为 `static_tsdf`、5 cm voxel、2D ESDF，深度集成 10 Hz、颜色 3 Hz、ESDF 10 Hz；只接 front native depth，不使用 LiDAR 或双目深度网络。`/front_depth/scan[_raw]` 虽采用 ROS `LaserScan` 消息类型，但数据由 Hawk 深度图投影产生；Jackal LiDAR prim、render product 和 publisher 均不创建。

## 导航与控制

```text
occupancy map ──> global Static/Obstacle/Inflation costmap
nvblox slice + front scan ──> local Nvblox/Obstacle/Inflation costmap

SmacPlanner2D
  -> MPPI DiffDrive (10 Hz, 20 x 0.1 s, batch 500, vx 0..0.75)
  -> /cmd_vel_nav_raw
  -> Velocity Smoother
  -> /cmd_vel_smoothed
  -> Collision Monitor
  -> /cmd_vel_safe
  -> Command Guard
  -> /cmd_vel_sim
  -> four-wheel DifferentialController on every physics step
```

MPPI 的 `vx_min=0.0`，Velocity Smoother 的最小线速度也是 0。Behavior Server 只注册 Spin 和 Wait，`navigate_forward_only.xml` 没有 BackUp 或 DriveOnHeading。

Command Guard 做 finite/平面化、速度/加速度/jerk、定位与感知新鲜度以及 0.25 秒 watchdog。Collision Monitor 保留 Stop、Slowdown 和 footprint approach 区域。

## Jackal 滑移转向

四个轮子仍由差速目标和 PhysX 接触驱动。由于各向同性接触会让四轮 skid-steer 弧线严重欠转，运行时增加参考分支标定的受限平面速度修正：

- 几何轮距：`0.37559 m`，只描述 USD/URDF 轮心；
- 控制与 wheel odometry 有效轮距：`0.800 m`；
- motion assist 命令超时：`0.25 s`；
- 最大修正带宽：线 `6.0 m/s²`、角 `30.0 rad/s²`；
- Isaac 内层硬保护：线/角速度 `1.0 m/s` / `1.5 rad/s`，轮速 `15 rad/s`，线/角加速度 `2.0 m/s²` / `6.0 rad/s²`；
- 正常导航命令仍由较低的 `1.1 m/s²` / `3.0 rad/s²` 约束；
- 无命令、零命令或超时后 idle brake 清零 base/轮速并让 base 进入 sleep；
- motion assist 不绕过 Command Guard、Collision Monitor、超时或正式实验倒车判定。

这些参数固定对照参考分支 `caae0c08a451544ec0362df12da165fdc8d75676` 的 skid-steer 复盘。酷家乐复盘中的只读 overlay、米制/Z-up/唯一 PhysicsScene、double-sided 墙面和地图/出生点配对原则同样保留；RTX LiDAR 专项处理不适用于当前 Hawk 原生深度链，死胡同 BackUp 例外也因本项目要求导航完全不倒车而没有引入。

## 验收数据流

```text
navigation_test_runner result + command trace
simulator time/profile/contact report
immutable trial metadata
  -> finalize_static_trial.py
  -> passed / valid failed / infrastructure invalid
  -> summarize_static_avoidance.py
  -> collision-free passages / valid trials
```

runner 一旦被调用，本轮就是有效实验；之后所有失败留在分母。runner 调用前的启动失败单独报告和重跑。20 次有效实验至少需要 19 次完整成功。

Ground truth 仅用于验证最小实际运动和轨迹证据，不反馈给定位、规划或控制。

## 性能观测

`AdaptiveOfficialBenchmark` 使用 Isaac Sim 6.0.1 的 App、Physics、CPU、Memory 和 Hardware 官方 recorders。外部采样器同时统计 simulator 与 ROS 进程树、系统内存和 `nvidia-smi` 指标。

工作负载 ready file 到达后才开始预热。预热和正式采样均按墙钟窗口稳定性结束，或达到最大墙钟时长；报告显式写入 `fixed_frame_count: null` 和 `fixed_kpi_thresholds: null`。

ready file 不是在 ROS 栈启动后直接创建：建图 profile 同时运行临时 8 路 MCAP
录制和安全原地交替旋转，导航 profile 循环发送真实 Nav2 目标；只有观察到
`/cmd_vel_sim` 的非零命令且 ground truth 位姿确实变化后，活动负载驱动器才允许
自适应预热开始。原始性能 MCAP 在验证 8 路图像均有消息后立即删除，只保留计数
证据。
