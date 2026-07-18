# 阶段9：前向双目动态导航与定位恢复验证

## 1. 本阶段范围与结论

阶段9按当前项目决策固定使用**前向Hawk双目、前向Hawk IMU和前左目原生深度**。侧向与后向Hawk不会在默认启动、运行时数据流或验收中启用。这样可以先把动态重建、动态避障、失锁安全停车、cuVGL重定位和Nav2任务恢复做成稳定闭环，同时控制RTX渲染、GPU推理和DDS带宽。

本阶段完成的功能为：

- Standalone Python在匿名session layer中创建并驱动异构动态障碍。
- nvblox从`static_tsdf`切换到`dynamic`，同时输出静态、动态和合并ESDF/map slice。
- Nav2局部代价地图消费`/nvblox_node/combined_map_slice`，并保留前向深度ObstacleLayer和Collision Monitor。
- cuVGL只使用前向左右图像执行按需全局重定位，不连续占用推理资源，也不发布TF。
- cuVSLAM tracking失效或强制重定位时立即撤销导航ready，Command Guard归零。
- 恢复管理器最多触发cuVGL三次；位姿通过创新门限、cuVSLAM连续健康后才恢复导航。
- `resilient_navigation`在定位恢复期间取消下游Nav2 goal，恢复后自动提交同一个goal。
- RViz增加dynamic ESDF、combined ESDF和动态点云显示，保留完整导航、代价地图、路径、TF、RobotModel、前向图像和深度显示。

## 2. 固定运行架构

```text
Front Hawk stereo + IMU ──> cuVSLAM tracking/status ──> localization health
Front Hawk stereo ────────> cuVGL on demand ──────────> innovation-gated map pose
wheel joint states ───────> wheel odometry twist ─────> robot_localization EKF

accepted VGL map pose + filtered odom ──> map -> odom
filtered wheel-local prediction ─────────> odom -> base_link
robot_state_publisher ───────────────────> base_link -> sensors/wheels/casters

front native depth + filtered TF ────────> dynamic nvblox
                                      ├── dynamic_map_slice
                                      ├── combined_map_slice
                                      ├── dynamic_esdf_pointcloud
                                      └── combined_esdf_pointcloud

SmacPlanner2D -> MPPI(DiffDrive) -> Velocity Smoother
  -> Collision Monitor -> Command Guard -> /cmd_vel_sim
  -> DifferentialController -> left/right drive joints
```

阶段9不把ground truth输入定位、TF、代价地图或控制。`/ground_truth/odometry`只供测试计算误差、轨迹长度和碰撞关联。

### 2.1 为什么阶段9局部TF使用轮速EKF

仓库中移动前景会显著改变前向双目特征分布。实测中，直接把含大面积移动物体的cuVSLAM tracking odometry作为局部控制TF，会出现短时位置跳变。阶段9因此采用以下职责分离：

- cuVSLAM仍持续处理前向双目+IMU，其status是导航健康硬门控。
- cuVGL提供全局`map`锚点。
- `robot_localization`只融合`/wheel/odometry`的平面速度，产生连续的局部`odom→base_link`。
- `navigation_tf_bridge`在重定位ready边沿锁定VGL锚点，并发布唯一`map→odom`。

这不是使用ground truth替代视觉定位：视觉失锁会立即停止机器人，视觉全局位姿仍决定地图对齐；轮速只承担两个视觉全局修正之间的平滑局部预测。

## 3. 前向双目数据契约

| 数据 | 阶段9设置 |
|---|---|
| 左右图像 | 1280×800、mono8、10 Hz目标发布频率 |
| CameraInfo | 与左右图像同仿真时间戳 |
| IMU | 120 Hz |
| 深度 | 640×400、32FC1米制 |
| cuVSLAM相机数 | 2 |
| cuVGL相机数 | 2，ID顺序为前左、前右 |
| cuVGL同步窗 | 3 ms运行时配置 |
| 周边相机 | 默认不创建对应OmniGraph，不参与验收 |

完整栈同时运行时，10 Hz双目比30 Hz更稳：它与`warehouse_v2_front`离线地图采样频率一致，并避免cuVSLAM、cuVGL、dynamic nvblox、MPPI和RViz同时运行时出现DDS历史队列积压。IMU仍保持120 Hz。

## 4. 动态障碍实现

动态配置位于`config/scenarios.yaml`的`warehouse_crossing`：

- 官方场景已有`/World/Forklift`，只在session layer中增加kinematic和轨迹覆盖。
- `/World/Stage9DynamicObstacles/crossing_box`为红色箱体。
- `/World/Stage9DynamicObstacles/crossing_capsule`为蓝色胶囊体。

所有障碍采用周期三角波往返轨迹。生成几何的USD变换必须保持translate op在scale op之前，否则scale会作用于waypoint，使实际位置偏离配置。仿真注册PhysX contact report；任何Nova Carter与阶段9动态障碍的接触都会使本轮验收失败。轨迹同时保证障碍进入局部膨胀区，但kinematic物体的物理扫掠不直接穿过静止机器人的footprint。

## 5. dynamic nvblox与Nav2接入

核心参数文件是`nvblox_dynamic.yaml`：

- `mapping_type: dynamic`
- 5 cm voxel，单前向深度相机，不使用lidar。
- 动态occupancy以10 Hz目标频率衰减。
- dynamic mapper输出2D ESDF高度带0.09–0.65 m。
- 静态和动态层共同形成`combined_map_slice`。

Nav2局部代价地图保持`global_frame=odom`，NvbloxCostmapLayer在阶段9启动时被改写为：

```text
/nvblox_node/combined_map_slice
```

视觉深度还并行生成：

- 原始`base_link` LaserScan供Collision Monitor低延迟停车。
- TF同步后的`/front_depth/points_odom`供ObstacleLayer清除和标记。

因此即使dynamic ESDF存在更新延迟，原始深度安全链也不会等待全局定位或地图融合。

## 6. 定位恢复状态机

```text
camera_warmup / obstruction_clearance
  -> triggering_vgl
  -> waiting_for_vgl_pose
  -> waiting_for_tracking
  -> navigation_ready
```

任一步超时或位姿被创新门限拒绝时重新尝试，最多三次；耗尽后进入`failed_safe`。恢复期间：

1. `/localization/ready=false`以transient-local QoS发布。
2. Command Guard拒绝所有运动命令。
3. 活跃的`/navigate_to_pose`由代理取消。
4. 前向障碍遮挡清除等待结束后触发`/visual_localization/trigger_localization`。
5. `vgl_pose_relay`检查相对当前锚点的平移和yaw创新。
6. 接受的PoseWithCovariance发送到`/visual_slam/initial_pose`。
7. 连续20个健康cuVSLAM状态后重新ready。
8. `/navigate_to_pose_resilient`自动重发原目标。

阶段9验收会在第一个活动目标行驶0.15 m后主动注入一次安全重定位，证明停车、重定位和目标恢复链确实执行，而不是只检查节点存在。

## 7. 干净终端复现

构建和静态契约：

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/build.sh
./scripts/run_phase9_tests.sh warehouse_v2_front
```

只执行一次完整动态导航：

```bash
./scripts/run_phase9.sh --map warehouse_v2_front --headless --rviz
```

不打开RViz的CI/诊断运行：

```bash
./scripts/run_phase9.sh --map warehouse_v2_front --headless --no-rviz
```

分进程调试时，先启动本项目仿真，再启动ROS侧：

```bash
./scripts/run_sim.sh --headless
./scripts/run_phase9_navigation.sh --map warehouse_v2_front --rviz
```

正式阶段9入口会创建并只清理自己拥有的进程组，不调用`killall`或跨项目`pkill`。默认使用ROS domain 59和绑定到127.0.0.1:11859的项目本地Fast DDS发现服务器。

## 8. 自动测试与通过条件

静态测试覆盖：

- dynamic nvblox参数、combined slice remap和Nav2插件契约。
- 前向双目cuVSLAM/cuVGL参数和严格同步配置。
- TF唯一所有者、轮速EKF和VGL锚点接口。
- 重定位状态机三次上限、创新门限和resilient action。
- 动态障碍类型、轨迹、接触报告和官方资产不修改约束。
- RViz动态ESDF/combined ESDF/动态点云配置。

实际运行必须同时满足：

- 三个NavigateToPose全部成功，终点位置≤0.25 m、航向≤10°。
- cuVSLAM持续提供健康状态，主TF链完整。
- 强制重定位被接受且至少完成一次目标暂停/恢复。
- localization unready宽限期后最大运动命令≤0.02。
- dynamic/combined ESDF和map slice全部非空。
- 原始深度安全scan和odom点云均非空、时间不回退。
- 仿真时间单调、三个动态障碍确实移动。
- Nova Carter与动态障碍PhysX接触数为0，最终无USD重叠。
- 官方Warehouse/Nova Carter USD的大小和mtime不变，stage只打开一次。

## 9. 本机实测记录

最终提交前的完整带RViz回归结果记录在本节；运行生成的JSON、bag、地图和日志属于`data/`运行产物，不提交Git。自动检查器为：

```bash
python3 tools/check_phase9_report.py <navigation-report.json>
python3 tools/check_stage9_sim_report.py <simulator-report.json>
```

2026-07-17最终带RViz完整回归：

| 指标 | 实测结果 |
|---|---:|
| 构建/包级契约测试 | 通过 / 38项通过 |
| 独立完整回归 | 3/3通过，共9/9目标成功 |
| VGL相机模式 | `front_stereo`，周边相机消息数0 |
| 带RViz目标1误差 | 0.199 m / 8.80° |
| 带RViz目标2误差 | 0.249 m / 5.48° |
| 带RViz目标3误差 | 0.039 m / 9.93° |
| 三轮ground-truth实际路径 | 5.024 / 5.519 / 5.151 m |
| 三轮强制恢复/目标续航 | 3 / 3 |
| unready宽限后最大命令 | 0.000 |
| dynamic map slice最大已知单元 | 66,880 |
| 带RViz dynamic ESDF最大点数 | 13,220 |
| 带RViz combined ESDF最大点数 | 114,811 |
| 带RViz combined map slice最大已知单元 | 183,232 |
| odom深度点云 | 581点 |
| 带RViz MPPI局部路径实测速率 | 18.11 Hz |
| 带RViz Nav2 raw/safe命令实测速率 | 18.16 / 18.20 Hz |
| 带RViz dynamic slice / ESDF实测速率 | 7.07 / 7.05 Hz |
| 带RViz combined ESDF实测速率 | 5.77 Hz |
| 带RViz TF / cuVSLAM status实测速率 | 62.06 / 8.25 Hz |
| 动态障碍 | 官方叉车、箱体、胶囊体均移动 |
| 三轮Robot与动态障碍接触 | 0 / 0 / 0 |
| 仿真时间回退/最终重叠 | 0 / 0 |
| 官方资产改动 | 无 |
| stage打开次数 | 1 |
| RViz | 正常启动、订阅动态输出、干净退出 |

带RViz一轮仿真推进101.57 s，墙钟运行123.85 s。完整栈负载下深度安全链约4.10 Hz；因此阶段9的健康超时使用2 s，但最终Twist看门狗仍为0.25 s。Nav2日志中的单次SmacPlanner2D非圆形footprint inflation提示是Jazzy 1.3.12已记录的configure-time false positive；实际全局InflationLayer为1.0 m，三目标均成功，未出现运行期规划器失败。

用于最终记录的运行入口为：

```bash
PHASE9_TRIALS=3 PHASE9_GOAL_TIMEOUT_S=240 \
  ./scripts/run_phase9_tests.sh warehouse_v2_front
```

## 10. 当前边界

- 前向相机被近距离大物体完全遮挡时，系统优先停车并等待视野恢复，然后最多重试三次；本阶段不依赖侧后相机绕过遮挡。
- dynamic nvblox只接收前向深度，因此机器人后方新出现的障碍必须先进入前向视野才能建立动态层；倒车速度已被严格限制，Collision Monitor仍负责当前前向深度可见区域。
- 阶段9验证的是功能闭环和3–5次回归入口，不代表最终各10次的统计验收；按用户范围，光照和颜色随机化不在本次正式成功率中。
