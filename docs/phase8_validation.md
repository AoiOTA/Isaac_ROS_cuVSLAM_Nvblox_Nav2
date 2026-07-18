# 阶段8 Nav2、RViz与完整视觉导航验证

## 1. 完成结论

阶段8已在目标主机上实现并完成两轮真实三目标回归：一轮不启动RViz以隔离核心导航，一轮启动完整RViz并在结束后实际启动Isaac Sim GUI验证第三人称相机。最终公共入口为：

```bash
./scripts/run_navigation.sh --map warehouse_v1 --rviz
./scripts/run_all.sh --map warehouse_v1 --headless --rviz
./scripts/run_phase8_tests.sh warehouse_v1
```

最终完整测试同时使用了：

- Isaac Sim 6.0.1 Standalone Python。
- 官方Warehouse与Nova Carter主USD。
- 项目运行时自建OmniGraph。
- Isaac ROS cuVSLAM 4.5。
- Isaac ROS nvblox 4.5。
- Isaac ROS Visual Global Localization 4.5。
- Nav2 Jazzy 1.3.12。
- RViz2 14.1.22。

没有加载`Nova_Carter_ROS.usd`，没有复用其OmniGraph，没有使用lidar，也没有把ground truth或wheel odometry接入定位和规划。

## 2. 固定运行数据流

### 2.1 定位与TF

```text
front stereo + IMU
  -> cuVSLAM tracking odometry/status/slam_path

cuVGL pose
  -> vgl_pose_relay
  -> /visual_slam/initial_pose
  -> cuVSLAM map localization

cuVSLAM map-frame slam_path + odom-frame tracking odometry
  -> navigation_tf_bridge
  -> map→odom at current simulation time

cuVSLAM
  -> odom→base_link

robot_state_publisher
  -> base_link以下固定/关节TF
```

阶段5至阶段7仍可让cuVSLAM直接发布两条主TF。阶段8为了避免GPU管线延迟造成旧时间戳`map→odom`无法服务Nav2当前时间查询，启动参数固定为：

```text
override_publishing_stamp=true
publish_map_to_odom_tf=false
```

`navigation_tf_bridge`是阶段8中`map→odom`的唯一发布者。其两个输入都来自cuVSLAM：

```text
T_map_odom = T_map_base(slam_path) × inverse(T_odom_base(tracking odometry))
```

该适配器不订阅仿真真值，不订阅wheel odometry，也不改变cuVSLAM的定位结果；它只解决Nav2对“当前可查询TF”的要求。cuVSLAM仍独占`odom→base_link`。

### 2.2 深度与局部障碍

```text
/front_stereo_camera/depth/image_raw
  -> depthimage_to_laserscan
  -> /front_depth/scan_raw (base_link)
       ├-> Collision Monitor低延迟安全输入
       └-> scan_timestamp_relay
            + cuVSLAM odom→base_link TF
            -> /front_depth/scan
            -> /front_depth/points_odom
```

原始LaserScan不等待定位TF，确保遇到近障碍时仍能快速停车。同步relay只选取不晚于已观察cuVSLAM TF的扫描，并把点转换到`odom`，因此局部ObstacleLayer不会收到未来时间戳数据。

nvblox继续使用原生`32FC1`米制深度，输出：

- TSDF与Mesh。
- `static_esdf_pointcloud`。
- `static_map_slice`。

阶段8局部代价地图同时使用`static_map_slice`和`points_odom`。阶段9才会切换dynamic/combined map slice。

### 2.3 Nav2与命令链

```text
MapServer occupancy map
  -> global StaticLayer + InflationLayer
  -> SmacPlanner2D

nvblox static_map_slice + visual depth PointCloud2
  -> rolling local costmap in odom
  -> MPPI DiffDrive
  -> /cmd_vel_nav_raw
  -> Velocity Smoother
  -> /cmd_vel_smoothed
  -> Collision Monitor
  -> /cmd_vel_safe
  -> Command Guard
  -> /cmd_vel_sim
  -> Isaac Sim DifferentialController
  -> joint_wheel_left + joint_wheel_right
```

被动脚轮从未出现在命令数组中。

Command Guard除阶段3的有限值、平面投影、硬限幅、加速度/jerk与250 ms看门狗外，阶段8还检查：

- `/localization/ready`。
- cuVSLAM tracking状态与1.0秒新鲜度。
- 深度消息与0.50秒新鲜度。
- nvblox map slice存在性与1.0秒新鲜度。

任一条件失败都会立即清零命令并在`/control/guard_status`报告具体状态。

## 3. Nav2配置

完整配置位于：

```text
ros2_ws/src/nova_carter_bringup/config/nav2.yaml
```

### 3.1 全局规划

- `global_frame=map`。
- MapServer加载`data/maps/warehouse_v1/occupancy/map.yaml`。
- StaticLayer与InflationLayer。
- `SmacPlanner2D`。
- `allow_unknown=false`。
- 0.20 m规划容差。

### 3.2 局部规划和控制

- `global_frame=odom`滚动窗口。
- 6 m × 6 m，0.05 m分辨率。
- NvbloxCostmapLayer、PointCloud2 ObstacleLayer、InflationLayer。
- MPPI `motion_model=DiffDrive`，20 Hz。
- 本次阶段8测量时的最大线速度为0.55 m/s、最大角速度0.90 rad/s。当前共享导航配置
  已为手动阶段9提升至1.10 m/s、1.40 rad/s；本节数值保留为历史测量条件。
- 56步、0.05秒模型步长、1000条采样轨迹。
- `PoseProgressChecker`同时检查0.10 m平移和0.15 rad旋转进展。
- `StoppedGoalChecker`使用0.20 m与10°容差。

初次实测发现默认式权重只输出约0.03 m/s。该问题位于`/cmd_vel_nav_raw`，后续所有安全阶段都正确透传，因此不是Isaac Sim控制器、Velocity Smoother或Command Guard造成。最终调参为：

- `vx_std=0.30`。
- `temperature=0.15`。
- GoalCritic权重12。
- PathFollowCritic权重10。
- PathAlignCritic权重14。
- PreferForwardCritic权重5。
- CostCritic权重3，并保留完整footprint碰撞检查。
- VelocityDeadbandCritic权重20，线/角死区均为0.08。

调整后真实导航常用线速度约0.08–0.15 m/s，转弯时角速度约0.1–0.3 rad/s。Velocity Smoother与Command Guard继续限制加速度、减速度和jerk，未通过提高硬速度上限来换取速度。

### 3.3 安全区

Collision Monitor订阅`/front_depth/scan_raw`，固定三层策略：

- StopZone：近障碍强制停车。
- SlowdownZone：历史测量时速度缩放为0.35；当前手动阶段9为0.65，并保留独立急停区。
- FootprintApproach：按1.2秒碰撞时间预测减速/停车。

机器人footprint固定为：

```yaml
[[0.14, 0.25],
 [0.14, -0.25],
 [-0.607, -0.25],
 [-0.607, 0.25]]
```

局部与全局代价地图都增加0.03 m padding。

## 4. 启动顺序与生命周期

TensorRT/cuVGL、cuVSLAM、nvblox与Nav2若同时初始化，会显著增加GPU与DDS启动突发负载。阶段8采用固定分段启动：

1. 立即启动阶段7定位、深度LaserScan、TF适配器和VGL bootstrap。
2. 3秒后启动nvblox。
3. 12秒后创建Nav2节点和MapServer生命周期管理器。
4. 再等待4秒，启动导航生命周期管理器。
5. 测试器等待九个节点全部进入`active`后才发送目标。

九个受管节点是：

```text
map_server
controller_server
smoother_server
planner_server
behavior_server
velocity_smoother
collision_monitor
bt_navigator
waypoint_follower
```

MapServer和导航分别使用独立生命周期管理器，bond timeout为15秒。

## 5. RViz显示

配置文件：

```text
ros2_ws/src/nova_carter_bringup/rviz/navigation.rviz
```

固定显示：

- RobotModel和TF。
- occupancy map。
- 全局与局部costmap。
- `/plan`全局路径。
- `/transformed_global_plan`控制器局部路径。
- cuVSLAM VO/SLAM路径与定位状态。
- nvblox Mesh和静态ESDF pointcloud。
- 前向RGB与深度。
- TF同步LaserScan与odom PointCloud2。
- Collision Monitor stop/slow/approach区域。
- Nav2 Goal工具与Navigation面板。

实际GUI回归中RViz成功启动OpenGL 4.6，订阅两路pointcloud，创建424×624 occupancy/global costmap与120×120 local costmap，没有插件装载失败或崩溃。GPU全栈运行时偶有旧`base_link`显示消息因队列满被丢弃，导航数据链不使用该RViz队列。

## 6. Isaac Sim第三人称相机

GUI模式在session layer创建：

```text
/World/FollowCameraRig
```

相机默认位于机器人后方3 m、高1.8 m，注视`base_link`上方0.4 m，并使用指数平滑更新位置和姿态。GUI模式把它设为active viewport camera；headless模式明确不创建viewport依赖，也不发布ROS图像。

实际GUI测试结果：

```json
{
  "camera_enabled": true,
  "correct_prim": true,
  "simulation_passed": true,
  "viewport_active": true
}
```

## 7. 自动测试内容

`run_phase8_tests.sh`依次执行：

1. `build.sh`环境、资产、Python/Bash/YAML和ROS构建检查。
2. 25项pytest测试。
3. 带RViz的cuVGL、cuVSLAM、nvblox、Nav2三目标真实导航。
4. GUI Standalone第三人称相机测试。

三目标runner实时检查：

- 九个Nav2生命周期节点全部active。
- 三个NavigateToPose action成功且error code为0。
- 位置误差≤0.25 m，航向误差≤12°。
- map/global costmap/local costmap非空。
- 原始与TF同步深度扫描非空。
- odom PointCloud2非空。
- nvblox map slice非空。
- 深度扫描时间戳不回退。
- `map→odom→base_link`存在。
- cuVSLAM持续tracking。
- `/plan`和`/transformed_global_plan`非空。
- 四级速度话题均非空，最终命令确实产生运动。
- `linear.y`始终为0。
- ground truth只读轨迹长度至少3 m。
- Command Guard实际进入active。
- 要求RViz时，`/rviz2`节点实际存在。

每5秒进度日志还记录目标、map pose、ground truth、Guard状态、Collision Monitor动作以及四级速度，用于区分定位、控制器和安全链问题。

## 8. 最终实测证据

### 8.1 核心导航回归（无RViz）

报告：

```text
data/reports/phase8/navigation-20260717T093447Z.json
```

结果：

- 三目标全部成功。
- 位置误差：0.047、0.122、0.180 m。
- 航向误差：9.94°、9.82°、1.99°。
- ground truth轨迹：5.47 m。
- `/cmd_vel_nav_raw`：20.05 Hz。
- `/cmd_vel_smoothed`与`/cmd_vel_safe`：20.01 Hz。
- `/cmd_vel_sim`：87.04 Hz。
- 局部路径：20.02 Hz。
- TF：46.67 Hz。
- 扫描时间戳回退：0。

### 8.2 完整回归（带RViz）

报告：

```text
data/reports/phase8/navigation-20260717T093826Z.json
```

结果：

- 三目标全部成功，Nav2 error code均为0。
- 位置误差：0.195、0.240、0.193 m。
- 航向误差：8.62°、3.07°、2.97°。
- ground truth轨迹：4.64 m。
- 最大Nav2 raw线/角速度：0.148 m/s、0.289 rad/s。
- 最大最终sim线/角速度：0.159 m/s、0.303 rad/s。
- `/cmd_vel_nav_raw`：20.06 Hz。
- `/cmd_vel_smoothed`与`/cmd_vel_safe`：20.02 Hz。
- `/cmd_vel_sim`：85.36 Hz。
- 原始视觉安全扫描：19.52 Hz。
- TF同步扫描/odom点云：6.44 Hz。
- nvblox slice：6.35 Hz。
- local costmap：2.68 Hz。
- 局部路径：20.02 Hz。
- TF：46.41 Hz。
- map/global costmap：264576 cells。
- local costmap：14400 cells。
- nvblox slice：204744 cells。
- odom点云：每帧581 points。
- 扫描时间戳回退：0。
- RViz节点存在且配置完整加载。

GUI相机报告：

```text
data/logs/stage4/run-20260717T094007Z.json
```

- GUI运行8.00秒。
- 推进375帧、6.25秒仿真时间。
- `/World/FollowCameraRig`存在并绑定viewport。
- 官方USD运行前后保持未修改。

`data/`下的报告和日志由`.gitignore`排除，不提交大型运行产物；本文记录的是本机实际执行结果。

## 9. 已知非阻塞信息

### 9.1 SmacPlanner2D配置期错误日志

Nav2 Jazzy 1.3.12的`SmacPlanner2D`内部把2D碰撞检查器设为`radius=true`且`possible_collision_cost=0`，但该版本在判断radius fast path之前先打印通用的非圆footprint inflation错误。项目全局costmap确实包含1.0 m InflationLayer，两个三目标回归都成功。这是本版本2D路径的单次假阳性，不是规划失败。

### 9.2 视觉处理墙钟频率

物理频率和时间戳语义保持120 Hz/30 Hz配置，但Isaac Sim、RTX渲染、cuVSLAM、VGL、nvblox和RViz共同运行时，墙钟可处理的TF同步深度与VisualSlamStatus约6–7 Hz。原始深度安全扫描仍约19.5 Hz，cuVSLAM保持tracking，Nav2控制链保持20 Hz。阶段10可继续做GPU/渲染负载与实时因子优化，阶段8不以删减安全组件换取表面频率。

## 10. 阶段8边界

本阶段完成静态TSDF与已有occupancy map上的首个完整视觉导航MVP。以下内容属于后续阶段：

- 四向8图VGL。
- nvblox dynamic与combined map slice。
- 自动移动的动态障碍。
- 失锁后三次VGL重试与任务恢复状态机。
- 光照、颜色和材质随机化。
- 按用户最终口径执行静态/动态/异构各10次的正式统计验收。

阶段8的代码、配置和验收入口已经为这些扩展固定了TF所有权、深度安全链、生命周期时序、差速控制边界和可观测诊断数据。
