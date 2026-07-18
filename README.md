# Nova Carter Isaac ROS Visual Navigation

本仓库用于在 Ubuntu 24.04、ROS 2 Jazzy、Isaac Sim 6.0.1 和 RTX 4090 上构建 Nova Carter 视觉导航系统。目标组件包括 Isaac Sim Standalone Python、Isaac ROS cuVSLAM、nvblox、Visual Global Localization 和 Nav2。

当前状态：**阶段0至阶段11的代码、配置和自动化已经完成；阶段10已正式提交，阶段11按用户最终口径执行静态/动态/异构各10次的30轮正式矩阵。** 已安装CUDA Toolkit 13.0.3、TensorRT 10.13.3.9和Isaac ROS 4.5.0；Standalone程序直接打开官方Warehouse，在匿名session layer中引用Nova Carter主USD，并在运行时自建控制、前向双目、深度和IMU OmniGraph。当前项目决策固定只使用前向Hawk双目，不启动侧向或后向相机；cuVSLAM连续跟踪、前向cuVGL全局重定位、dynamic nvblox、Nav2 MPPI DiffDrive、Velocity Smoother、Collision Monitor、Command Guard、自动暂停/恢复导航和阶段11验收自动化已组成完整动态导航闭环。

第一次运行请看[用户操作手册](docs/user_manual.md)，查找代码和配置职责请看[项目重要文件索引](docs/file_index.md)，完整数据链和TF所有权见[架构说明](docs/architecture.md)。

## 固定资产

- 场景：`/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd`
- 机器人：`/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd`
- Isaac Sim Python：`/home/lyb/miniconda3/envs/isaacsim/bin/python`
- ROS 2：`/opt/ros/jazzy`

项目只加载 Nova Carter 主资产。不会加载或复用 `Nova_Carter_ROS.usd` 中的官方ROS OmniGraph；项目图由 Standalone Python 在 `/World/Graphs` 下运行时创建。

## 公共检查

从任意终端执行：

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/build.sh
```

该命令执行：

- ROS 2 Jazzy和Nav2是否可发现。
- RTX 4090和驱动是否可访问。
- Isaac Sim 6.0.1解释器是否可用。
- 两个固定USD能否由USD API打开。
- Nova Carter主资产中是否存在左右驱动轮、相机，以及是否未内置OmniGraph。
- Bash、Python和Fast DDS XML配置语法。
- Isaac ROS已安装时，附加检查CUDA、TensorRT共享库和五个核心ROS包。

该命令不会安装依赖、启动Isaac Sim GUI、修改官方USD或终止其他进程。

## 阶段2至阶段6仿真、定位与重建入口

Headless模式：

```bash
./scripts/run_sim.sh --headless
```

GUI模式：

```bash
./scripts/run_sim.sh --gui
```

测试时可添加`--duration 60`，使程序按墙钟运行60秒后自动退出。程序只打开一次Warehouse stage，在匿名session layer中创建`/World/NovaCarter`引用，强制选择`Physics=physx`、`Sensors=All_Sensors`和`ROS=Disabled`，并从场景碰撞几何自动搜索出生点。它不会加载`Nova_Carter_ROS.usd`、保存组合stage或管理其他项目的进程。

另一个终端启动ROS侧安全控制与轮式里程计：

```bash
./scripts/run_control.sh
```

输入为`/cmd_vel_safe`，安全输出为`/cmd_vel_sim`。Command Guard拒绝NaN/Inf、执行1.0 m/s和1.2 rad/s硬限幅、只保留`linear.x/angular.z`、进行加速度/jerk约束，并在0.25秒无命令时立即归零。仿真发布`/clock`、`/joint_states`和`/ground_truth/odometry`，ROS节点发布不带TF的`/wheel/odometry`。

一键执行阶段3实际物理验收：

```bash
./scripts/run_phase3_tests.sh
```

该入口自动启动并清理自身进程组，执行2 m直行、2 m倒车、原地180°、0.75 m半径圆弧、S弯、连续急转、超限、看门狗和非法命令测试。它使用临时ROS domain与机器上其他项目隔离，不终止外部Isaac Sim或ROS节点。

普通仿真日志位于`data/logs/stage3`；运动验收结果位于`data/reports/phase3`。报告记录图结构、被控关节、轨迹误差、轮里程计误差、平滑性、安全时延、PhysX重叠、timeline推进以及官方USD运行前后的文件指纹。

阶段2的组合基线见[Phase 2 Validation](docs/phase2_validation.md)，阶段3控制结果见[Phase 3 Validation](docs/phase3_validation.md)。

另一个终端可单独启动`robot_state_publisher`和双路Isaac ROS GPU灰度转换：

```bash
./scripts/run_sensors.sh
```

一键执行阶段4通信与运动联合验收：

```bash
./scripts/run_phase4_tests.sh
```

该测试抽检完整RGB/深度载荷，并持续验证30 Hz双目/深度、120 Hz Clock/IMU、mono8、CameraInfo、时间戳、TF和JointState；同时执行静止、左右圆弧、原地旋转和停车阶段。详细结果见[Phase 4 Validation](docs/phase4_validation.md)。

单独启动ROS侧前向双目灰度转换、robot_state_publisher和cuVSLAM：

```bash
./scripts/run_visual_slam.sh
```

完整阶段5启动使用`ros2 launch nova_carter_bringup phase5.launch.py`，它同时包含阶段3安全控制。自动验收入口为：

```bash
./scripts/run_phase5_tests.sh
```

该测试在隔离的ROS domain中自动执行122秒双向S形轨迹，验证cuVSLAM从未失锁、`map→odom→base_link`唯一且连续、视觉轨迹方向和尺度与只读ground truth一致，并实际调用地图保存、全部优化位姿读取和地图加载服务。最终实测连续成功跟踪123.10秒、地图包含1216个优化位姿。详细结果见[Phase 5 Validation](docs/phase5_validation.md)。

在另一台电脑上手动配置、迁移相机或逐项调参时，使用[cuVSLAM完整配置、迁移与调参手册](docs/cuvslam_configuration.md)。该手册包含输入数据契约、CameraInfo/baseline验证、TF和IMU配置、完整YAML、launch组织、地图服务、调参顺序、故障诊断和量化验收。

单独启动nvblox或启动完整阶段6：

```bash
./scripts/run_nvblox.sh
./scripts/run_phase6.sh
```

前者要求相机和cuVSLAM TF已经由其他进程提供；后者同时包含阶段3控制、阶段5 cuVSLAM和阶段6 nvblox。手动保存当前重建：

```bash
./scripts/save_nvblox_map.sh data/maps/warehouse_v1/nvblox warehouse
```

完整阶段6自动验收：

```bash
./scripts/run_phase6_tests.sh
```

最终实测完成15.48 m双向S形扫描且cuVSLAM零失锁；TSDF、Mesh、静态ESDF和map slice全部非空，保存得到104.6 MB nvblox地图和14.8 MB PLY。详细证据见[Phase 6 Validation](docs/phase6_validation.md)。在另一台电脑安装、迁移传感器和调参时，以[nvblox完整配置、迁移与调参手册](docs/nvblox_configuration.md)为准；其中记录了完整YAML、launch、TF/深度契约、保存服务、Nav2接入、调参顺序和2D ESDF服务限制。

## 阶段7自动建图和全局重定位

自动创建第一版对齐地图：

```bash
./scripts/run_mapping.sh --map warehouse_v1
```

该入口自动启动Standalone、控制、cuVSLAM和nvblox，执行闭环采集路线并录制MCAP，保存在线cuVSLAM、nvblox、PLY和Nav2 occupancy，然后调用官方离线工具生成对齐的cuVSLAM与cuVGL BoW地图，并导出/复用ALIKED、LightGlue TensorRT引擎。

审计地图或执行五次独立重启重定位：

```bash
python3 tools/check_phase7_maps.py data/maps/warehouse_v1
./scripts/run_phase7_tests.sh warehouse_v1
```

本机最终回归地图包含142个cuVGL关键帧，五个不同初始yaw均得到cuVGL位姿并通过`/visual_slam/initial_pose`恢复cuVSLAM。单独启动已有地图的cuVGL使用`./scripts/run_vgl.sh warehouse_v1`。详细结果见[Phase 7 Validation](docs/phase7_validation.md)；另一台电脑的完整安装、建图、launch、接口和调参过程见[cuVGL完整配置、迁移与调参手册](docs/cuvgl_configuration.md)。

## 阶段8 Nav2视觉导航与RViz

从干净终端启动已有地图的完整系统并自动执行默认三目标路线：

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/run_all.sh --map warehouse_v1 --headless --rviz
```

只启动ROS侧定位、重建、Nav2和RViz（要求另一个终端已运行本项目仿真）：

```bash
./scripts/run_navigation.sh --map warehouse_v1 --rviz
```

完整阶段8回归入口会重新构建、运行25项静态测试、执行带RViz的真实三目标导航，并启动GUI验证第三人称相机：

```bash
./scripts/run_phase8_tests.sh warehouse_v1
```

本机最终带RViz回归的三个目标全部成功，位置误差为0.195、0.240和0.193 m，航向误差为8.62°、3.07°和2.97°；真值轨迹4.64 m，四级速度链约20 Hz，局部MPPI路径约20 Hz，TF约46 Hz，深度时间戳无回退。详细架构、接口、调参依据与实测证据见[Phase 8 Validation](docs/phase8_validation.md)。

## 阶段9 前向双目动态导航与自动恢复

阶段9默认地图和完整启动入口为：

```bash
./scripts/run_phase9_mapping.sh --map warehouse_v2_front
./scripts/run_phase9_navigation.sh --map warehouse_v2_front --rviz
./scripts/run_phase9.sh --map warehouse_v2_front --headless --rviz
./scripts/run_phase9_tests.sh warehouse_v2_front
```

`run_phase9.sh`自动启动项目独占的本地Fast DDS发现服务器、Standalone仿真、前向双目cuVSLAM、前向双目cuVGL、dynamic nvblox、Nav2和可选RViz，并执行三目标路线。仿真在匿名session layer中驱动官方叉车以及项目创建的箱体和胶囊体；官方Warehouse和Nova Carter USD不会被保存或修改。

定位恢复链路会先撤销`/localization/ready`使最终Command Guard归零，再触发前向cuVGL。通过位姿创新门限并连续恢复cuVSLAM tracking后，`resilient_navigation`自动重发同一个Nav2目标；最多三次仍失败则保持安全停车。局部地图由dynamic nvblox输出的`combined_map_slice`提供，并由原生深度LaserScan和PointCloud2承担低延迟安全响应。完整设计、接口、命令、测试矩阵和实测结果见[Phase 9 Validation](docs/phase9_validation.md)。

本机最终执行3次独立完整重启回归（首轮带RViz），共9/9个目标成功、3/3次强制重定位后自动续航，三轮动态障碍接触均为0；38项包级契约测试全部通过。

## 阶段10 自动化、必要加固与预验收

阶段10按当前范围继续只使用前向双目，明确不做光照或颜色随机化。单次可复现实验、三类安全故障注入和20+20预验收入口为：

```bash
./scripts/run_phase10_trial.sh --class static --seed 1000 --goal-index 0 --record-bag
./scripts/run_phase10_trial.sh --class dynamic --seed 11000 --goal-index 0 --record-bag
./scripts/run_phase10_hardening.sh warehouse_v2_front
./scripts/run_phase10_preacceptance.sh --map warehouse_v2_front
```

完整回归入口`./scripts/run_phase10_tests.sh warehouse_v2_front`会执行构建、54项自动测试、真实故障硬化和静态20次/动态20次预验收。每轮都自动生成固定seed场景、启动隔离DDS和完整视觉导航栈、记录MCAP/轨迹/命令/GPU/PhysX接触、计算目标误差、路径伸长率与正常导航加速度/jerk，并只清理本轮创建的进程组。矩阵、单轮和run目录均有独占锁；Nav2只有在生命周期守卫确认8个managed node全部active后才开始试验，部分激活会自动完整重启ROS栈。最终实测静态20/20、动态20/20、40轮全部0碰撞，静态/动态路径伸长率P95分别为13.73%/13.08%，合并P95为13.58%，最低实时因子为0.763。三目标故障硬化和强制lifecycle失败后的干净重启也已实际通过。配置、统计口径、参数冻结和实测证据见[Phase 10 Validation](docs/phase10_validation.md)，实验产物说明见[Experiments](docs/experiments.md)。

## 阶段11 正式复杂场景与长距离验收

阶段11继续冻结为前向双目视觉系统，不启用lidar或四向相机；按本轮要求不执行光照和颜色随机化。新增能力包括：

- 从实际官方Warehouse `UsdPhysics.CollisionAPI`提取430个有效碰撞体；
- 按Nova Carter不对称footprint和0.03 m padding建立5 cm栅格；
- 以8航向SE(2) A*生成六个目标的独立理论最优路径；
- 三个短距离目标和三个9.5–12.3 m理论长度的跨区域长距离目标；
- 静态、3 actor动态和6 actor叉车/box/capsule异构动态场景；
- 单轮目标、碰撞、路径、定位安全、实时因子、频率、数据年龄、命令时延、加速度/jerk和GPU全项判定；
- 静态、动态、异构各10次的固定seed正式矩阵与安全断点续跑。

先生成USD理论基准，再运行单轮或正式矩阵：

```bash
./scripts/prepare_stage11_reference.sh --map warehouse_v2_front

./scripts/run_stage11_trial.sh \
  --class heterogeneous --seed 41000 --goal-index 5 \
  --headless --no-rviz --record-bag

./scripts/run_acceptance.sh \
  --matrix-id phase11-formal-20260718 --record-bag
```

中断后使用相同ID继续：

```bash
./scripts/run_acceptance.sh \
  --matrix-id phase11-formal-20260718 \
  --resume --skip-build --record-bag
```

最终代码已完成61项自动测试。实际代表性长距离运行中，静态目标完成12.50 m且路径伸长1.88%，异构动态目标完成11.68 m且路径伸长0.68%；两轮均为0碰撞，终点位置误差分别为3.2 cm和4.7 cm，实时因子为0.862/0.837，raw-to-sim命令新鲜度P95为47.0/54.7 ms，nvblox为7.10/7.18 Hz。正式30轮结果记录在[Phase 11 Validation](docs/phase11_validation.md)及`data/reports/phase11/acceptance-summary-latest.json`。

## 阶段1完整环境配置

另一台电脑需要重新配置环境时，以[阶段1裸机环境完整配置手册](docs/installation.md)为准。该文档把操作拆分为独立步骤，包含每条命令的目的、预期结果、安全门和失败恢复，不要求先运行本仓库的安装脚本。

## 阶段1自动化与复测入口

下面的入口用于本仓库自动化和已经配置好环境后的复测；它们不能替代上面的逐步安装文档。

```bash
# 无sudo、只读检查
./scripts/bootstrap_baremetal.sh --preflight

# 配置官方源、执行官方初始化，并在实际安装前做APT模拟
./scripts/bootstrap_baremetal.sh --install

# Isaac Sim运行时OmniGraph到系统Jazzy的实际消息测试
./scripts/smoke_ros_bridge.sh

# CUDA、TensorRT和Isaac ROS运行时测试
python3 tools/check_stage1.py

# cuVSLAM、nvblox和VGL组件实际装载并等待输入
./scripts/smoke_isaac_ros_nodes.sh
```

安装脚本不修改 `~/.bashrc`，不卸载系统OpenCV，并在APT模拟出现任何待删除包时自动停止。
不同光照/颜色实验按本轮用户决策不属于阶段10和阶段11正式范围。当前公开的阶段0至阶段11命令均为实际可执行入口。

本机阶段1验收已确认：

- CUDA测试内核在RTX 4090上编译、执行并返回预期结果。
- TensorRT成功构建并序列化最小网络引擎。
- cuVSLAM、nvblox和VGL组件均能实际装载并保持等待输入。
- Isaac Sim 6.0.1运行时自建OmniGraph发布的Clock、Image和CameraInfo可由系统Jazzy接收。

详细设计见 [docs/architecture.md](docs/architecture.md)，环境与后续安装约束见 [docs/installation.md](docs/installation.md)。
