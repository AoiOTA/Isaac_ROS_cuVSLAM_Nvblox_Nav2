# Nova Carter Isaac ROS Visual Navigation

本仓库用于在 Ubuntu 24.04、ROS 2 Jazzy、Isaac Sim 6.0.1 和 RTX 4090 上构建 Nova Carter 视觉导航系统。目标组件包括 Isaac Sim Standalone Python、Isaac ROS cuVSLAM、nvblox、Visual Global Localization 和 Nav2。

当前状态：**阶段0至阶段7已完成并在本机实测通过。** 已安装CUDA Toolkit 13.0.3、TensorRT 10.13.3.9和Isaac ROS 4.5.0；Standalone程序直接打开官方Warehouse，在匿名session layer中引用Nova Carter主USD，并在运行时自建控制、前向双目、深度和IMU OmniGraph。前向双目+IMU已接入cuVSLAM，主TF链、连续定位及地图保存/加载均已通过自动验收；Isaac Sim原生深度已接入nvblox；MCAP、cuVSLAM、cuVGL、nvblox、Mesh和occupancy地图已能自动生成，五次独立重启的cuVGL全局定位与cuVSLAM恢复全部通过。

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
自动地图集合、导航和最终验收入口会在对应阶段实现，在实现前不会用空壳脚本冒充可用功能。

本机阶段1验收已确认：

- CUDA测试内核在RTX 4090上编译、执行并返回预期结果。
- TensorRT成功构建并序列化最小网络引擎。
- cuVSLAM、nvblox和VGL组件均能实际装载并保持等待输入。
- Isaac Sim 6.0.1运行时自建OmniGraph发布的Clock、Image和CameraInfo可由系统Jazzy接收。

详细设计见 [docs/architecture.md](docs/architecture.md)，环境与后续安装约束见 [docs/installation.md](docs/installation.md)。
