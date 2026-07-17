# Nova Carter Isaac ROS Visual Navigation

本仓库用于在 Ubuntu 24.04、ROS 2 Jazzy、Isaac Sim 6.0.1 和 RTX 4090 上构建 Nova Carter 视觉导航系统。目标组件包括 Isaac Sim Standalone Python、Isaac ROS cuVSLAM、nvblox、Visual Global Localization 和 Nav2。

当前状态：**阶段0至阶段4已完成并在本机实测通过。** 已安装CUDA Toolkit 13.0.3、TensorRT 10.13.3.9和Isaac ROS 4.5.0；Standalone程序直接打开官方Warehouse，在匿名session layer中引用Nova Carter主USD，并在运行时自建控制、前向双目、深度和IMU OmniGraph。

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

## 阶段4仿真、控制与传感器入口

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
建图、导航和最终验收入口会在对应阶段实现，在实现前不会用空壳脚本冒充可用功能。

本机阶段1验收已确认：

- CUDA测试内核在RTX 4090上编译、执行并返回预期结果。
- TensorRT成功构建并序列化最小网络引擎。
- cuVSLAM、nvblox和VGL组件均能实际装载并保持等待输入。
- Isaac Sim 6.0.1运行时自建OmniGraph发布的Clock、Image和CameraInfo可由系统Jazzy接收。

详细设计见 [docs/architecture.md](docs/architecture.md)，环境与后续安装约束见 [docs/installation.md](docs/installation.md)。
