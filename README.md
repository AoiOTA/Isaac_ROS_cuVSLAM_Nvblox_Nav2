# Nova Carter Isaac ROS Visual Navigation

本仓库用于在 Ubuntu 24.04、ROS 2 Jazzy、Isaac Sim 6.0.1 和 RTX 4090 上构建 Nova Carter 视觉导航系统。目标组件包括 Isaac Sim Standalone Python、Isaac ROS cuVSLAM、nvblox、Visual Global Localization 和 Nav2。

当前状态：**阶段0和阶段1已完成并在本机实测通过。** 已安装CUDA Toolkit 13.0.3、TensorRT 10.13.3.9和Isaac ROS 4.5.0；仿真导航主体从阶段2开始实现。

## 固定资产

- 场景：`/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd`
- 机器人：`/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd`
- Isaac Sim Python：`/home/lyb/miniconda3/envs/isaacsim/bin/python`
- ROS 2：`/opt/ros/jazzy`

项目只加载 Nova Carter 主资产。不会加载或复用 `Nova_Carter_ROS.usd` 中的官方ROS OmniGraph；后续图将由 Standalone Python 在运行时创建。

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
仿真、建图、导航和验收入口会在对应阶段实现，在实现前不会用空壳脚本冒充可用功能。

本机阶段1验收已确认：

- CUDA测试内核在RTX 4090上编译、执行并返回预期结果。
- TensorRT成功构建并序列化最小网络引擎。
- cuVSLAM、nvblox和VGL组件均能实际装载并保持等待输入。
- Isaac Sim 6.0.1运行时自建OmniGraph发布的Clock、Image和CameraInfo可由系统Jazzy接收。

详细设计见 [docs/architecture.md](docs/architecture.md)，环境与后续安装约束见 [docs/installation.md](docs/installation.md)。
