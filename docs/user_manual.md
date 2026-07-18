# Nova Carter 视觉导航用户操作手册

本手册面向已经完成本仓库裸机安装的 Ubuntu 24.04 / ROS 2 Jazzy / Isaac Sim 6.0.1 工作站。正常操作不需要在 Isaac Sim 中拖拽 USD、不需要点击 Play、不需要在 RViz 中手工设置初始位姿或目标。

## 1. 系统边界

当前冻结方案使用 Nova Carter 前向 Hawk 双目、前向 IMU 和 Isaac Sim 原生前向深度；不启用 lidar，也不创建侧向或后向相机图。cuVSLAM 负责连续视觉跟踪，前向双目 cuVGL 负责启动与失锁后的全局重定位，dynamic nvblox 和前向深度 LaserScan 负责三维/局部障碍感知，Nav2 负责规划与 MPPI 差速控制。

`/ground_truth/odometry` 和 PhysX 接触只用于离线评估，绝不输入定位、规划或控制。轮式里程计只做短时平滑预测；是否允许运动仍由视觉定位健康、深度新鲜度和 nvblox 新鲜度共同控制。因此这里的“纯视觉导航”是无 lidar、无 ground truth 导航输入的视觉环境感知与视觉定位系统，不等同于禁用 IMU 或轮速反馈。

## 2. 第一次使用

打开一个干净终端：

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/build.sh
```

成功标志是三个 ROS 包完成构建，并且环境检查报告同时显示：

- Isaac Sim `6.0.1.0`；
- RTX 4090 与驱动可访问；
- CUDA 13.0、TensorRT 10.13.3.9；
- cuVSLAM、nvblox、Visual Global Localization 均为 Isaac ROS 4.5.0；
- 官方 Warehouse 和 Nova Carter USD 能被实际打开。

若是全新的另一台电脑，请先按[裸机安装手册](installation.md)完成依赖，再执行本节命令。脚本不会修改 `~/.bashrc`。

## 3. 日常完整导航

带 Isaac Sim GUI 和 RViz 的自动动态导航：

```bash
./scripts/run_phase9.sh --map warehouse_v2_front --gui --rviz
```

低开销 headless 运行：

```bash
./scripts/run_phase9.sh --map warehouse_v2_front --headless --no-rviz
```

该入口会依次启动本项目专属 Fast DDS discovery server、Isaac Sim Standalone、前向传感器、cuVSLAM/cuVGL、dynamic nvblox、Nav2 和自动目标执行器；结束时只清理自己创建的进程组。`Ctrl-C` 可以安全停止整套系统。不要使用 `killall` 或全局 `pkill`，机器上可能还有其他项目。

不改代码而执行一个或多个自定义 `map` 坐标目标，可传入扁平的 `x,y,yaw` 三元组：

```bash
PHASE9_GOAL_POSES='[2.0,0.0,0.0,1.8,10.5,1.570796327]' \
  ./scripts/run_phase9.sh --map warehouse_v2_front --headless --no-rviz
```

脚本仍会自动全局重定位、逐个发送目标并验证数据链。目标必须位于当前occupancy map的连通自由空间；yaw单位为弧度。需要正式可比较的结果时不要自定义目标，使用阶段11固定六目标和固定seed。

阶段11的单目标、固定 seed、完整指标运行更适合复现实验：

```bash
./scripts/run_stage11_trial.sh \
  --class heterogeneous --seed 41000 --goal-index 5 \
  --headless --no-rviz --record-bag
```

其中：

- `--class` 为 `static`、`dynamic` 或 `heterogeneous`；
- `--goal-index` 为 0–5，3–5 是跨越多个仓库区域的长距离目标；
- `--seed` 固定障碍尺寸、初相位和速度；
- `--record-bag` 记录压缩 MCAP，`--no-bag` 用于快速调试；
- 每次运行自动发送目标，不允许人工干预。

## 4. 地图准备与重建

当前导航默认使用已生成的 `warehouse_v2_front`：

```text
data/maps/warehouse_v2_front/
├── occupancy/map.yaml
├── occupancy/map.pgm
├── cuvslam/data.mdb
├── cuvgl/bow_index.pb
└── config/
```

若地图缺失或需要重新建立前向视觉地图：

```bash
./scripts/run_phase9_mapping.sh --map warehouse_v2_front
```

该命令自动采集、保存 cuVSLAM/nvblox/PLY/occupancy、生成 cuVGL 地图并准备 TensorRT 引擎。详细输入、同步和调参分别见[建图手册](mapping.md)、[cuVSLAM手册](cuvslam_configuration.md)、[cuVGL手册](cuvgl_configuration.md)和[nvblox手册](nvblox_configuration.md)。

阶段11还需要根据官方场景实际 CollisionAPI 生成理论最优路径基准：

```bash
./scripts/prepare_stage11_reference.sh --map warehouse_v2_front
```

输出在 `data/reference/warehouse_usd_005/`。它会重新打开固定官方 Warehouse USD，提取真实碰撞体，按 Nova Carter 带 padding 的不对称 footprint 生成 5 cm 栅格，并运行 8 航向 SE(2) A*；不会修改官方 USD。

## 5. RViz 与第三人称视角

`--rviz` 会加载 `nova_carter_bringup/rviz/navigation.rviz`，包含：

- RobotModel 和完整 TF；
- occupancy map、全局/局部代价地图；
- Smac 全局路径与 MPPI 局部路径；
- 前向左右图像、深度和 LaserScan；
- nvblox mesh、ESDF 和 combined map slice；
- footprint、Collision Monitor 区域和定位状态。

`--gui` 模式会创建 `/World/FollowCameraRig` 并平滑跟随机器人。第三人称相机只用于观察，不发布 ROS 图像，也不参与导航。headless 模式不创建 viewport 依赖。

## 6. 阶段11正式验收

完整验收固定执行静态 40 次、动态 40 次、异构动态 50 次：

```bash
./scripts/run_acceptance.sh \
  --matrix-id phase11-final-v1-20260718 \
  --record-bag
```

中断后使用同一个 matrix ID 继续；脚本只复用身份、seed、目标和状态均匹配的通过轮：

```bash
./scripts/run_acceptance.sh \
  --matrix-id phase11-final-v1-20260718 \
  --resume --skip-build --record-bag
```

如果仿真ready之后、ROS目标运行器启动之前发生进程级瞬时中断，该空轮没有`navigation.json/goals`，不会占用正式成功率的失败预算；脚本默认最多自动重试2次。只要目标已经实际发出，后续任何导航、碰撞、定位或性能失败都照常计入分母，不能靠重试隐藏。

正式输出：

```text
data/reports/phase11/acceptance/<matrix-id>/summary.json
data/reports/phase11/acceptance/<matrix-id>/trials.csv
data/reports/phase11/acceptance/<matrix-id>/report.md
data/reports/phase11/acceptance-summary-latest.json
```

每轮的权威结论是 `data/runs/<run-id>/result.json`。只有目标误差、碰撞、定位安全、深度/地图新鲜度、频率、实时因子、命令时延、平滑性和场景有效性全部通过，该轮才是 `passed`。最终矩阵还检查各类别成功率、长距离成功率和成功轨迹伸长率 P95。

按当前项目范围，光照和颜色随机化明确关闭；正式验收不包含光照/颜色变化，不应把这项排除误读为已通过该类泛化测试。

## 7. 常用结果读取

查看最近正式汇总：

```bash
jq '{status, trial_count, classes, aggregate}' \
  data/reports/phase11/acceptance-summary-latest.json
```

查看单轮失败项：

```bash
jq '{status, checks, path, goal_results, observed_rates_hz}' \
  data/runs/<run-id>/result.json
```

关键产物说明：

- `scenario.yaml/json`：本轮固定 seed 场景与目标；
- `navigation.json`：ROS 数据流、终点、轨迹、命令与时延原始统计；
- `simulator.json`：实际资产、仿真时间、传感器图、actor 运动和 PhysX 接触；
- `trajectory.csv`：只用于指标的 ground truth 轨迹；
- `command_trace.csv`：四级速度链及平滑性；
- `gpu.csv`：利用率、显存、功耗；
- `rosbag/`：压缩 MCAP 证据；
- `result.json`：单轮最终机器判定。

## 8. 安全停止与并发规则

- 正常前台运行按一次 `Ctrl-C`；自动脚本会先停止 ROS bag 和 Nav2，再通过 stop sentinel 关闭 Isaac Sim。
- 同一时刻只允许本项目一个 Stage11 trial/矩阵和一个 Isaac Sim，`flock` 会拒绝重复运行。
- 不要删除正在使用的 `data/locks/*`、run目录或 MCAP。
- 不要把 `/ground_truth/odometry` 接到 EKF、Nav2、TF 或控制器。
- 不要绕过 Command Guard、Collision Monitor、定位 ready 或 stale-data 检查来“提高成功率”。

## 9. 故障诊断

先收集只读诊断：

```bash
./scripts/collect_diagnostics.sh
```

报告写入 `data/logs/diagnostics/`。进一步按[故障排查手册](troubleshooting.md)定位。常见顺序是：

1. `result.json/checks` 找唯一失败门；
2. `ros.log` 检查 cuVSLAM、VGL、Nav2 lifecycle 和 Command Guard；
3. `simulator.json` 检查时间、传感器图、actor 运动和接触；
4. `gpu.csv` 检查外部GPU负载和实时因子；
5. 修复后换新 run ID 重跑，不覆盖原始证据。

完整文件导航见[项目重要文件索引](file_index.md)。
