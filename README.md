# Kujiale Jackal 4-Hawk / 8-Image-Stream Visual Navigation

本分支面向 Ubuntu 24.04、ROS 2 Jazzy、Isaac Sim 6.0.1、Isaac ROS 4.5 和 RTX 4090，将仿真环境替换为酷家乐房间、机器人替换为 Clearpath Jackal，并安装四组 Hawk 双目。

当前实现状态：代码构建、离线契约测试以及 `mapping_8cam` / `navigation_6cam` 的 Isaac Sim headless 烟测已通过。实际地图尚未生成，因此正式导航、至少 20 次静态避障统计和本机完整性能观测仍需在手动建图完成后执行；仓库不会预填或伪造这些结果。

## 固定范围

| 项目 | 当前契约 |
|---|---|
| 场景 | `kujiale_0026_A_to_B_door_open.usd`，只打开一次 |
| 机器人 | Clearpath Jackal，四轮滑移转向 |
| 传感器 | front / left / right / back 四组 Hawk 双目，共 8 路 RGB 图像流 |
| 建图 | `mapping_8cam`：8 路全部发布并参与 cuVSLAM/cuVGL 建图 |
| 导航 | `navigation_6cam`：只发布 front / left / right，后向 render product 和 ROS publisher 均不创建 |
| nvblox | 只使用 front Hawk 左目产生的原生模拟深度，不使用 lidar、ESS 或 FoundationStereo |
| 场景运动 | 仅静态环境；动态 profile 会直接报错 |
| Nav2 | SmacPlanner2D + MPPI DiffDrive，只允许非负前向速度；恢复树没有 BackUp/DriveOnHeading |
| 验收 | 无碰撞通行次数 / 有效实验次数 ≥ 95%，且有效实验至少 20 次 |
| 性能 | 等实际 ROS 工作负载就绪后按墙钟自适应预热/采样；无 600 帧基线，无文档示例 KPI 门槛 |

官方 USD 不会被保存或修改。酷家乐 stage 只打开一次，Jackal、Hawk、物理修复和运行图均写入匿名 session layer。

## 数据链

```text
4 x Hawk stereo
  mapping: 8 RGB ──> cuVSLAM + offline cuVGL map
  navigation: 6 RGB ──> cuVSLAM + cuVGL localization

front native depth ──> nvblox static TSDF/ESDF
                   └─> LaserScan ──> Nav2 costmaps + Collision Monitor

Nav2 ──> Velocity Smoother ──> Collision Monitor ──> Command Guard
     ──> /cmd_vel_sim ──> four-wheel differential graph
     ──> bounded skid-steer motion assist + idle brake
```

Jackal 的轮心几何轮距为 `0.37559 m`，但差速控制与轮里程计使用参考分支标定后的有效轮距 `0.800 m`。运行时平面运动补偿只修正 PhysX 各向同性接触导致的系统性欠转，并保留命令超时、加速度边界、碰撞监控和空闲制动。

底盘与酷家乐适配以参考分支提交 `caae0c08a451544ec0362df12da165fdc8d75676` 为固定依据，特别对照了 [skid-steer 解决方案](https://github.com/AoiOTA/Isaac_Sim_ROS2_Nav/blob/codex/kujiale-navigation-mapping/docs/skid_steer_navigation_solution.md) 和 [酷家乐 USD 导航复盘](https://github.com/AoiOTA/Isaac_Sim_ROS2_Nav/blob/codex/kujiale-navigation-mapping/docs/kujiale_usd_navigation_postmortem_20260717.md)。本项目复用其轮子物理、控制时基、运动补偿、有效轮距、MPPI 转弯参数和只读 session-layer 修复；传感器改为四组 Hawk，定位改为 8/6 路视觉链，并按本次要求彻底移除导航倒车行为。

## 资产与环境

默认资产路径写在 `config/environment.env` 和 `config/assets.yaml`：

```text
/home/lyb/kujiale_usd_rooms_20260717/kujiale_0026/kujiale_0026_A_to_B_door_open.usd
/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Robots/Clearpath/Jackal/jackal.usd
/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd
```

首次使用先检查并构建：

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
git lfs install
./scripts/build.sh
```

## 1. 建立实际八路地图

推荐使用经过同场景参考占据图校核的自动闭环覆盖路线：

```bash
./scripts/run_mapping.sh --map kujiale_jackal_8cam --auto --headless
```

旧占据图只用于选择无碰撞行驶折线，不会复制到结果中。最终地图仍由本次实时四 Hawk
八路 MCAP、cuVSLAM/cuVGL 和 front Hawk 深度 nvblox 全流程生成。需要人工覆盖时使用：

```bash
./scripts/run_mapping.sh --map kujiale_jackal_8cam --interactive --gui
```

人工模式使用 `W/S` 前后、`A/D` 转向、`Space` 急停；松键超过 0.18 秒会自动停车，按 `Q` 停车并保存。自动模式只发非负线速度，并检查闭环完成度、cuVSLAM 跟踪、横向偏差和 PhysX 接触。

流程会临时录制 8 路同步 MCAP，从同一份数据生成 cuVSLAM 与 cuVGL 地图，并保存 nvblox、mesh、occupancy 和冻结配置。只有全部步骤成功后才删除 raw bag 与离线中间产物。默认输出：

```text
data/maps/kujiale_jackal_8cam/
├── config/
├── cuvgl/
├── cuvslam/
├── mesh/
├── nvblox/
├── occupancy/
└── manifest.json
```

检查地图：

```bash
python3 tools/check_map_manifest.py data/maps/kujiale_jackal_8cam
python3 tools/validate_acceptance_routes.py \
  data/maps/kujiale_jackal_8cam --config config/acceptance.yaml
```

`config/acceptance.yaml` 中的三个目标目前是候选值。路线验证器会要求出生点与目标均为已知自由空间、满足 Jackal 膨胀半径、连通，并且直线被障碍阻挡以确保确实发生绕行。若验证失败，应根据实际 occupancy map 修正目标后再做实验。

## 2. 导航

完整 headless 导航：

```bash
./scripts/run_all.sh --map kujiale_jackal_8cam --headless --no-rviz
```

带 GUI 与 RViz：

```bash
./scripts/run_all.sh --map kujiale_jackal_8cam --gui --rviz
```

如果仿真已由其他终端启动，只运行 ROS 侧：

```bash
./scripts/run_navigation.sh --map kujiale_jackal_8cam --rviz
```

导航入口会先验证地图 manifest，然后强制选择 `navigation_6cam`。后向相机不仅不订阅，也不会创建渲染资源。

## 3. 静态避障统计

正式批次：

```bash
./scripts/run_static_acceptance.sh \
  --map kujiale_jackal_8cam --headless
```

口径固定为：

- 至少 20 次有效实验；20 次时至少需要 19 次完整成功。
- 有效实验从 `navigation_test_runner` 被调用时开始。
- 此后目标超时、Nav2 失败、任何机器人非地面物理接触、定位不健康、人工干预、仿真时间异常、后向相机误启或负向线速度都计为失败并留在分母。
- runner 调用前的基础设施启动失败写入报告但不进入有效实验分母；脚本会重跑，连续 5 次基础设施失败则停止并保留现场。
- 每轮输出 `result.json`；批次输出 `summary.json`、`summary.csv` 和 `summary.md`。

只有汇总中的 `status: passed` 且 `collision_free_passage_rate >= 0.95` 才代表达到指标。

## 4. RTX 4090 实际性能观测

地图完成后同时测建图 8 路与导航 6 路：

```bash
./scripts/run_performance_benchmark.sh \
  --profile all --map kujiale_jackal_8cam --headless
```

脚本先启动真实 Isaac/ROS 工作负载，待话题、定位、nvblox 和 Nav2 就绪后才开始自适应预热；稳定后按墙钟采样，或在配置的最大时长停止。输出包含 Isaac Sim 官方 recorder 的 Mean FPS、Real Time Factor、App/Physics frametime，以及整个仿真与 ROS 进程树的 RSS/VMS/USS、GPU 利用率、显存、功耗和温度。

文档中的 benchmark Summary Report 仅是格式示例，本项目不会拿其中数值作通过门槛，也没有固定 600 帧停止条件。CPU governor、驱动和硬件信息会随报告记录；脚本不会擅自修改系统 governor。

本机保持 1280×720 GUI 跟随视口、四组 Hawk/八路图像和完整 ROS 工作负载时，最终 30 秒稳定采样为
22.732 FPS / 0.379 RTF；短窗口最好观测为 23.697 / 0.395。完整 A/B、官方依据、未采用方案和复测方法见
[RTX 4090 性能优化与实测](docs/performance_optimization.md)。

## 5. Git 与运行产物

只有 `data/maps/kujiale_jackal_8cam` 的运行时地图允许进入版本库，其中二进制、图像、nvblox 和 mesh 由 Git LFS 管理。raw `.mcap`/`.db3`、离线工作目录、TensorRT cache、日志、实验 run 和报告默认忽略。

地图生成后可检查 LFS 归属：

```bash
git check-attr filter -- data/maps/kujiale_jackal_8cam/nvblox/*.nvblx
git lfs status
```

## 6. 验证与文档

```bash
python3 -m pytest -q
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --return-code-on-test-failure
colcon test-result --verbose
```

- [用户操作手册](docs/user_manual.md)
- [架构与 TF 所有权](docs/architecture.md)
- [建图与地图产物](docs/mapping.md)
- [RTX 4090 性能优化与实测](docs/performance_optimization.md)
- [重要文件索引](docs/file_index.md)

旧的 phase9–phase11 动态实验脚本和验证文档保留用于历史回溯，但不属于本分支支持范围；正式入口仅以上述命令为准。
