# 酷家乐 Jackal 用户操作手册

本手册只描述 `codex/kujiale-jackal-8cam` 的静态视觉导航流程。历史 Warehouse/Nova Carter 和动态实验入口不适用于本分支。

## 1. 前置检查

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/build.sh
```

确认 `config/environment.env` 中三个资产路径存在。脚本会核对 USD default prim 与 SHA-256，并构建：

- `jackal_control`
- `jackal_bringup`
- `jackal_experiments`
- `jackal_teleop`

不要同时启动两个本项目 Isaac Sim；`data/locks/navigation_sim.lock` 会拒绝重复实例，但不会终止其他项目的进程。

## 2. 第一次建图

推荐先运行自动闭环覆盖：

```bash
./scripts/run_mapping.sh --map "kujiale_rebuild_$(date +%Y%m%d_%H%M%S)" --auto --headless
```

该模式沿 `config/mapping_coverage.yaml` 的约 38 m 闭环低速行驶，不倒车，并在开放区域
扫描。参考分支的旧 occupancy 只用于规划路线，绝不会作为新地图输出。八路图像、
cuVSLAM/cuVGL、nvblox、mesh 和 occupancy 都来自本轮采集；cuVSLAM 回环优化后的公共帧
分别输入 cuVGL 和 nvblox，最终 occupancy 不依赖 cuVGL 的二次选帧，也不直接保存在线预览。

需要人工控制和 GUI 时运行：

```bash
./scripts/run_manual_mapping.sh --map kujiale_manual_20260719
```

键位：

| 键 | 动作 |
|---|---|
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 左转 / 右转 |
| `Space` | 立即停车 |
| `Q` | 停车、结束录制并保存地图 |

安全规则：

- 松开运动键超过 0.18 秒自动发零速度。
- 建图可以人工后退；导航验收不允许负向线速度。
- 低速覆盖所有房间、门洞和走廊，转弯时给四组 Hawk 留出重叠视野。
- 回到已走过区域形成闭环后再按 `Q`。
- 不要直接关闭终端或强杀进程；失败时 raw bag 会保留在忽略目录，便于诊断。

人工模式会同时打开 Isaac Sim 第三人称跟随 GUI 和建图 RViz。详细的保存阶段、窗口说明
与故障处理见[手动键盘建图、保存与 RViz 导航全流程](manual_mapping_navigation.md)。

自动和人工模式都会在晋升地图前检查 PhysX 接触。与地面共面的门洞底面只有在低位、
近竖直法向且接触者是轮子/caster 时才归类为支撑；其他任何机器人非地面接触仍会使
建图失败并保留现场。

脚本会等待 8 个标准化图像话题，临时录制 MCAP，并保存：

- 8 相机 cuVSLAM 数据库；
- 8 相机 cuVGL keyframes、vocabulary 和 BoW index；
- front 原生深度按优化位姿重新融合的 nvblox `.nvblx` 与 PLY mesh；
- Nav2 occupancy `map.yaml` / `map.pgm`；
- cuVGL 同步配置与 `manifest.json`。

成功后 EDEx/抽帧中间目录自动删除，raw bag 默认保留在
`data/bags/<地图名>_<run-id>/capture`；确认地图和导航后可自行归档，或在建图命令显式传
`--discard-bag`。地图目录非空时脚本拒绝覆盖，重建请使用新地图名或先由用户自行归档旧地图。

## 3. 地图检查与目标校准

```bash
python3 tools/check_map_manifest.py data/maps/kujiale_latest_20260719_160004
python3 tools/validate_acceptance_routes.py \
  data/maps/kujiale_latest_20260719_160004 \
  --config config/acceptance.yaml \
  --output data/reports/route-validation.json
```

若路线验证失败，根据 occupancy map 修改 `config/acceptance.yaml` 的 `goals[].pose`。每个目标必须：

- 在地图范围内且属于已知自由空间；
- 按 `0.28 m` 保守圆形代理膨胀后仍安全；
- 与 map 原点 `[0, 0]` 连通；
- 原点到目标的直线穿过障碍，以保证测试包含实际绕行。

候选目标只有经过真实地图验证后才能用于正式统计。

## 4. 导航运行

快速自动路线：

```bash
./scripts/run_all.sh --map kujiale_latest_20260719_160004 --headless --no-rviz
```

需要观察时：

```bash
./scripts/run_all.sh --map kujiale_latest_20260719_160004 --gui --rviz
```

需要自己在 RViz 发布目标时使用手动入口：

```bash
./scripts/run_manual_navigation.sh --map kujiale_latest_20260719_160004
```

终端出现 `MANUAL_NAVIGATION_READY` 后选择 RViz `2D Goal Pose`。cuVGL 自动把当前
视觉位置锚定到保存地图，不使用 `2D Pose Estimate`，也不需要手工标定出生点。

推荐使用手动入口统一管理 GUI、RViz、自动定位门禁和 DDS discovery server。需要从第二个
终端只读检查当前链路时运行：

```bash
./scripts/check_manual_navigation.sh
```

此时可在 RViz 用 `2D Goal Pose` 发目标。导航始终使用 front、left、right 三组 Hawk 的 6 路图像；back Hawk 不创建 render product。nvblox 仍只接 front 原生深度。

Nav2 的线速度下限为 `0.0 m/s`，Behavior Server 只有 Spin 和 Wait，行为树没有 BackUp/DriveOnHeading。任何小于 `-0.01 m/s` 的最终命令都会使正式实验失败。

## 5. 静态避障正式统计

先跑一个单轮排查：

```bash
./scripts/run_static_trial.sh \
  --map kujiale_latest_20260719_160004 --goal-index 0 --attempt-index 1 --headless
```

再跑正式批次：

```bash
./scripts/run_static_acceptance.sh \
  --map kujiale_latest_20260719_160004 --headless
```

批次按三个目标 round-robin，直到得到 20 次有效实验。有效实验开始后，下列任一项都会记失败：

- 未到达目标或超时；
- PhysX 检测到任何非地面机器人接触；
- cuVSLAM/主 TF/Command Guard 不健康；
- 存在人工干预标记；
- 最终命令轨迹包含倒车；
- simulator 未通过或仿真时间显著回退；
- 动态环境被启用；
- 不是 6 路导航，或创建了后向 render product。

通过要求为：

```text
collision_free_passage_count / valid_trial_count >= 0.95
valid_trial_count >= 20
```

20 次有效实验时，19/20 通过，18/20 不通过。基础设施无效尝试仍列在汇总中，但不进入有效分母。

2026-07-19 本机正式批次结果为 20 个有效实验、19 次无碰撞通行、0 次物理接触、
0 次基础设施无效尝试，即 `19/20 = 95.00%`。第 20 次因局部控制停顿超时未到达，
但定位、仿真和碰撞检查均健康。逐轮分布见
[酷家乐 Jackal 验证记录](kujiale_jackal_validation.md)。

## 6. 性能实测

```bash
./scripts/run_performance_benchmark.sh \
  --profile all --map kujiale_latest_20260719_160004 --gui
```

也可单独运行：

```bash
./scripts/run_performance_benchmark.sh --profile mapping_8cam --gui
./scripts/run_performance_benchmark.sh \
  --profile navigation_6cam --map kujiale_latest_20260719_160004 --gui
```

无人观察时可以把 `--gui` 换成 `--headless`，但两种模式是不同工况，结果不可混写。

`mapping_8cam` 会启动实际 8 路 cuVSLAM + nvblox 负载；`navigation_6cam` 会加载地图、cuVGL、Nav2 和 nvblox。性能预热只在工作负载全部 ready 后开始。

默认自适应参数在 `config/acceptance.yaml/performance.adaptive_sampling`。可用脚本参数临时改变最小/最大预热与采样墙钟时长，但不允许把文档示例数值变成通过门槛。

报告重点查看：

- `official_isaac_sim_6_0_1.mean_fps`
- `official_isaac_sim_6_0_1.real_time_factor`
- App/Physics frametime 的 mean、P95、P99
- 整个项目进程树 RSS/VMS/USS
- GPU utilization、memory、power、temperature
- `host_context.cpu_governors` 和 NVIDIA driver

保持 1280×720 GUI 第三人称跟随视口、不缩预览并启用实际 ROS 工作负载时，本机最终
自适应稳定样本为：

| Profile | Mean FPS | RTF | App mean ms | Physics mean ms |
|---|---:|---:|---:|---:|
| `mapping_8cam`（8 路） | 22.462 | 0.374 | 44.524 | 13.322 |
| `navigation_6cam`（6 路） | 24.290 | 0.405 | 41.170 | 14.127 |

它们是 RTX 4090 本机观测值，不是验收阈值。两者都关闭 lidar，导航不创建后 Hawk
render product。优化配置、官方依据和被回退的实验见
[performance_optimization.md](performance_optimization.md)，最终运行证据见
[kujiale_jackal_validation.md](kujiale_jackal_validation.md)。

## 7. 结果与停止

| 路径 | 内容 |
|---|---|
| `data/logs/mapping/` | 建图过程日志 |
| `data/runs/static-acceptance/` | 单轮原始结果 |
| `data/reports/static-acceptance/` | 静态统计 JSON/CSV/Markdown |
| `data/reports/performance/` | 8 路/6 路性能观测 |
| `data/maps/kujiale_latest_20260719_160004/` | 当前本机保留且由 Git 忽略的运行时地图 |
| `data/bags/kujiale_latest_20260719_160004_20260719T080027Z/` | 与当前地图对应、必须保留的原始 MCAP |

前台运行按一次 `Ctrl-C`。自动脚本只停止自己创建的进程组，并通过 stop file 让 Isaac Sim 写完报告；不要使用 `killall` 或全局 `pkill`。

地图和 raw bag 均由 Git 忽略并保留在本机；日志、实验 run 和报告默认可清理。正式结果是否达标以新生成的 `summary.json` 为准，不能用短时技术烟测代替。
