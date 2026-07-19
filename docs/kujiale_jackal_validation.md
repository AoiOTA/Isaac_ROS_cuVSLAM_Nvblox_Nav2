# 酷家乐 Jackal 最终验证记录

本页汇总 2026-07-19 在本机 RTX 4090、Isaac Sim 6.0.1、ROS 2 Jazzy 和
Isaac ROS 4.5 上完成的正式静态验收与 GUI 性能观测。运行日志和 JSON/CSV 报告按仓库
策略保存在忽略目录中；本页记录命令、口径、关键统计和摘要哈希，便于本机复核。

本页数据对应历史地图 `kujiale_jackal_8cam`，该地图已按空间清理要求从当前工作区移除；
这些结果不能作为 `kujiale_latest_20260719_160004` 的验收结论，最新地图必须重新执行正式批次。

## 静态避障正式批次

运行命令：

```bash
./scripts/run_static_acceptance.sh \
  --map kujiale_jackal_8cam --headless \
  --output-dir data/reports/static-acceptance/20260719T003000-static-v3-20
```

运行基线：

- Git commit：`2486d6602859b96a8158657fd6ebc84a73eb9162`
- `config/simulation.yaml` SHA-256：
  `401ab08842740c1db69ca51cfb2514b977b6635ecaf9d773f4a1d5bb240ec257`
- 验收时地图 manifest SHA-256：
  `aa6ecee9f77cd93581004db28f02599fea411716a5bec66cf3a30b8ad27e6a8c`

正式汇总：

| 指标 | 结果 |
|---|---:|
| 有效实验 | 20 |
| 无碰撞通行 | 19 |
| 静态避障率 | **95.00%** |
| 要求 | ≥95.00%，且有效实验 ≥20 |
| 基础设施无效实验 | 0 |
| Jackal 非地面物理接触总数 | **0** |
| 汇总状态 | **passed** |

这里的“无碰撞通行”要求目标到达、无物理碰撞、定位健康、6 路导航相机、仿真正常、
无人工干预且无倒车全部同时成立。第 20 轮虽然物理接触仍为 0，但在 south_corridor
的 MPPI 局部停顿后超时，因此按失败留在有效分母中，不能写成 20/20。

路线分布与成功样本统计：

| 项目 | 结果 |
|---|---|
| 目标分布 | north_room 7、south_corridor 7、east_room 6 |
| 成功终点 XY 误差 | min 0.164 m、mean 0.186 m、max 0.208 m |
| 成功终点 yaw 误差 | min 2.195°、mean 6.341°、max 9.127° |
| 成功目标墙钟时长 | min 48.452 s、mean 77.910 s、max 143.275 s |
| 成功实际路径长度 | min 3.482 m、mean 4.838 m、max 5.696 m |
| 20 轮仿真 RTF | min 0.710、mean 0.741、max 0.760 |
| 首次 map/ground-truth XY 差 | min 0.001 m、mean 0.014 m、max 0.091 m |

20 轮全部满足：`navigation_6cam`、6 个活动图像流、后 Hawk render product 未创建、
lidar 关闭、最小命令线速度 `0.0 m/s`、simulator passed、定位健康。对 20 份
navigation bringup 日志检索 tracking lost、PnP failure 和 active-track failure，没有匹配项。

唯一失败轮为 `attempt-020 / south_corridor`：导航终止时距目标 2.770 m，runner
退出码为 1；simulator、定位、深度、nvblox、主 TF、Nav2 生命周期和物理碰撞检查仍通过，
物理碰撞数为 0。该结果使本批次恰好达到 95% 而没有额外余量，后续若修改控制器、地图或
Isaac Sim/驱动版本，应重新执行完整 20 轮，不能沿用本次比例。

本机报告摘要哈希：

```text
b3205b82af13b614d5d56783a991ddcaa1dfe929ad5c174ff2f1de81cab37605  summary.json
6b32c5e73d2dc9b1490061f1fdebd9818fc30052459313a8c048b732bf6c8cae  summary.csv
```

## GUI 性能正式观测

运行命令：

```bash
./scripts/run_performance_benchmark.sh \
  --profile all --map kujiale_jackal_8cam --gui \
  --output-dir data/reports/performance/20260719T051130-gui-all
```

性能脚本修复和最终运行契约对应 commit `6004646`。主机为 Intel Core i5-14600KF、
20 个逻辑 CPU、NVIDIA GeForce RTX 4090、驱动 595.71.05，CPU governor 为
`performance`。报告使用稳定后的自适应墙钟采样，没有固定 600 帧，也没有把官方文档
示例输出当作 KPI。

两个 profile 的共同条件：

- Isaac Sim GUI 开启，只显示 1280×720 第三人称跟随视口；
- `preview_resolution_reduced=false`，没有用缩小预览窗口优化；
- RTX Real-Time 2.0、DLSS Performance、cached retrace 0.1；
- lidar 关闭；
- ROS/cuVSLAM/nvblox 或 Nav2 真实工作负载就绪并产生物理运动后才开始预热。

最终自适应稳定样本：

| 指标 | `mapping_8cam` | `navigation_6cam` |
|---|---:|---:|
| 活动图像流 | 8 | 6 |
| 后 Hawk render product | 创建 | **不创建** |
| 预热时长 | 20.008 s | 30.033 s |
| 采样时长 | 35.014 s | 30.002 s |
| Mean FPS | **22.462** | **24.290** |
| Real Time Factor | **0.374** | **0.405** |
| App mean / P95 / P99 | 44.524 / 66.110 / 71.370 ms | 41.170 / 61.791 / 66.095 ms |
| Physics mean / P95 / P99 | 13.322 / 18.030 / 21.915 ms | 14.127 / 18.051 / 22.020 ms |
| GPU 利用率 mean | 37.0% | 34.9% |
| GPU 显存 mean / max | 10547.0 / 10802 MiB | 12839.8 / 13008 MiB |
| GPU 功耗 mean / max | 117.96 / 122.32 W | 116.53 / 125.61 W |

导航 Physics 出现过一个 348.885 ms 最大值，因此表中同时保留 P95/P99，避免均值隐藏
长尾。建图临时 MCAP 共记录 7001 条消息，八路图像分别为 221 或 222 帧，验证通过后
已删除；导航性能驱动接受 1 个真实目标、失败 0 个，并在观测内实际移动 1.724 m。
性能观测只证明完整工作负载和测量契约，不代替导航效果验收。

本机报告摘要哈希：

```text
20b227e5e075bb592d4e48613a43abc2527961b30cfcdfb08ba95d92d99a9902  summary.json
a3ada98553f63a9467795a146df26e7e74cb9811348cb085c3dac58811527d71  mapping_8cam/performance.json
ebb25d9a5ae634a48f7590916ea6ee41d6299bebcc050aced57ff4e045ad2dd4  navigation_6cam/performance.json
```

## 手动流程结论

手动入口已经配置为 Isaac Sim GUI + RViz：建图使用 W/S/A/D/Space/Q 并在退出时保存
地图；导航等待 cuVGL 自动定位和 Nav2 active 后，才允许 RViz `2D Goal Pose` 转发到
`/navigate_to_pose`。不需要用 `2D Pose Estimate` 手工标定出生点。cuVGL 免除的是已知
场景中的初始位姿输入；机器人仍需位于地图覆盖、视觉可识别的区域。

完整人工操作步骤见[手动键盘建图、保存与 RViz 导航全流程](manual_mapping_navigation.md)。
按照任务范围，自动化仅验证了工作流、话题、定位、生命周期和目标接口正常，没有代替用户
评价手动导航的具体路径效果。

## 解释边界

[Isaac Sim Benchmarks](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/benchmarks.html)
中的 Summary Report 是输出格式示例，不是 RTX 4090 的统一通过线。本项目只报告同一硬件、
同一渲染和传感器契约下的本机实际值。配置依据与历史 A/B 见
[RTX 4090 性能优化与实测](performance_optimization.md)。
