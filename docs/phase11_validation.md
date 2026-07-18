# 阶段11正式验收与验证记录

## 1. 固定范围

阶段11冻结为前向 Hawk 双目、前向 IMU、Isaac Sim 原生前向深度、cuVSLAM、前向 cuVGL、dynamic nvblox 和 Nav2。所有 lidar 关闭，侧向和后向相机 OmniGraph 不创建。按用户本轮决定，光照和颜色随机化均关闭，也不把它们计入最终通过项。

导航输入不包含 ground truth。`/ground_truth/odometry`和PhysX contact只由试验运行器离线计算路径与碰撞指标；第三人称相机只供观察。

## 2. 为长距离和低延迟冻结的关键链路

```text
前向双目 + IMU -> cuVSLAM健康/连续跟踪
前向双目 -> cuVGL地图锚/失锁恢复
前向原生深度 -> dynamic nvblox -> combined ESDF/map slice
前向原生深度 -> LaserScan(真实光学传感器帧) -> Nav2 ObstacleLayer/Collision Monitor

MapServer + SmacPlanner2D -> MPPI DiffDrive @ 20 Hz
  -> Velocity Smoother -> Collision Monitor -> Command Guard
  -> /cmd_vel_sim -> 左右两主动轮差速OmniGraph
```

长距离运行中，局部 ObstacleLayer 必须订阅保留真实传感器帧和sensor origin的 `/front_depth/scan`。不能把已变换到 `odom` 的 PointCloud2当作传感器输入：PointCloud2没有独立sensor-origin字段，机器人远离odom原点后Nav2会把 `(0,0)`误当raytrace起点并超出滚动窗口。`/front_depth/points_odom`仍可用于RViz和诊断，但不进入ObstacleLayer。

nvblox动态三维层使用8 m滚动清图半径，并以1 Hz清理范围外块。全局静态结构由MapServer occupancy保留；这样机器人跨越十余米时不会无限增长GPU哈希层，实测长距离combined slice恢复到7 Hz以上。

## 3. USD理论最优路径

`prepare_stage11_reference.sh`执行以下可复现流程：

1. 用固定Isaac Sim 6.0.1 Python打开实测完整路径的官方Warehouse USD；
2. 遍历实际`UsdPhysics.CollisionAPI`，在机器人0.05–1.20 m垂直带内提取世界AABB；
3. 按5 cm分辨率栅格化430个有效碰撞体；
4. 用Nova Carter不对称footprint `[[0.14,0.25],[0.14,-0.25],[-0.607,-0.25],[-0.607,0.25]]`和0.03 m padding逐航向膨胀；
5. 执行8航向SE(2) A*，允许原地转向和前/后向差速运动；
6. 为六个固定目标保存理论最短路径长度和路径点。

本机生成的理论长度：

| 目标 | 理论最优长度 |
|---|---:|
| `east_lane` | 2.000 m |
| `north_east` | 2.621 m |
| `north_west` | 1.707 m |
| `far_north` | 12.269 m |
| `far_north_east` | 9.548 m |
| `far_mid_aisle` | 11.597 m |

算法使用实际USD碰撞几何而不是Nav2自己规划出的路径，因此不会用被测规划器给自己定义“理论最优”。动态actor引起的绕行只增加实际路径，不更改静态理论基线。

## 4. 场景矩阵

| 类别 | 正式次数 | 内容 | 门槛 |
|---|---:|---|---:|
| static | 10 | 官方Warehouse全部复杂静态碰撞体，无项目动态actor | ≥95% |
| dynamic | 10 | 官方叉车、box、capsule共3种移动障碍 | ≥90% |
| heterogeneous | 10 | 3个基础actor + 3个远距离box/capsule，共6个异构动态障碍 | ≥90% |

每类按序轮换六个目标，后3个目标跨越多个仓库结构带并要求实际轨迹至少7 m。正式seed集合固定为：static 21000–21009、dynamic 31000–31009、heterogeneous 41000–41009。汇总器验证完整身份集合，重复报告、错seed、错目标或错class不能补数。

正式“试验”要求ROS runner至少实际提交过一个目标并产生`navigation.json/goals`。仿真ready后、目标runner启动前的进程/采集基础设施空轮会保留原始目录并自动重试，不能消耗允许的导航失败名额；一旦目标已提交，所有失败都必须进入该类别分母。

移动actor保留视觉和PhysX碰撞几何。它们可以在机器人进入安全距离时退往预先验证的free-space refuge，但移动距离、yield次数、最终位置和接触都进入报告；不能通过关闭碰撞来制造“避障成功”。异构组必须实际存在叉车、box、capsule三种kind且至少6个actor全部移动。

## 5. 单轮通过条件

单轮只有以下全部为真才判为`passed`：

- Nav2在超时内成功，终点位置误差≤0.25 m、航向误差≤10°；
- PhysX碰撞事件为0，出生点和终点无非预期overlap；
- simulator、ROS runner、仿真时间单调、官方资产未修改；
- 实际运行图只有FrontStereo、FrontDepth、FrontImu，lidar/侧后相机关闭；
- static/dynamic/heterogeneous actor数量、kind与运动有效；
- 长距离轮实际路径≥7 m；
- 实时因子≥0.70；
- `/cmd_vel_nav_raw`≥18 Hz，视觉状态≥7 Hz，深度scan≥3.5 Hz，nvblox slice≥4.5 Hz；
- 原始Nav2命令到最终仿真命令的P95新鲜度≤100 ms；
- 前向深度P95数据年龄≤250 ms；
- 定位未ready的宽限期后最终命令绝对值≤0.02，不允许盲目运动；
- 线/角加速度P95≤1.60 m/s²和3.00 rad/s²；
- 线/角jerk P95≤5.50 m/s³和11.00 rad/s³；
- 自动运行器发送目标，`manual_intervention=false`；要求记录时MCAP非空。

成功轨迹伸长率定义为：

```text
max(0, actual_ground_truth_path / USD_SE2_Astar_optimal_path - 1)
```

最终统计成功轮的P95，要求≤20%。ground truth只在导航结束后进入分母/分子统计，不发布主TF、不进入任何控制节点。

## 6. 代表性真实运行

最终代码在同一台RTX 4090工作站已完成三类代表性运行。动态短距离轮和异构长距离轮均全项通过；异构长距离关键结果为：

- 目标`far_mid_aisle`，实际路径11.676 m；
- 位置/航向误差0.047 m / 7.94°；
- USD理论路径伸长率0.68%；
- 6个异构actor全部运动，最小运动4.515 m，发生1次安全yield；
- PhysX碰撞0；
- 实时因子0.837；
- command raw-to-sim新鲜度P95 54.68 ms；
- 前向深度年龄P95 41.67 ms；
- controller / visual / depth / nvblox为20.00 / 8.07 / 4.47 / 7.18 Hz；
- 线/角加速度P95 0.098 / 0.389，线/角jerk P95 1.800 / 6.060；
- 峰值显存11.25 GiB；
- 23.9 MB压缩MCAP有效。

静态长距离最终复测和30轮正式矩阵的结果在完成实际执行后写入本节，不以单元测试替代。

## 7. 执行命令

构建、离线测试和三类代表性smoke：

```bash
./scripts/run_stage11_tests.sh warehouse_v2_front
```

单轮复现：

```bash
./scripts/run_stage11_trial.sh \
  --class heterogeneous --seed 41000 --goal-index 5 \
  --headless --no-rviz --record-bag
```

正式10/10/10矩阵：

```bash
./scripts/run_acceptance.sh \
  --matrix-id phase11-final-v1-20260718 --record-bag
```

中断恢复：

```bash
./scripts/run_acceptance.sh \
  --matrix-id phase11-final-v1-20260718 \
  --resume --skip-build --record-bag
```

权威结果为`data/reports/phase11/acceptance/<matrix-id>/summary.json`，人类可读结果为同目录`report.md`。运行数据默认不提交Git；阶段文档记录最终摘要和matrix ID，完整原始证据留在本机。

## 8. 明确未覆盖

- 不做不同光照、色温、材质颜色随机化测试；
- 不宣称前向双目对所有现实世界外观域变化具有验收结论；
- 不启用四向VGL或lidar；
- 不把仿真通过等同于真实机器人安全认证。

这些边界不影响本阶段对官方Warehouse内复杂静态、动态、异构动态和长距离纯视觉导航的指标验证。
