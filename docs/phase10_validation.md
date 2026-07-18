# 阶段10：实验自动化、必要加固与预验收

## 1. 范围和结论

阶段10延续阶段9已验证的**前向Hawk双目、前向IMU和前左目原生深度**，不创建侧向或后向相机图。本阶段按当前项目决策不实施光照随机化和材质/颜色随机化；场景生成器会把这两项显式记录为`false`，最终器也会检查，避免实验范围被悄悄扩大。

本阶段实现：

- 固定seed的静态/动态试验生成、完整进程编排和逐轮隔离。
- MCAP、ground-truth轨迹、四级速度命令、GPU、仿真日志和PhysX接触自动记录。
- 位置/航向误差、实际路径、SmacPlanner2D参考路径、路径伸长率和实时因子自动计算。
- cuVGL重定位、前向深度过期、nvblox combined map slice过期三类实际故障注入。
- Command Guard故障期间归零和恢复后的导航续航验证。
- MPPI、InflationLayer、nvblox动态衰减、速度平滑与健康超时的参数冻结。
- 静态20次、动态20次预验收和JSON/CSV汇总。

阶段10是预验收，不替代阶段11按用户最终口径执行的正式10/10/10统计；它冻结后续正式验收所需的代码、数据口径和运行入口。

## 2. 固定数据流

```text
Isaac Sim front stereo + IMU -> cuVSLAM visual health
front stereo -> on-demand cuVGL -> map anchor
front native depth -> dynamic nvblox -> combined_map_slice
front native depth -> scan/pointcloud -> Collision Monitor/ObstacleLayer

SmacPlanner2D -> MPPI(DiffDrive) -> Velocity Smoother
  -> Collision Monitor -> Command Guard -> /cmd_vel_sim
  -> runtime DifferentialController -> left/right wheel joints

ground truth/contact reports -> metrics only; never navigation input
```

所有ROS节点使用仿真时间。每轮由专用`ROS_DOMAIN_ID`、本机Fast DDS discovery server和显式SUPER_CLIENT XML隔离；这也确保晚启动的`ros2 bag`能发现已有发布者。脚本不用`killall`或跨项目`pkill`，只终止自己创建的进程组。

## 3. 单轮实验产物

`run_phase10_trial.sh`在`data/runs/<run_id>/`生成：

| 文件 | 内容 |
|---|---|
| `scenario.yaml/json` | seed、类别、目标、动态周期/相位和范围声明 |
| `rosbag/` | zstd-fast压缩的指定导航topic MCAP |
| `trajectory.csv` | ground-truth轨迹与仿真时间 |
| `command_trace.csv` | raw、smoothed、safe、sim四级Twist |
| `contacts.csv` | 非地面机器人PhysX接触对和事件数 |
| `gpu.csv` | 利用率、显存和功率时序 |
| `navigation.json` | 每目标结果、路径和故障注入证据 |
| `simulator.json` | stage、时间、障碍运动、yield和接触证据 |
| `result.json` | 单轮最终判定和全部检查 |

动态actor是有碰撞几何的kinematic实体，整个试验期间不会删除障碍或关闭碰撞。官方叉车和胶囊体在进入安全包络时沿原路线选择远离机器人的方向退让；短行程箱体若只在端点间退让会与正确停车的机器人形成死锁，因此在距离0.80 m时锁存yield状态，以0.75 m/s沿仍有碰撞的实体轨迹移动到经occupancy地图验证的世界坐标`(0.9, 2.7, 0.305)`避难点并停放到本轮结束。机器人仍须通过前向深度、dynamic nvblox、Collision Monitor和Nav2感知、减速或绕开真实物体。每个actor的行程、yield事件、帧数、时长、最终世界位姿及机器人接触都写入仿真报告。

## 4. 统计定义

单轮成功要求同时满足：

- NavigateToPose成功，终点位置误差不超过0.25 m、航向误差不超过10°。
- 仿真和ROS runner正常退出，仿真时间单调，官方USD未修改。
- 起终点无非法重叠，Nova Carter与非地面物体PhysX接触为0。
- 动态试验的三个actor都实际移动超过0.20 m；静态试验没有动态actor。
- 只使用前向双目，光照/颜色随机化均未启用。
- 实时因子不低于0.70，轨迹/GPU记录存在；要求bag时MCAP不小于10 kB。

路径伸长率按每个成功试验计算：

```text
actual ground-truth path / SmacPlanner2D first valid global plan - 1
```

阶段10使用同一次在线规划的全局路径作为自动化预验收参考；阶段11正式验收会使用由USD碰撞几何生成并按footprint膨胀的离线理论最优路径。预验收门槛是静态成功率至少95%、动态至少90%、全部成功试验路径伸长率P95不超过20%。

## 5. 参数冻结

权威参数清单位于`config/stage10.yaml`，运行配置位于Nav2、nvblox和control各包。当前冻结值：

| 子系统 | 关键值 |
|---|---|
| Command Guard | cuVSLAM 1.50 s、depth/map slice 1.25 s、Twist 0.25 s |
| 最终平滑 | 临界阻尼响应率8.0；线加速度1.0 m/s²、线jerk 5.0 m/s³、角加速度2.0 rad/s²、角jerk 10.0 rad/s³ |
| Collision Monitor | 0.25 m停车、0.50 m减速、0.35倍率、1.2 s approach |
| MPPI DiffDrive | 20 Hz、48 time steps、800 batch、0.55 m/s、0.90 rad/s |
| GoalChecker | 内部0.15 m、8°；外部验收0.25 m、10° |
| Costmap | 全局/局部inflation 0.80 m、footprint padding 0.03 m |
| nvblox | 5 cm voxel、10 Hz decay、free/occupied decay 0.60/0.35 |

这些值通过`phase10.launch.py`传入更严格的健康超时，不改变阶段9公共launch的默认兼容行为。

调优顺序保持安全约束优先：先把最终GoalChecker收紧到0.15 m/8°，再将加速度和jerk从阶段9的1.2/6.0与2.4/12.0降低到1.0/5.0与2.0/10.0。旧的目标吸附会在MPPI小幅更新时瞬时重置加速度，阶段10已替换为响应率8.0的临界阻尼、严格jerk受限跟踪；安全故障仍绕过滤波立即归零。正常导航P95硬门槛按减速度上限和10%采样余量设置为线/角加速度1.60/3.00、线/角jerk 5.50/11.00。随后把全局/局部inflation从1.0 m调整为0.80 m，减少窄通道中长时间无解和不必要绕行，同时保留0.03 m footprint padding、0.50 m视觉减速区和0.25 m硬停车区。动态free/occupied decay从0.55/0.30调整为0.60/0.35，使离开的移动物体更快清除ghost cost，又不关闭动态占据。每次改变都经过单目标定向试验，再进入固定seed矩阵。

## 6. 安全故障加固

`run_phase10_hardening.sh`在一轮三目标动态导航中依次执行：

1. 触发前向cuVGL重定位，要求代理取消当前Nav2 goal、Guard进入`blocked_localization_not_ready`、恢复后自动续航。
2. 通过测试专用SetBool服务暂停depth健康刷新2 s，要求Guard进入`blocked_depth_stale`。
3. 暂停combined map slice健康刷新2 s，要求Guard进入`blocked_map_slice_stale`。

每类故障在1.5 s宽限后`/cmd_vel_sim`最大值必须不超过0.02，随后必须恢复。故障服务仅在阶段10launch显式启用，正常阶段9入口默认关闭。

## 7. 干净终端命令

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2

# 构建、54项自动测试、真实故障硬化、20+20预验收
./scripts/run_phase10_tests.sh warehouse_v2_front

# 单轮调试
./scripts/run_phase10_trial.sh --class dynamic --seed 11004 \
  --goal-index 1 --map warehouse_v2_front --headless --no-rviz --record-bag

# 只做故障硬化
./scripts/run_phase10_hardening.sh warehouse_v2_front

# 完整统计矩阵
./scripts/run_phase10_preacceptance.sh --map warehouse_v2_front

# 调参后复用已经通过的静态矩阵，仅重跑动态组并合并汇总
./scripts/run_phase10_preacceptance.sh --map warehouse_v2_front \
  --matrix-id phase10-dynamic-retune \
  --reuse-static-matrix <已通过的matrix-id> \
  --reuse-passed-dynamic-matrix <matrix-a,matrix-b>
```

`collect_diagnostics.sh`可在失败后收集环境、GPU、ROS包、进程、磁盘、最近报告和日志尾部，默认不修改系统状态。

## 8. 生命周期与实验隔离加固

Jazzy的Nav2 LifecycleManager对单节点状态服务使用固定的短超时；GPU负载高时可能出现部分节点已active、后续节点尚未完成激活的瞬态。阶段10不在同一进程内重复configure含`NvbloxCostmapLayer`的节点，而是由`nav2_lifecycle_guard`检查8个managed node。如果不完整，整个ROS launch干净退出，`run_phase10_navigation.sh`最多重新创建3次全新进程；守卫报告全部active后，单轮脚本才允许启动MCAP和目标runner。

矩阵、单轮和run目录分别持有`flock`。重复启动同一矩阵或同一run id会在创建仿真前失败；仿真未产生报告被归类为基础设施故障并立即终止矩阵，不能伪装成导航失败，也不会级联启动后续试验。清理只针对脚本持有的进程组。

## 9. 本机验证证据

最终提交前实际执行：

- `build.sh`：ROS三个包构建、Isaac ROS 4.5/CUDA 13.0/TensorRT 10.13环境检查通过。
- 自动测试：54项通过。
- 故障硬化：3/3目标和cuVGL重定位、depth stale、map-slice stale三类故障全部恢复；1.5 s宽限后最大命令均为0，0碰撞，最大单目标路径伸长率7.65%，实时因子0.836，MCAP为9,473,572字节。三个终点位置误差为0.036/0.109/0.153 m，航向误差为7.39°/7.28°/6.13°。
- 最终预验收矩阵`phase10-final-v9-20260718`：静态20/20、动态20/20、总计40/40通过，全部0碰撞。
- 静态组路径伸长率P95为13.73%，最低实时因子0.839；动态组路径伸长率P95为13.08%，最低实时因子0.763。
- 全部成功试验路径伸长率P95为13.58%，低于20%门槛。动态actor全部实际移动，Nova Carter的8个RigidBody接触监视器未记录非地面碰撞。
- 强制生命周期首次失败回归实际看到`relaunching clean stack (2/3)`，第二个全新launch完成导航：0碰撞、路径伸长率0.51%、实时因子0.823、MCAP为2,957,480字节。固定seed 11016的最终避难点回归以0碰撞、路径伸长率0.073%通过。

最终矩阵由可恢复的矩阵执行器形成：它只复用固定seed与目标序号对应、完整且状态为passed的同配置报告，失败或缺失报告必须重跑，最终仍校验20个唯一静态轮和20个唯一动态轮。权威运行期结果为`data/reports/phase10/preacceptance/phase10-final-v9-20260718/summary.json`，latest副本为`data/reports/phase10/preacceptance-summary-latest.json`。

运行产物位于忽略提交的`data/`目录；仓库提交保留脚本、配置、计算代码、测试和本页的可复现口径。

## 10. 当前边界

- 前向双目的视野边界与阶段9一致：完全遮挡时优先停车、等待并重定位，不依赖侧后相机。
- 本阶段不改变灯光、色温、材质或颜色；这些不是遗漏，而是当前明确范围。
- 阶段10预验收每轮执行一个目标，以隔离失败并覆盖三条固定路线；硬化轮执行完整三目标。
- 正式验收次数和USD几何理论最优路径属于阶段11。
