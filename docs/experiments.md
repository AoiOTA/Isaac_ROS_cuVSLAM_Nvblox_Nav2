# Experiments

阶段10实验由脚本端到端执行，不要求在Isaac Sim、RViz或终端中手动发送目标。当前实验范围固定为前向双目，并关闭光照与颜色随机化。

## 单轮实验

```bash
./scripts/run_phase10_trial.sh --class static --seed 1000 --goal-index 0 --record-bag
./scripts/run_phase10_trial.sh --class dynamic --seed 11000 --goal-index 0 --record-bag
```

可选参数包括`--map`、`--run-id`、`--run-dir`、`--headless`、`--gui`、`--rviz`、`--no-rviz`、`--record-bag`和`--no-bag`。`--goal-index`为0、1或2；`--all-goals`用于硬化轮。每个入口自动推导仓库根目录、source ROS/overlay、创建隔离DDS、启动完整栈、等待生命周期守卫确认最终Nav2进程全部active、执行目标、生成结果并安全清理。矩阵、全局单轮和run目录各有独占锁；相同run目录不会覆盖旧bag或报告。

每轮目录`data/runs/<run_id>`至少包含`scenario.yaml`、`trajectory.csv`、`command_trace.csv`、`contacts.csv`、`gpu.csv`、`navigation.json`、`simulator.json`和`result.json`。启用记录时还包含zstd-fast MCAP。`result.json`是该轮权威判定，原始日志用于定位失败原因。

## 故障硬化

```bash
./scripts/run_phase10_hardening.sh warehouse_v2_front
```

该入口在实际动态导航中注入重定位、depth stale和map-slice stale，不是mock节点。通过要求三目标完成、每类Guard状态出现、宽限后最终Twist为零、恢复成功、无PhysX碰撞且MCAP有效。

## 预验收矩阵

```bash
./scripts/run_phase10_preacceptance.sh --map warehouse_v2_front
```

默认从`config/stage10.yaml`读取20次静态、20次动态和seed base 1000。三目标按试验序号轮换。汇总位于：

```text
data/reports/phase10/preacceptance/<matrix-id>/summary.json
data/reports/phase10/preacceptance/<matrix-id>/trials.csv
data/reports/phase10/preacceptance-summary-latest.json
data/reports/phase10/preacceptance-trials-latest.csv
```

调参只影响动态组时可以复用一套已通过且数量完全匹配的静态结果：

```bash
./scripts/run_phase10_preacceptance.sh \
  --static-trials 20 --dynamic-trials 20 --seed-base 1000 \
  --matrix-id dynamic-retune-v2 \
  --reuse-static-matrix previous-full-matrix-id \
  --record-bag --skip-build
```

脚本会逐个检查被复用静态`result.json`的数量和passed状态，再与新动态20次一起计算最终门槛，不会用缺失或失败报告补数。`--reuse-passed-dynamic-matrix id1,id2`可仅复用相同序号且已通过的动态轮；缺失和失败轮仍会按原seed重跑。最终验证矩阵`phase10-final-v9-20260718`为静态20/20、动态20/20、0碰撞，全部成功路径伸长率P95为13.58%。

## 结果解释

- `navigation_passed=false`：先看目标状态、终点误差、progress timeout和四级命令。
- `no_physx_collision=false`：以`contacts.csv`和simulator contact pairs为准，不能靠关闭接触检查通过。
- `scenario_class_valid=false`：动态actor没有全部创建/运动，或静态轮错误启用了动态profile。
- `realtime_factor=false`：检查GPU记录、外部Isaac Sim负载和ROS日志中的控制循环miss。
- `compact_mcap_recorded=false`：检查本轮Fast DDS super-client XML、discovery日志和`rosbag.log`。
- `path.stretch`超限：同时检查参考global plan、actual ground-truth轨迹、MPPI绕行和动态障碍yield记录。

完整口径、冻结参数和实测证据见[phase10_validation.md](phase10_validation.md)。运行产物默认被Git忽略，不应提交大体积bag、地图或日志。
