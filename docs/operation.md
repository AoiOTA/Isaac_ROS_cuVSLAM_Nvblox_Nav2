# 运行命令速查

本页只列当前酷家乐 + Jackal 静态流程。

## 构建

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/build.sh
```

## Simulator 烟测

```bash
./scripts/run_sim.sh --headless --duration 10 --camera-profile navigation_6cam
./scripts/run_sim.sh --headless --duration 10 --camera-profile mapping_8cam
```

## 八路手动建图

```bash
./scripts/run_manual_mapping.sh --map kujiale_manual_20260719
python3 tools/check_map_manifest.py data/maps/kujiale_manual_20260719
python3 tools/validate_acceptance_routes.py \
  data/maps/kujiale_manual_20260719 --config config/acceptance.yaml
```

## 六路导航

```bash
./scripts/run_all.sh --map kujiale_latest_20260719_160004 --headless --no-rviz
./scripts/run_all.sh --map kujiale_latest_20260719_160004 --gui --rviz
./scripts/run_manual_navigation.sh --map kujiale_latest_20260719_160004
```

推荐始终使用上面的手动入口；它会统一管理 GUI、RViz、自动定位门禁和本地 DDS discovery
server。运行期间可在另一个终端做只读检查：

```bash
./scripts/check_manual_navigation.sh
```

## 静态避障统计

```bash
./scripts/run_static_trial.sh \
  --map kujiale_latest_20260719_160004 --goal-index 0 --attempt-index 1 --headless

./scripts/run_static_acceptance.sh \
  --map kujiale_latest_20260719_160004 --headless
```

## 自适应性能观测

```bash
./scripts/run_performance_benchmark.sh \
  --profile all --map kujiale_latest_20260719_160004 --gui
```

这会保留 1280×720 第三人称跟随视口，但不会在 GUI 中预览八路相机。无人观察时可把
`--gui` 换成 `--headless`；报告会记录模式，不能把两种工况合并比较。

性能流程不是空闲栈采样。`mapping_8cam` 在安全原地交替旋转时运行完整
cuVSLAM、nvblox，并把与正式建图相同的 8 路图像、CameraInfo、IMU、TF 和
clock 写入临时 MCAP；采样完成后验证 8 路消息均非零并删除 MCAP。
`navigation_6cam` 会循环执行 `config/acceptance.yaml` 中已经通过占据图验证的
真实 Nav2 目标，只有同时观察到 `/cmd_vel_sim` 非零和 ground truth 物理位姿变化
后才开始自适应预热。两种负载的活动证据分别写入各 profile 目录的
`workload.json`。

2026-07-19 本机 GUI 最终样本：建图 8 路为 `22.462 FPS / 0.374 RTF`，导航
6 路为 `24.290 FPS / 0.405 RTF`。完整统计与静态 20 轮验收见
[酷家乐 Jackal 验证记录](kujiale_jackal_validation.md)。

## 自动测试

```bash
python3 -m pytest -q
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test --return-code-on-test-failure
colcon test-result --verbose
```

正常停止使用一次 `Ctrl-C`。脚本只管理自己的进程组；不要使用 `killall` 或全局 `pkill`。动态 profile、旧 Warehouse 地图和旧 phase9–phase11 验收不属于当前支持范围。
