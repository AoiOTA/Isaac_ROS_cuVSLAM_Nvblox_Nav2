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
./scripts/run_mapping.sh --map kujiale_jackal_8cam --interactive
python3 tools/check_map_manifest.py data/maps/kujiale_jackal_8cam
python3 tools/validate_acceptance_routes.py \
  data/maps/kujiale_jackal_8cam --config config/acceptance.yaml
```

## 六路导航

```bash
./scripts/run_all.sh --map kujiale_jackal_8cam --headless --no-rviz
./scripts/run_all.sh --map kujiale_jackal_8cam --gui --rviz
```

分离启动：

```bash
# terminal A
./scripts/run_sim.sh --headless --camera-profile navigation_6cam

# terminal B
./scripts/run_navigation.sh --map kujiale_jackal_8cam --rviz
```

## 静态避障统计

```bash
./scripts/run_static_trial.sh \
  --map kujiale_jackal_8cam --goal-index 0 --attempt-index 1 --headless

./scripts/run_static_acceptance.sh \
  --map kujiale_jackal_8cam --headless
```

## 自适应性能观测

```bash
./scripts/run_performance_benchmark.sh \
  --profile all --map kujiale_jackal_8cam --headless
```

性能流程不是空闲栈采样。`mapping_8cam` 在安全原地交替旋转时运行完整
cuVSLAM、nvblox，并把与正式建图相同的 8 路图像、CameraInfo、IMU、TF 和
clock 写入临时 MCAP；采样完成后验证 8 路消息均非零并删除 MCAP。
`navigation_6cam` 会循环执行 `config/acceptance.yaml` 中已经通过占据图验证的
真实 Nav2 目标，只有同时观察到 `/cmd_vel_sim` 非零和 ground truth 物理位姿变化
后才开始自适应预热。两种负载的活动证据分别写入各 profile 目录的
`workload.json`。

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
