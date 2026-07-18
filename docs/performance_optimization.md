# RTX 4090 性能优化与实测

本页记录 2026-07-18 至 2026-07-19 在本机 RTX 4090、Isaac Sim 6.0.1 上的 A/B 结果。`mapping_8cam`
这个历史 profile 名称表示四组 Hawk 双目产生的八路图像流，不表示八台物理相机。所有 GUI
对比均保持 1280×720 第三人称跟随视口，没有通过缩小预览窗口获得结果。

## 官方依据

- [Isaac Sim Performance Optimization Handbook](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/reference_material/sim_performance_optimization_handbook.html)：RT2、retrace、fractional cutout、DLSS、headless viewport、texture streaming、CPU governor 与线程建议。
- [Multi-Tick Rendering](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_multitick_rendering.html)：Camera prim 使用 `omni:sensor:tickRate`，ROS 2 Helper 自动跟随传感器 tick；跳过的 tick 不产生无用渲染。
- [Robot Simulation Tips](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_simulation/robot_simulation_tips.html)：物理步长、应用循环与渲染时钟必须分别理解和一致配置。
- [Isaac Sim Benchmarks](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/benchmarks.html)：官方 Nova Carter ROS 2 项目同样把四组 Hawk 记作八个 render products；示例输出不是本项目 KPI。

## 保留的优化

- 物理与应用目标频率均为 60 Hz，避免原 120 Hz 配置在当前循环中重复计算 PhysX。
- headless 模式设置 `disable_viewport_updates=True`，只跳过无人观看的编辑器视口；Hawk render products 不受影响。
- GUI 和 headless 都使用 RTX Real-Time 2.0，retrace 设为 0.1，关闭 fractional cutout，DLSS 固定为 Performance。
- RTX 4090 整机 telemetry 中，建图与导航负载平均约占 10.5 GiB 和 12.8 GiB / 24 GiB，
  因此关闭 texture streaming，以显存余量换吞吐。
- 四组 Hawk 的八个 Camera prim 均为 10 Hz Multi-Tick；导航不创建 back Hawk 的两个 render products。
- Jackal 自带 SICK LiDAR prim、可见支架和碰撞体在匿名 session layer 中关闭，也不创建 LiDAR publisher。
- Linux CPU governor 使用 `performance`。性能报告会记录实际 governor，脚本不会擅自修改系统设置。

## GUI 完整工作负载结果

建图工作负载包含四组 Hawk/八路图像、cuVSLAM、nvblox、ROS 2 发布、临时 MCAP、Jackal
实际运动和 1280×720 第三人称跟随视口；导航工作负载加载地图、cuVGL、cuVSLAM、nvblox
和 Nav2，执行真实目标并只渲染 front/left/right 三组 Hawk。采样按稳定后的墙钟时间进行，
没有固定 600 帧条件。

| 观测 | 路数 | Mean FPS | RTF | App mean ms | Physics mean ms | 说明 |
|---|---:|---:|---:|---:|---:|---|
| 初始 GUI | 8 | 22.026 | 0.367 | 45.403 | 13.628 | legacy RTX，CPU governor 为 powersave |
| RT2 调优、powersave | 8 | 22.769 | 0.379 | 43.918 | 13.678 | 八路完整、零 tracking error |
| RT2 调优、performance 最佳窗口 | 8 | 23.697 | 0.395 | 42.200 | 13.970 | 历史 20 秒稳定样本 |
| 历史最终 mapping | 8 | 22.732 | 0.379 | 43.992 | 13.186 | 历史 30 秒样本，八路各 187 帧 |
| **最终 `mapping_8cam`** | **8** | **22.462** | **0.374** | **44.524** | **13.322** | 35.014 秒稳定样本，临时 MCAP 八路均非零 |
| **最终 `navigation_6cam`** | **6** | **24.290** | **0.405** | **41.170** | **14.127** | 30.002 秒稳定样本，真实 Nav2 目标运动 |

最终建图样本的 App P95/P99 为 `66.110/71.370 ms`，Physics P95/P99 为
`18.030/21.915 ms`；最终导航样本分别为 `61.791/66.095 ms` 和
`18.051/22.020 ms`。导航 Physics 曾有一次 `348.885 ms` 最大值，因此不能只看均值；
P95/P99 和 30 秒总体 RTF 没有被这个单点替代。建图临时 MCAP 共 7001 条消息，八路图像
分别记录 221 或 222 帧；导航接受 1 个目标、失败 0 个，并在样本内实际运动 1.724 m。

最终结果都满足 `mode=gui`、`preview_resolution_reduced=false`、1280×720、lidar 关闭；
导航还满足后 Hawk render product 未创建。短窗口和不同批次间的差异说明 RTX 周期性
传感器负载存在自然波动，因此这里只报告实际观测，不设置虚假的固定 KPI。地图 manifest
和正式运行记录均已生成，详见 [酷家乐 Jackal 验证记录](kujiale_jackal_validation.md)。

## 已验证但未采用

- 960×540 预览没有收益，而且用户要求保持窗口尺寸，相关参数未保留。
- RTX Minimal 虽提高表面 FPS，但导致大量 cuVSLAM PnP/active-track 错误。
- Hydra Storm 无法成功创建 GUI renderer；其高 FPS 来自预览未正常渲染，结果无效。
- 手动周期性开关 viewport 会重建资源并降低 FPS。
- GPU Dynamics 在单 Jackal 场景降至 18.713 FPS / 0.312 RTF。
- 16 CPU worker、降低 solver iterations、RT2 max-bounces=2 均没有可重复收益。
- 将全部酷家乐 rigid bodies 静态化降低了 Physics frametime，但完整 GUI 总 FPS 下降，因此回退。

## 复测

确认 governor 后运行完整 GUI 双 profile 观测：

```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
./scripts/run_performance_benchmark.sh \
  --profile all --map kujiale_jackal_8cam --gui
```

报告中的 `rendering.preview_resolution_reduced` 必须为 `false`，并同时检查八/六路消息计数、tracking
日志、碰撞、App/Physics P95/P99 和 governor，不能只比较 Mean FPS。
