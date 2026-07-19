# 重要文件索引

## 公共配置

| 文件 | 作用 |
|---|---|
| `config/assets.yaml` | 酷家乐、Jackal、Hawk 的固定路径、default prim 与 SHA-256 |
| `config/environment.env` | Isaac Sim、ROS 2、Isaac ROS 与 DDS 环境 |
| `config/simulation.yaml` | 单次 stage、固定出生点、60 Hz physics/update 与 RTX 4090 RT2 配置 |
| `config/control.yaml` | 有效轮距、四轮关节、速度限制、PhysX、idle brake、motion assist |
| `config/sensors.yaml` | 四组 Hawk prim、8 路图像话题、分辨率、频率、外参和 0.40 m depth 自车裁剪 |
| `config/acceptance.yaml` | 静态 20 次 / 95% 口径、候选目标、路线验证和自适应性能策略 |
| `.gitattributes` | 文本属性说明；运行时地图不再进入 Git/LFS |
| `.gitignore` | 所有运行时地图、raw bag、中间产物、模型 cache、日志、run 和报告排除规则 |

## Isaac Sim

| 文件 | 作用 |
|---|---|
| `isaac_sim/navigation_sim.py` | 唯一正式 Standalone 入口、camera profile、静态范围、报告和性能采样 |
| `isaac_sim/jackal_sim/stage.py` | 打开酷家乐一次，在 session layer 组合 Jackal 与四组 Hawk |
| `isaac_sim/jackal_sim/graphs.py` | Clock、四轮差速控制、JointState 和 GroundTruth OmniGraph |
| `isaac_sim/jackal_sim/sensors.py` | 8/6 路 RGB、front depth/IMU 图与后向资源硬关闭 |
| `isaac_sim/jackal_sim/articulation_runtime.py` | Jackal articulation 运行时稳定性与速度接口 |
| `isaac_sim/jackal_sim/idle_brake.py` | 命令超时与静止 creep 制动 |
| `isaac_sim/jackal_sim/skid_steer_motion_assist.py` | 有边界的 skid-steer 曲率响应修正 |
| `isaac_sim/jackal_sim/contact_monitor.py` | 所有 Jackal rigid body 的 PhysX 接触统计与接触样本证据 |
| `isaac_sim/jackal_sim/contact_classification.py` | 区分零冲量正间隙 proximity 与低位竖直轮地支撑接触 |
| `isaac_sim/jackal_sim/performance.py` | Isaac 官方 recorder + 自适应墙钟采样 |

`dynamic_obstacles.py`、`dynamic_motion.py` 与旧 phase 脚本只为历史回溯保留；`navigation_sim.py` 在本分支拒绝 dynamic profile。

## ROS 2 包

| 路径 | 作用 |
|---|---|
| `ros2_ws/src/jackal_control/` | Command Guard、wheel odometry 与控制 launch |
| `ros2_ws/src/jackal_bringup/` | 传感器转换、cuVSLAM、cuVGL、nvblox、Nav2、TF、RViz |
| `ros2_ws/src/jackal_experiments/` | 自动闭环建图、地图保存、定位 bootstrap、导航 runner 与证据采集 |
| `ros2_ws/src/jackal_teleop/` | 带 0.18 秒 deadman 的 W/S/A/D/Space/Q 建图控制 |

关键 bringup 文件：

| 文件 | 作用 |
|---|---|
| `config/visual_slam_mapping_8cam.yaml` | 8/8 图像参与建图 |
| `config/visual_slam_navigation_6cam.yaml` | 只声明 front/left/right 的 6 个运行时输入，避免等待已关闭后向相机 |
| `config/vgl_mapping_8cam.yaml` | 8 相机 cuVGL 配置 |
| `config/vgl_navigation_6cam.yaml` | front/left/right 共 6 相机 cuVGL 配置 |
| `config/mapping_topics_8cam.yaml` | 离线建图固定话题顺序 |
| `config/nvblox.yaml` | front native depth 的 map-frame static TSDF/ESDF |
| `config/nav2.yaml` | forward-only Smac/MPPI、costmaps、Collision Monitor 与 smoother |
| `behavior_trees/navigate_forward_only.xml` | 无倒车恢复的 Nav2 行为树 |
| `launch/phase8.launch.py` | 强制 `navigation_6cam` 的完整导航入口 |
| `rviz/mapping.rviz` | 手动建图的轨迹、机器人、TF、mesh/ESDF 视图 |
| `rviz/navigation.rviz` | map/costmap/路径/安全区与 `/goal_pose` 工具 |

## 用户脚本

| 脚本 | 用途 |
|---|---|
| `scripts/build.sh` | 环境、资产、语法与 ROS build 检查 |
| `scripts/run_sim.sh` | 单独启动 GUI/headless simulator，可选 camera profile |
| `scripts/run_mapping.sh` | 自动闭环或人工四 Hawk / 8 路图像建图，完整成功后清理 raw/intermediate |
| `scripts/run_manual_mapping.sh` | Isaac Sim GUI + 建图 RViz + 键盘控制的一键入口 |
| `scripts/create_vgl_map.sh` | 用在线 cuVSLAM 优化 TUM 和临时 MCAP 生成公共优化帧，再从其分支生成同坐标系 cuVGL map |
| `scripts/run_navigation.sh` | 检查 manifest 并启动 6 路 ROS 导航 |
| `scripts/run_all.sh` | 启动 simulator + ROS 导航 + 自动路线 runner |
| `scripts/run_manual_navigation.sh` | GUI + RViz + cuVGL 自动定位 + 2D Goal Pose 一键入口 |
| `scripts/check_manual_navigation.sh` | 不发目标的 late-join DDS/定位/Nav2/相机只读检查 |
| `scripts/run_static_trial.sh` | 一次静态有效/无效实验与最终分类 |
| `scripts/run_static_acceptance.sh` | 至少 20 次、95% 的静态批次 |
| `scripts/run_performance_benchmark.sh` | 真实 8 路/6 路负载的自适应性能观测 |

自动建图折线与控制门限在 `config/mapping_coverage.yaml`；运行时驱动为
`jackal_experiments/mapping_coverage_driver.py`；在线数据库与闭环/平面质量门禁为
`jackal_experiments/visual_map_saver.py`，拓扑/碰撞门禁为 `tools/validate_mapping_run.py`。

性能配置依据、A/B 数据和回退实验见 `docs/performance_optimization.md`；2026-07-19 的
静态 20 轮与最终 GUI 8/6 路性能证据摘要见 `docs/kujiale_jackal_validation.md`。

## 地图、验收与性能工具

| 文件 | 作用 |
|---|---|
| `tools/write_map_manifest.py` | 检查 8 路 bag 和运行时 artifact，写 manifest |
| `tools/check_map_manifest.py` | 检查资产/profile/hash/文件组并拒绝 raw capture 泄漏 |
| `tools/check_visual_map_stage.py` | 校验公共 cuVSLAM 八路优化帧及其 cuVGL 子集的位姿一致性 |
| `tools/prepare_native_depth_fusion.py` | 从 MCAP 对齐公共 cuVSLAM 优化帧与 front 原生深度 |
| `tools/write_optimized_frames_report.py` | 校验 8 路公共优化帧完整性并冻结 cuVSLAM/MCAP 位姿溯源哈希 |
| `tools/run_offline_nvblox_fusion.py` | 调用官方 fuser 生成 TSDF mesh 与静态 occupancy |
| `tools/validate_acceptance_routes.py` | occupancy known-free、膨胀、连通与绕行验证 |
| `tools/create_static_trial_metadata.py` | 创建不可变 trial 身份与目标元数据 |
| `tools/finalize_static_trial.py` | 合并导航/仿真/碰撞/命令并分类单轮 |
| `tools/summarize_static_avoidance.py` | 计算无碰撞通行次数 / 有效实验次数 |
| `tools/record_performance_metrics.py` | 项目进程树与 RTX telemetry CSV |
| `tools/summarize_performance.py` | 规范化一份官方 + 整机性能报告 |
| `tools/compare_performance_profiles.py` | 生成 mapping 8 路与 navigation 6 路并列报告，无 KPI gate |

## 生成数据

| 路径 | Git 策略 |
|---|---|
| `data/maps/kujiale_latest_20260719_160004/` | 当前本机保留地图，忽略且不复制到 Git/LFS |
| `data/maps/<其他地图>/` | 忽略；按空间策略清理旧地图 |
| `data/bags/kujiale_latest_20260719_160004_20260719T080027Z/` | 当前本机保留 MCAP，忽略且不复制到 Git/LFS |
| `data/bags/<其他采集>/` | raw MCAP/DB3 与离线临时工作区，忽略 |
| `data/models/vgl/` | TensorRT cache，忽略 |
| `data/logs/`、`data/runs/`、`data/reports/` | 运行证据，默认忽略 |

历史 `docs/phase*_validation.md` 描述旧 Warehouse/Nova Carter 分支的既有结果，不是当前酷家乐静态验收证据。
