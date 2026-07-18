# 项目重要文件索引

本索引说明“文件负责什么、何时需要修改、是否为运行生成”。日常使用先看[用户操作手册](user_manual.md)，环境迁移先看[安装手册](installation.md)。

## 根目录与公共配置

| 文件 | 作用 | 何时修改 |
|---|---|---|
| `README.md` | 项目状态、架构概要、阶段入口和验证摘要 | 里程碑、公共命令或最终结果变化时 |
| `pyproject.toml` | Python工具的格式与测试基础配置 | 增加Python工具链规则时 |
| `.gitignore` | 排除构建、地图、bag、日志和大体积运行数据 | 增加新的生成目录时 |
| `config/environment.env` | ROS、Isaac Sim Python、资产根目录、DDS固定环境 | 换电脑或安装路径变化时 |
| `config/assets.yaml` | 两个官方USD的逻辑/实测完整路径与variant策略 | 资产版本或根目录变化后先重新验证 |
| `config/robot.yaml` | 轮半径、轮距、速度边界和机器人几何 | 更换机器人或实测轮参数时 |
| `config/sensors.yaml` | 前向双目、深度、IMU和话题约定 | 分辨率、频率或传感器prim变化时 |
| `config/scenarios.yaml` | 仿真出生点、动态actor和基础路线模板 | 新增可重复场景时 |
| `config/stage10.yaml` | 阶段10故障硬化与20+20预验收口径 | 回归阶段10时 |
| `config/stage11.yaml` | 六个正式目标、10/10/10、平滑/实时/时延门槛、异构障碍 | 阶段11范围或验收门槛变化时；修改后必须重跑受影响轮次 |
| `config/fastdds.xml` | 常规本地Fast DDS配置 | DDS接口或网卡策略变化时 |

## Isaac Sim Standalone

| 文件 | 作用 |
|---|---|
| `isaac_sim/navigation_sim.py` | 唯一正式Standalone入口；先创建SimulationApp，再组合场景、机器人、图、actor、监控与报告 |
| `isaac_sim/nova_carter_sim/stage.py` | 打开官方Warehouse、session layer引用Nova Carter、variant、出生点与关键prim验证 |
| `isaac_sim/nova_carter_sim/graphs.py` | 运行时创建Clock、差速控制、JointState和GroundTruth OmniGraph |
| `isaac_sim/nova_carter_sim/sensors.py` | 运行时创建前向双目、深度、CameraInfo、IMU图；正式范围不创建侧后图 |
| `isaac_sim/nova_carter_sim/dynamic_obstacles.py` | 在session layer创建box/capsule，绑定碰撞与视觉几何 |
| `isaac_sim/nova_carter_sim/dynamic_motion.py` | 叉车/box/capsule轨迹、yield、清隙不减的refuge撤离和恢复逻辑 |
| `isaac_sim/nova_carter_sim/contact_monitor.py` | PhysX接触采集与机器人碰撞计数 |
| `isaac_sim/nova_carter_sim/follow_camera.py` | GUI第三人称平滑跟随；不发布ROS数据 |
| `isaac_sim/nova_carter_sim/runtime.py` | timeline主循环、时间监控、报告和退出 |
| `isaac_sim/nova_carter_sim/process_lock.py` | 防止本项目重复启动Isaac Sim，不影响其他项目 |

官方USD不在仓库内，任何项目代码都不得保存或修改它们。`Nova_Carter_ROS.usd`只读参考且正式运行永不加载。

## ROS 2：控制包

| 文件 | 作用 |
|---|---|
| `nova_carter_control/command_guard.py` | 最终速度安全门：finite/平面化/限速/加速度/jerk/watchdog/定位与感知新鲜度 |
| `nova_carter_control/wheel_odometry.py` | 左右主动轮短时里程计，不发布主TF |
| `nova_carter_control/kinematics.py` | 两轮差速正逆运动学 |
| `nova_carter_control/config/control_params.yaml` | 轮参数与Command Guard冻结参数 |
| `nova_carter_control/launch/control.launch.py` | 控制节点启动与话题串接 |

路径前缀是 `ros2_ws/src/nova_carter_control/`。

## ROS 2：bringup包

| 文件 | 作用 |
|---|---|
| `bringup/launch/phase10.launch.py` | 阶段9–11正式ROS总图：传感器转换、定位恢复、dynamic nvblox、Nav2和RViz |
| `bringup/launch/nav2.launch.py` | Nav2、Velocity Smoother、Collision Monitor和生命周期管理 |
| `bringup/launch/nvblox_dynamic.launch.py` | dynamic nvblox与前向深度remap |
| `bringup/launch/depth_scan.launch.py` | 原生深度转换低延迟LaserScan和可视化PointCloud2 |
| `bringup/config/nav2.yaml` | SmacPlanner2D、MPPI DiffDrive、滚动局部代价地图、安全区与速度平滑 |
| `bringup/config/nvblox_dynamic.yaml` | dynamic TSDF/ESDF/combined slice和8 m滚动清图参数 |
| `bringup/config/visual_slam_phase9.yaml` | 前向双目+IMU cuVSLAM参数与健康接口 |
| `bringup/config/vgl.yaml` | 前向双目cuVGL配置 |
| `bringup/config/depth_to_scan.yaml` | 深度LaserScan范围、角度和帧配置 |
| `bringup/config/visual_wheel_ekf.yaml` | 平滑局部odom的轮速EKF；视觉健康仍是硬运动条件 |
| `bringup/nova_carter_bringup/navigation_tf_bridge.py` | VGL地图锚与局部odom组合，唯一发布 `map→odom` |
| `bringup/nova_carter_bringup/scan_timestamp_relay.py` | 深度时间戳/TF同步、scan与points派生 |
| `bringup/urdf/nova_carter.urdf.xacro` | 从USD提取并简化的base、相机、IMU、轮和脚轮TF |
| `bringup/rviz/navigation.rviz` | 完整导航可视化布局 |

表中 `bringup/` 是 `ros2_ws/src/nova_carter_bringup/` 的缩写。`*_4way.yaml`保留为早期实验参考，不属于当前前向双目正式运行路径。

## ROS 2：实验与恢复包

| 文件 | 作用 |
|---|---|
| `experiments/localization_bootstrap.py` | 启动时触发全局定位 |
| `experiments/localization_recovery_manager.py` | 失锁停车、前向cuVGL重定位、最多3次重试和恢复判定 |
| `experiments/vgl_pose_relay.py` | 创新门控并把VGL位姿送入cuVSLAM，不争抢TF |
| `experiments/resilient_navigation.py` | 定位失效时取消Nav2目标，恢复后自动重发同一目标 |
| `experiments/nav2_lifecycle_guard.py` | 确认8个Nav2 managed node全部active，失败则整栈干净重启 |
| `experiments/navigation_test_runner.py` | 自动发送目标，采集终点、频率、时延、数据年龄、平滑性、轨迹和命令证据 |
| `experiments/manual_goal_bridge.py` | 将RViz `/goal_pose`手动目标转发到可恢复导航action，新目标替换当前目标 |
| `experiments/occupancy_saver.py` | 自动保存occupancy地图 |
| `experiments/nvblox_map_saver.py` | 自动保存nvblox与PLY |

表中 `experiments/` 是 `ros2_ws/src/nova_carter_experiments/nova_carter_experiments/` 的缩写。

## 用户脚本

| 脚本 | 用途 |
|---|---|
| `scripts/build.sh` | 干净终端环境/资产检查、语法检查和colcon构建 |
| `scripts/bootstrap_baremetal.sh` | Isaac ROS裸机安装/预检；安装阶段可能需要sudo密码 |
| `scripts/run_phase9.sh` | 日常一键前向双目静态障碍导航，默认由RViz手动选点；`--auto`用于回归 |
| `scripts/run_mapping.sh` / `run_phase9_mapping.sh` | 自动采集与生成地图 |
| `scripts/run_phase10_hardening.sh` | 重定位、depth stale、map stale真实故障硬化 |
| `scripts/prepare_stage11_reference.sh` | 从实际USD生成理论路径参考 |
| `scripts/run_stage11_trial.sh` | 单次阶段11静态/动态/异构正式口径运行 |
| `scripts/run_stage11_smoke.sh` | 三类长距离代表性smoke |
| `scripts/run_stage11_tests.sh` | 构建、全部自动测试和代表性smoke |
| `scripts/run_acceptance.sh` | 静态/动态/异构各10次正式矩阵，支持resume |
| `scripts/collect_diagnostics.sh` | 只读收集系统、GPU、ROS、TF、频率与配置 |
| `scripts/lib/common.sh` | 所有脚本共用的根目录推导、source、检查与日志函数 |

阶段2–8的 `run_phase*_tests.sh` 是历史里程碑回归入口，仍可运行，但最终系统优先使用阶段9–11入口。

## 阶段11离线工具

| 文件 | 作用 |
|---|---|
| `tools/extract_usd_collision_geometry.py` | 用Isaac Sim USD运行时读取实际CollisionAPI和世界AABB，输出几何清单/栅格 |
| `tools/build_stage11_reference_paths.py` | 按带padding不对称footprint执行8航向SE(2) A*并保存六个理论最优路径 |
| `tools/generate_stage11_scenario.py` | 按class/seed/goal生成不可人工修改的单轮场景 |
| `tools/finalize_stage11_trial.py` | 合并ROS、仿真、GPU、MCAP和理论路径，执行单轮全项判定 |
| `tools/summarize_stage11_acceptance.py` | 验证30个身份集合并计算类别成功率、P95、长距离和资源指标 |
| `tools/record_gpu_metrics.py` | 按秒采样RTX利用率、显存和功耗 |
| `tools/write_fastdds_super_client.py` | 为晚加入的rosbag参与者生成本地discovery SUPER_CLIENT配置 |

## 测试与文档

- `ros2_ws/src/*/test/`：运动学、TF、cuVSLAM/VGL/nvblox/Nav2、阶段9–11配置契约。
- `tests/smoke/`：不启动完整GPU栈的离线工具和场景单元测试。
- `docs/phase2_validation.md` 至 `docs/phase11_validation.md`：各阶段设计、命令和实测证据。
- `docs/cuvslam_configuration.md`、`cuvgl_configuration.md`、`nvblox_configuration.md`：跨电脑配置与调参手册。
- `docs/architecture.md`：TF所有权与完整数据链。
- `docs/troubleshooting.md`：按症状定位故障。

## 生成数据（默认不提交）

| 目录 | 内容 |
|---|---|
| `data/maps/` | cuVSLAM、cuVGL、nvblox、mesh、occupancy地图 |
| `data/models/vgl/` | ALIKED/LightGlue TensorRT engine缓存 |
| `data/reference/` | 实际USD CollisionAPI栅格与理论路径 |
| `data/runs/` | 每轮场景、日志、bag、轨迹、命令、GPU和结果 |
| `data/reports/` | 预验收/正式验收汇总 |
| `data/logs/` | 通用运行与诊断日志 |
| `data/locks/` | 进程互斥锁；运行中不得删除 |

这些目录只跟踪 `.gitkeep`，大文件不进入Git。复制项目到另一台电脑时，代码仓库之外还必须单独迁移目标地图和VGL模型，或在那里重新运行建图流程。
