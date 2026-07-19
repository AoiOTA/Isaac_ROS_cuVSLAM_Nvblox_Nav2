# 酷家乐四组 Hawk（八路图像流）建图

## 输入契约

`mapping_8cam` 固定按以下顺序使用 8 路单目图像：

1. front left / right
2. left left / right
3. right left / right
4. back left / right

四组 Hawk 均为 `1280×800 @ 10 Hz`，front IMU 为与当前 PhysX 步频一致的 `60 Hz`。nvblox 只接收 front Hawk 左目的 `640×400` 原生模拟深度；视觉建图与 nvblox 不共享合成双目深度。
front Hawk 深度的近裁剪面固定为 `0.40 m`，用于在渲染源头排除相机下方的 Jackal
车体；它不是雷达量程，也不会创建任何 LiDAR 资源。

Jackal LiDAR 明确关闭，模拟器不会创建 LiDAR prim、render product 或 ROS publisher。
导航中名为 `/front_depth/scan[_raw]` 的 `LaserScan` 是由 front Hawk 原生深度投影
得到的二维安全表示，不是雷达数据。

话题清单在 `ros2_ws/src/jackal_bringup/config/mapping_topics_8cam.yaml`。cuVSLAM 建图要求 `num_cameras=8` 且 `min_num_images=8`，因此任一路缺帧都不能悄悄退化为少相机地图。

## 建图命令

```bash
./scripts/run_mapping.sh --map kujiale_jackal_8cam --auto --headless
```

`config/mapping_coverage.yaml` 保存一条约 38 m 的低速闭环覆盖路线。路线只把参考分支中
同一 USD、同一出生点且经过三次冷启动标定的 occupancy 当作规划依据；旧栅格不会进入
新地图，产物仍必须来自本轮实时八路 MCAP 与 front Hawk depth。自动驱动会检查：

- 线速度始终非负且不超过 `0.28 m/s`；
- cuVSLAM 跟踪、路线进度和最大横向偏差；
- 每个开放区域的定点扫描与最终闭环；
- simulator 报告中的四 Hawk/八路拓扑、LiDAR 关闭和 PhysX 零碰撞。
- 在线全局优化轨迹至少 500 个位姿、覆盖至少 80% 驾驶时长；
- 优化轨迹相对 ground truth 路程比在 `[0.85, 1.15]`、闭环误差不超过 `0.50 m`；
- 高度范围不超过 `0.10 m`，三维/平面路程比不超过 `1.01`。

人工建图仍可用：

```bash
./scripts/run_manual_mapping.sh --map kujiale_manual_20260719
```

该脚本的共同硬保护包括：

- 必须显式选择 `--interactive` 或 `--auto`，不能静默选择驾驶方式；
- 人工模式必须有 TTY 且使用 GUI，默认同时启动建图 RViz；
- 目标地图目录非空时拒绝覆盖；
- 同一时刻只允许一个 mapping workflow。

GUI 出现后保持启动终端获得焦点，用 `W/S/A/D` 驾驶，`Space` 停车，`P` 查看累计距离，
`Q` 保存。未达到 2m 最低真实运动时 `Q` 会被拒绝；2m 只是防误触门槛，仍应缓慢遍历所有
目标区域、门洞与走廊，并形成闭环。
完整操作与 cuVGL 自动出生点定位见
[手动键盘建图、保存与 RViz 导航全流程](manual_mapping_navigation.md)。

酷家乐若干开门洞底边包含与地面共面的三角碰撞面。PhysX 的 contact offset 会在轮胎
尚有正间隙时提前发出 report；只有“所有 contact 都是非负间隙且冲量范数不超过
`1e-9`”才归为无物理接触的 proximity record；同一 event 中的 record 逐个拆分。剩余
已经发生接触的 record 还必须同时满足“轮子或 caster、接触点距标定地面不超过 3 cm、
接触法向的竖直分量绝对值至少 0.8”才归为支撑接触。负间隙、可测冲量、底盘接触、较高
接触或水平墙面法向仍按真实碰撞计数；报告会分别保留每一 actor pair 的全部 record 和
真实受力 record 的最小间隙、最大冲量和最高接触点用于审计。

## 生成流程

```text
8 RGB + 8 CameraInfo + front depth + IMU + TF + clock
  + cuVSLAM odometry / optimized-pose / status diagnostics
  -> indexed MCAP (required topic audit immediately after recording)

live 8-camera VIO/cuVSLAM
  -> save_map: cuVSLAM database
  -> get_all_poses: globally optimized TUM trajectory
  -> planar/path/closure quality gate
  -> map-frame odometry pose bag + rectified MCAP frames
  -> ALIKED features + cuVGL vocabulary/BoW index

live cuVSLAM pose + front native depth
  -> online nvblox mesh/ESDF coverage preview only

globally optimized cuVGL keyframes + matching front native depth (near clip 0.40 m)
  -> offline nvblox TSDF mesh pass (repeated-evidence weight gate)
  -> offline nvblox static occupancy pass
  -> occupancy coverage gates (Nav2 applies footprint/inflation later)

all runtime groups
  -> manifest.json
  -> delete temporary extracted images; retain indexed MCAP by default
```

这对应“一次在线采集和预览、停止仿真后离线统一生成、重新在线加载导航”三段式流程。
Isaac ROS 4.5 自带的 `create_map_offline.py` 是上游参考编排器，但其 `depth_model` 入口只接受
ESS 或 FoundationStereo；本项目明确使用 Isaac Sim front Hawk 原生深度，所以不调用该单体
脚本做深度推理。项目分别调用同一官方链路的 `rosbag_to_mapping_data`、
`create_cuvgl_map.py` 和 `nvblox_ros fuse_cusfm`，并将在线 IMU/VIO 保存的 cuVSLAM 数据库与
最终优化轨迹作为唯一位姿解。这样既遵循官方离线重融合顺序，也不会引入未要求的深度模型。

cuVSLAM、cuVGL、nvblox 和 occupancy 必须来自同一次采集与同一最终优化 SLAM 解。项目不再用
离线纯视觉 cuVSLAM 覆盖在线 VIO 数据库；这种混用在平面 A/B 中曾产生 `1.92 m`
闭环误差。`create_vgl_map.sh` 将在线 `GetAllPoses` 导出的 TUM 轨迹转换成带 `map` frame 的
标准 ROS odometry pose bag，再传给 `rosbag_to_mapping_data` 选帧。这既保留了在线全局优化
坐标，又避开 TUM 直传在静止段造成跨数秒关键帧配对失败的问题。默认同步窗为 `40000 µs`，并
把同一值写入地图内冻结的 cuVGL runtime config。Isaac ROS 4.5 的 `GetAllPoses` 会把全局优化位姿的
`PoseStamped.frame_id` 留空；保存报告明确记录这一版本策略，几何门槛仍全部执行。官方接口说明见
[Isaac ROS Visual SLAM API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html)
与 [Isaac Mapping ROS](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mapping_and_localization/isaac_mapping_ros/index.html)。

在线静态 nvblox 使用 `global_frame=map`，但它只用于 RViz 覆盖检查。回环发生时，
`map→odom` 的历史修正不会重新融合旧体素；因此最终 mesh 和 occupancy 都从 MCAP 按优化
关键帧重建。最终 2D 图使用 nvblox 静态概率占据融合，2.5 cm 端点半带形成单个 5 cm
voxel 宽的表面，不包含车体 footprint 或 inflation。导航时的动态滚动 nvblox 是另一份配置，仍使用
连续 `odom`；footprint 和 inflation 只由 Nav2/路线验证层施加一次。

`config/offline_mapping.yaml` 冻结原生深度范围、同步窗、TSDF 与静态占据参数。质量门禁
要求地图至少 35% 为已知自由、至少 50% 为已知区域且未知不超过 50%；这些是场景覆盖
比例，不是固定帧数。门禁会拒绝此前仅 19.8% 自由、69.2% 未知的在线图。每次结果还在
`occupancy/save_report.json` 记录三类像素、连通分量和全部检查项；随后使用与正式验收
一致的 footprint 代理检查北房间、南走廊、东房间均与出生点连通，结果写入
`occupancy/route_validation.json`。本次离线参数与路线契约分别冻结在地图的 `config/`
目录，manifest 哈希可检测事后改动。

任一门槛失败时，目标地图不会写 manifest，MCAP 也不会删除，便于从同一原始数据复盘。
成功时默认把原始包保留在 `data/bags/<地图名>_<run-id>/capture`；只有显式传入
`--discard-bag` 才会删除，临时抽帧目录则始终在成功后清理。

## 运行时地图

```text
data/maps/kujiale_jackal_8cam/
├── config/       cuVGL runtime pb.txt
├── cuvgl/        keyframes, vocabulary, bow_index
├── cuvslam/      在线 cuVSLAM database、optimized_poses.tum、质量报告
├── mesh/         PLY
├── nvblox/       .nvblx, PLY, rates/timings
├── occupancy/    map.yaml, map.pgm
└── manifest.json
```

`manifest.json` 记录：

- 三个源 USD 的路径、default prim 与 SHA-256；
- `mapping_8cam=8`、`navigation_6cam=6`、后向导航 render product 为 false；
- 8 个图像、8 个 CameraInfo、front 原生深度/CameraInfo、IMU、TF 与 `/clock` 的实际消息数；
- cuVSLAM odometry、优化位姿与状态诊断话题的实际消息数（诊断缺失不替代必需输入门禁）；
- 每个运行时目录的文件数、总字节数与 tree hash；
- raw capture 不进入地图/Git；manifest 记录它是本机保留还是显式删除；
- 当前地图的验证状态。

完整检查：

```bash
python3 tools/check_map_manifest.py data/maps/kujiale_jackal_8cam
```

检查器会拒绝缺失的 cuVSLAM DB、cuVGL keyframe metadata/vocabulary/BoW index、nvblox
binary、mesh、occupancy、被改动的 artifact hash、旧资产 hash，以及 `.mcap`/`.db3` 或
capture/offline/online_cuvslam 泄漏。

## Git LFS

只有这一张地图的运行时文件可提交。`.gitattributes` 对数据库、protobuf/bin、keyframe 图像、nvblox、mesh 和 occupancy PGM 启用 LFS；其他地图、raw bag、TensorRT engine、日志与中间数据仍被忽略。

```bash
git check-attr filter -- \
  data/maps/kujiale_jackal_8cam/nvblox/kujiale.nvblx
git lfs status
```

不要把 temporary MCAP 复制进地图目录，也不要提交 `data/bags`。

## 导航侧复用

导航使用同一份 8 相机 cuVSLAM calibration，但运行时只发布前、左、右共 6 路图像；`min_num_images=2` 允许 cuVSLAM 从可用流跟踪。cuVGL 使用独立的 6 相机输入配置，后向话题和渲染资源均不存在。

地图生成后必须用真实 occupancy map 验证 `config/acceptance.yaml` 中的候选路线，未通过路线检查的地图不能进入 20 次正式实验。
