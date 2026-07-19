# 手动键盘建图、保存与 RViz 导航全流程

本流程用于在酷家乐场景中人工驾驶 Jackal 建图，并在随后启动时通过 cuVGL 自动全局
定位，在 RViz 中用 `2D Goal Pose` 手动下发导航目标。建图和导航都会同时打开 Isaac Sim
第三人称跟随 GUI 与 RViz；相机图像显示默认关闭，避免重复渲染/传输影响帧率。

## 1. 一次性准备

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
git lfs install
git lfs pull
./scripts/build.sh
```

每次建图必须使用一个空的新地图名。仓库已有的正式地图
`kujiale_jackal_8cam` 不会被覆盖。下面示例使用 `kujiale_manual_20260719`；若同名目录
已经有内容，请换一个名字。

## 2. 启动手动键盘建图

```bash
./scripts/run_manual_mapping.sh --map kujiale_manual_20260719
```

脚本会依次启动：

1. Isaac Sim GUI：只显示第三人称 Jackal 跟随视角；
2. 四组 Hawk / 8 路图像、front 深度和 IMU；Jackal LiDAR 不创建；
3. 8 相机 cuVSLAM、静态 nvblox 与临时 MCAP；
4. 建图专用 RViz：显示机器人、TF、cuVSLAM 轨迹、nvblox mesh 和 ESDF；
5. 当前终端中的键盘控制。

请保持启动命令所在终端获得键盘焦点。键位如下：

| 键 | 动作 |
|---|---|
| `W` | 前进 |
| `S` | 后退（仅人工建图允许） |
| `A` | 原地/行进中左转 |
| `D` | 原地/行进中右转 |
| `1`–`5` | 设置速度档位：精细到快速；默认 `3` 档 |
| `+` / `-` | 上调 / 下调一档速度 |
| `Space` | 立即停车 |
| `P` | 显示 cuVSLAM 里程计累计行驶距离 |
| `Q` | 行驶至少 2m 后，停车并进入验证、保存流程 |

默认 `3` 档为 `0.55 m/s`、`1.00 rad/s`；最高 `5` 档为约 `0.74 m/s`、`1.35 rad/s`，
仍低于仿真控制硬上限。调档会先停车，必须重新按运动键，避免在行驶中突然加速。松开
运动键超过 `0.18 s` 后会自动发零速度。不要按住按键高速穿门；建议在门洞使用 `1`–`2`
档，在开阔区域可使用 `3`–`5` 档，并缓慢旋转，使前、左、右、后 Hawk 获得足够视角重叠。
键盘事件来自启动脚本的终端，不来自 Isaac Sim 或 RViz 窗口；把 GUI 和终端并排放置，但
驾驶时让终端保持焦点。`Q` 前的 2m 仅是防止误保存静止/抖动数据的最低门槛，不代表已经
覆盖完整场景；建议随时按 `P` 查看距离，并实际遍历所有需要导航的区域。
结束前回到起点附近并再次观察已建区域，形成闭环；cuVSLAM 保存门禁要求闭环误差不
超过 `0.50 m`。

## 3. 保存地图

完成覆盖后，在键盘终端按一次 `Q`。此时不是立即退出，脚本还会自动执行：

1. 停车并安全结束 MCAP；
2. 保存 nvblox `.nvblx`、mesh 与 occupancy `map.yaml/map.pgm`；
3. 保存在线 cuVSLAM 数据库与全局优化轨迹；
4. 检查轨迹时长、位姿数、闭环和平面几何；
5. 关闭 Isaac Sim 和 RViz；
6. 从同一轨迹与 8 路数据生成 cuVGL keyframes、vocabulary 和 BoW index；
7. 写入带文件哈希的 `manifest.json`，成功后删除临时 raw MCAP。

cuVGL 离线索引可能耗时较长。在终端出现以下内容之前不要关闭终端：

```text
Map complete: .../data/maps/kujiale_manual_20260719
```

只有出现这行后，`manifest.json` 才已写入，可以启动导航。手动建图即使发生物理接触仍会
继续生成可用于手动导航的地图，但 `manifest.json` 会永久记录碰撞次数，并标为
`manual_collision_recorded`；它不能作为正式“零物理碰撞”或静态避障验收的证据。自动建图
与正式验收仍严格要求零碰撞。若终端显示 `Map was not promoted`，说明除手动接触外还有其他
关键门禁失败；该次地图会保留 nvblox、occupancy、cuVSLAM 与诊断日志，但不能用于导航。
cuVSLAM 保存失败会直接打印累计平面路径、闭环误差和时长；详细原因在对应
`data/logs/mapping/<时间>/save-cuvslam.log` 与地图目录的 `cuvslam/save_report.json`。
后续仿真/覆盖门禁失败则记录在 `mapping-validation.json`。

临时 MCAP 使用可按时间索引的 `zstd_fast` 配置，以便离线 cuVGL 对齐所有四组 Hawk 图像；
生成阶段还会要求至少 40 组同步关键帧。若这个质量门禁失败，不会写入 manifest，避免把只有
少量关键帧的视觉定位图当作可用地图。

若在线 cuVSLAM、nvblox 和占据栅格均已保存，但离线 cuVGL 转换失败，原始 MCAP 会保留，
不需要重新驾驶。修复转换问题后执行：

```bash
./scripts/recover_manual_map.sh --map <地图名>
```

恢复脚本会从八路图像中只保留完整同步组，运动时按位姿选关键帧，停车时用最大 `0.5 s`
间隔的连续帧避免官方转换器失去同步，随后生成 cuVGL、写入 `manifest.json` 并运行地图
完整性检查。只有看到 `Map recovery complete:` 或原流程中的 `Map complete:` 才表示地图
可以进入导航流程。

随后检查地图：

```bash
python3 tools/check_map_manifest.py \
  data/maps/kujiale_manual_20260719
```

自定义手工地图默认被 Git 忽略，但可以直接用于本机导航。若保存门禁失败，脚本不会
伪造 manifest，也不会删除临时数据；查看最新的 `data/logs/mapping/<时间>/`，修正后
优先使用上述恢复命令继续离线生成。只有在线地图质量门失败时才需要换新地图名重建。

## 4. 出生点是否需要人工标定

不需要在 RViz 使用 `2D Pose Estimate`，也不需要为每次仿真手工填写出生点到地图的
变换。本工作流使用 cuVGL 自动完成全局定位：

```text
当前 6 路图像
  -> cuVGL 与保存的 8 路地图特征匹配
  -> map 中的全局位姿
  -> /visual_slam/initial_pose
  -> cuVSLAM 连续健康跟踪
  -> 固定 map -> odom 锚点
  -> /localization/ready=true
```

就绪门禁还要求 occupancy map 非空、`map→odom` 已发布且 Nav2 action server 可用。
因此目标不会在定位尚未完成时误发。cuVGL 免除的是“已知场景中的人工初始位姿标定”，
并不意味着可以出生在未建图区域：场景 USD 必须与建图时一致，机器人应位于地图覆盖且
视觉特征可辨认、无碰撞的位置。默认仿真出生点满足这一条件。NVIDIA 官方将 cuVGL
定义为从地图与当前双目/多双目图像输出全局位姿，并建议定位轨迹靠近建图轨迹；参见
[Visual Global Localization 概念](https://nvidia-isaac-ros.github.io/v/release-4.0/concepts/visual_global_localization/index.html)
与 [cuVGL ROS API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mapping_and_localization/isaac_ros_visual_global_localization/index.html)。

## 5. 启动手动 RViz 导航

使用刚才保存的地图：

```bash
./scripts/run_manual_navigation.sh --map kujiale_manual_20260719
```

也可直接使用仓库内已经验证的正式地图：

```bash
./scripts/run_manual_navigation.sh --map kujiale_jackal_8cam
```

此命令会同时启动 Isaac Sim GUI、6 路导航相机、cuVGL/cuVSLAM、nvblox、Nav2 和导航
RViz。cuVSLAM 与 cuVGL 的运行时输入数都严格为 6；后向 Hawk 的 render product 与
ROS publisher 在导航中不创建。脚本还会在 loopback 上启动项目专用 Fast DDS discovery
server，避免 GPU 高负载期间晚启动的 Nav2/RViz 节点漏发现；退出时会一并关闭。终端会
列出当前仍在等待的组件；只有看到以下提示后才下发目标：

```text
MANUAL_NAVIGATION_READY action=/navigate_to_pose fixed_frame=map
Navigation ready: use RViz 2D Goal Pose
```

## 6. 在 RViz 发布目标

1. 确认 RViz 左上角 `Fixed Frame` 为 `map`；配置已默认完成。
2. 在工具栏选择 `2D Goal Pose`。
3. 在 occupancy map 的已知自由区域按下鼠标并拖动箭头；箭头方向是期望终点朝向。
4. 松开鼠标后，RViz 向 `/goal_pose` 发布 `PoseStamped`。
5. 就绪门控桥将它转发到 Nav2 `/navigate_to_pose` action。

新的点击会取消并替换正在执行的旧目标。定位短暂失效时，桥会取消当前手动目标并把
最新目标排队，等待全局定位恢复后再发送。RViz 默认显示：

- occupancy map、全局/局部 costmap；
- Jackal、TF、cuVSLAM 轨迹；
- 全局规划与 MPPI 局部路径；
- front depth 安全扫描与 Collision Monitor 区域。

front RGB/depth、左右/后向图像、nvblox mesh/ESDF 和 front depth 大点云显示默认关闭；
需要诊断时可在 `Displays` 中单独开启。关闭显示不会关闭导航需要的传感器或 nvblox
计算，只是不让 RViz 订阅和绘制这些高负载数据。正常人工测试只看 Isaac Sim 第三人称
跟随视角即可。

本功能验证只保证启动、自动定位、RViz 目标转发与可视化链路正常，不代表某个手动目标
一定能成功到达。目标应选择在已知自由空间内，并由你按实际导航行为评估效果。

## 7. 停止与故障定位

导航完成后在启动终端按一次 `Ctrl-C`。脚本只停止自己创建的 Isaac Sim、ROS 与 RViz
进程组，不会使用全局 `pkill/killall`。

若 180 秒内没有进入 ready：

1. 确认出生位置在这张地图覆盖区域，环境几何与光照没有被改动；
2. 查看 `data/logs/navigation/<时间>/navigation.log` 中 cuVGL、cuVSLAM 和
   `navigation_tf_bridge` 信息；
3. 确认 6 个导航图像话题存在，后向话题不存在；
4. 不要用 `2D Pose Estimate` 绕过失败，因为那会掩盖 cuVGL 全局定位链路问题。

若需要从另一个终端做只读复核，不要直接运行普通 `ros2 topic/action` 命令；它不是当前
discovery server 的 late-join super client。使用项目提供的检查入口，它不会发布目标：

```bash
cd /home/lyb/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/check_manual_navigation.sh
```

成功输出同时证明 occupancy map、`map→odom`、cuVGL/cuVSLAM 就绪、Nav2 action server
可用，且前/左/右 Hawk 正在发布、后 Hawk 没有发布。
