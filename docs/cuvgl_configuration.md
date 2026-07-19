# cuVGL完整配置、迁移与调参手册

本文对应本项目已实测的 Ubuntu 24.04、ROS 2 Jazzy、Isaac ROS 4.5.0、CUDA 13.0、TensorRT 10.13.3.9、RTX 4090 和 Isaac Sim 6.0.1。目标是在另一台电脑上从裸机环境完成前向双目 cuVGL 建图、TensorRT 引擎导出、全局定位，并把结果交给 cuVSLAM。命令均以Isaac ROS 4.5 Debian包的实际安装布局为准。

官方参考：

- [Visual Global Localization节点和API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mapping_and_localization/isaac_ros_visual_global_localization/index.html)
- [isaac_mapping_ros工具](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mapping_and_localization/isaac_mapping_ros/index.html)
- [cuVGL定位教程](https://nvidia-isaac-ros.github.io/concepts/visual_global_localization/tutorials/tutorial_cuvgl_localization.html)

## 1. 系统组成和数据方向

```text
同步双目mono8 + CameraInfo + base_link静态TF
  → MCAP
  → rosbag_to_mapping_data生成EDEx
  → cuVSLAM离线计算轨迹和地图坐标系
  → 按距离/角度抽取地图帧
  → ALIKED提取关键点
  → BoW vocabulary和index
  → cuVGL map

在线同步双目
  → cuVGL /visual_localization/pose (map→base_link)
  → /visual_slam/initial_pose
  → cuVSLAM在已加载地图中局部搜索
  → cuVSLAM继续独占map→odom→base_link
```

cuVGL只提供离散的全局位姿，不发布本项目主TF。`publish_map_to_base_tf=false`和`publish_map_to_odom_tf=false`必须保持关闭。cuVSLAM是`map→odom`及`odom→base_link`的唯一发布者。

## 2. 安装和版本检查

先按[阶段1裸机安装手册](installation.md)完成Isaac ROS源、CUDA和TensorRT，再执行：

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-isaac-mapping-ros \
  ros-jazzy-isaac-ros-visual-mapping \
  ros-jazzy-isaac-ros-visual-global-localization \
  ros-jazzy-isaac-ros-visual-slam

source /opt/ros/jazzy/setup.bash
ros2 pkg prefix isaac_mapping_ros
ros2 pkg prefix isaac_ros_visual_mapping
ros2 pkg prefix isaac_ros_visual_global_localization
dpkg-query -W 'ros-jazzy-isaac-*mapping*' 'ros-jazzy-isaac-ros-visual-global-localization'
```

本项目实测期望三个prefix均为`/opt/ros/jazzy`，版本为4.5.0。检查工具位置：

```bash
ls -l /opt/ros/jazzy/lib/isaac_mapping_ros/rosbag_to_mapping_data
ls -l /opt/ros/jazzy/lib/isaac_mapping_ros/create_map_offline.py
ls -l /opt/ros/jazzy/lib/isaac_ros_visual_mapping/create_cuvgl_map.py
ls -l /opt/ros/jazzy/bin/visual_mapping/export_extractor_engine
ls -l /opt/ros/jazzy/bin/visual_mapping/export_lightglue_engine
```

## 3. 输入数据契约

单组前向双目使用两个相机，固定顺序为左、右：

| 输入 | 要求 |
|---|---|
| 左右图像 | `sensor_msgs/Image`，`mono8`，1280×800，约30 Hz |
| CameraInfo | 与相应图像同stamp和frame_id；K、P非零 |
| 双目同步 | 左右stamp差不超过3 ms；本仿真实际为同一render tick |
| 基线 | 右相机P矩阵`Tx=-fx*baseline`；本机基线0.15 m |
| TF | `base_link→左右optical frame`为静态且唯一 |
| optical轴 | ROS约定：Z向前、Y向下 |
| 时间 | 所有节点`use_sim_time=true`，时间不得回退 |

本项目topic配置文件为：

```yaml
stereo_cameras:
  - name: front_stereo_camera
    left: /front_stereo_camera/left/image_raw
    left_camera_info: /front_stereo_camera/left/camera_info
    right: /front_stereo_camera/right/image_raw
    right_camera_info: /front_stereo_camera/right/camera_info
```

在另一台电脑上先检查：

```bash
ros2 topic hz /front_stereo_camera/left/image_raw
ros2 topic hz /front_stereo_camera/right/image_raw
ros2 topic echo --once /front_stereo_camera/left/camera_info
ros2 run tf2_ros tf2_echo base_link front_stereo_camera_left_optical
ros2 run tf2_ros tf2_echo base_link front_stereo_camera_right_optical
```

## 4. 采集MCAP

机器人应覆盖目标区域，包含平移、左右转向和回到已见区域。定位轨迹最好距建图轨迹不超过1 m。录制：

```bash
export ROS_DOMAIN_ID=47
ros2 bag record --storage mcap --output /data/bags/warehouse_mapping \
  /front_stereo_camera/left/image_raw \
  /front_stereo_camera/right/image_raw \
  /front_stereo_camera/left/camera_info \
  /front_stereo_camera/right/camera_info \
  /front_stereo_imu/imu /tf /tf_static /clock

ros2 bag info /data/bags/warehouse_mapping
```

停止时向rosbag进程发送SIGINT，等待缓存完整落盘。不要用`kill -9`。本项目自动入口是：

```bash
./scripts/run_mapping.sh --map warehouse_v1
```

它执行闭环路线与 MCAP 采集、cuVSLAM/cuVGL 优化，然后按优化关键帧离线生成
nvblox Mesh/occupancy；在线 nvblox 只保留为覆盖质检日志。

## 5. 导出TensorRT引擎

引擎和GPU架构、TensorRT版本有关；换GPU或TensorRT后必须重新导出：

```bash
source /opt/ros/jazzy/setup.bash
PKG_PREFIX=$(ros2 pkg prefix isaac_ros_visual_mapping)
PKG_SHARE=$(ros2 pkg prefix isaac_ros_visual_mapping --share)
OUTPUT_MODEL_DIR=/data/models/vgl
mkdir -p "$OUTPUT_MODEL_DIR"
cp -a "$PKG_SHARE/models/." "$OUTPUT_MODEL_DIR/"

$PKG_PREFIX/bin/visual_mapping/export_extractor_engine \
  --feature_type=aliked \
  --configure_file="$PKG_SHARE/configs/isaac/keypoint_creation_config.pb.txt" \
  --model_dir="$PKG_SHARE/models" \
  --output_model_dir="$OUTPUT_MODEL_DIR"

$PKG_PREFIX/bin/visual_mapping/export_lightglue_engine \
  --feature_type=aliked \
  --worker_config_file="$PKG_SHARE/configs/isaac/matching_task_worker_config.pb.txt" \
  --model_dir="$PKG_SHARE/models" \
  --output_model_dir="$OUTPUT_MODEL_DIR"

find "$OUTPUT_MODEL_DIR" -type f -name '*.engine' -printf '%p %s bytes\n'
```

RTX 4090实测生成约5.0 MB的ALIKED engine和约27.2 MB的LightGlue engine。导出器可能先打印无法写入只读`/opt/ros`目录；只要随后明确保存到`OUTPUT_MODEL_DIR`且文件非零，就是成功。

## 6. 生成EDEx和对齐的cuVSLAM地图

> 当前 `kujiale_jackal_8cam` 工作流不采用本节的离线纯视觉 cuVSLAM 重算。四 Hawk
> 建图由在线 VIO/cuVSLAM 保存数据库和 `GetAllPoses` 优化轨迹；工作流将该 TUM 转换为带
> `map` frame 的 ROS odometry pose bag，再与同一 MCAP 传给 `rosbag_to_mapping_data`，避免
> TUM 直传在静止段的关键帧时间跳变。具体以 `scripts/create_vgl_map.sh` 和 `docs/mapping.md`
> 为准。本节仅保留为通用离线数据集参考。

```bash
export ISAAC_ROS_WS=/absolute/path/to/ros2_ws
MAP_ROOT=/data/maps/warehouse_v1
BAG=/data/bags/warehouse_mapping
TOPICS=/absolute/path/to/mapping_topics.yaml
mkdir -p "$MAP_ROOT/offline"

ros2 run isaac_mapping_ros create_map_offline.py \
  --sensor_data_bag="$BAG" \
  --map_dir="$MAP_ROOT/offline" \
  --steps_to_run edex compute_poses \
  --camera_topic_config="$TOPICS" \
  --base_link_name=base_link \
  --use_raw_image=True \
  --print_mode=tail
```

关键输出为`offline/edex`、`offline/cuvslam_map/data.mdb`、`offline/poses`及`offline/map_frames/rectified`。

## 7. 生成cuVGL地图

Isaac ROS 4.5 Debian包有两个需要显式规避的布局问题：

1. `create_cuvgl_map.py`默认推导`<prefix>/bin/visual_mapping`，但Debian映射二进制实际在`<prefix>/lib/isaac_ros_visual_mapping`。
2. `create_map_offline.py --vgl_model_dir`在4.5包中没有传给内部cuVGL调用，会忽略项目缓存并重复构建引擎。

使用以下已实测命令：

```bash
VM_PREFIX=$(ros2 pkg prefix isaac_ros_visual_mapping)
VM_SHARE=$(ros2 pkg prefix isaac_ros_visual_mapping --share)

ros2 run isaac_ros_visual_mapping create_cuvgl_map.py \
  --map_folder="$MAP_ROOT/offline/cuvgl_map" \
  --raw_image_folder="$MAP_ROOT/offline/map_frames/rectified" \
  --extract_feature --build_bow_index \
  --feature_type=aliked --print_mode=tail \
  --binary_folder_path="$VM_PREFIX/lib/isaac_ros_visual_mapping" \
  --config_folder_path="$VM_SHARE/configs/isaac" \
  --model_dir="$OUTPUT_MODEL_DIR"
```

成功输出必须包含`keyframes/frames_meta.json`、左右相机关键帧`.pb`、`vocabulary/bow_vocabulary_*.pb`和`bow_index.pb`。将同一批次的`offline/cuvslam_map`和`offline/cuvgl_map`分别复制为运行时`cuvslam/`和`cuvgl/`，并复制`$VM_SHARE/configs/isaac`到地图的`config/`。三者不能混用不同建图批次。

## 8. 完整ROS参数

```yaml
visual_global_localization_node:
  ros__parameters:
    use_sim_time: true
    num_cameras: 2
    stereo_localizer_cam_ids: "0,1"
    camera_optical_frames:
      - front_stereo_camera_left_optical
      - front_stereo_camera_right_optical
    map_frame: map
    odom_frame: odom
    base_frame: base_link
    enable_rectify_images: false
    publish_rectified_images: false
    enable_continuous_localization: false
    use_initial_guess: false
    publish_map_to_base_tf: false
    publish_map_to_odom_tf: false
    image_sync_match_threshold_ms: 3.0
    image_buffer_size: 100
    image_qos_profile: SENSOR_DATA
    localization_precision_level: 2
    vgl_frequency: 1.0
    vgl_enable_debug: false
    verbose_logging: false
    init_glog: false
    glog_v: 0
```

仿真图像无畸变且已作为rectified输入，所以`enable_rectify_images=false`。真实相机raw畸变图改为true；上游已矫正时保持false，避免二次矫正。

## 9. Launch、触发和输出

ComposableNode使用插件`nvidia::isaac_ros::visual_global_localization::VisualGlobalLocalizationNode`。输入remap为`visual_localization/image_0/camera_info_0`到左目、`image_1/camera_info_1`到右目，参数必须提供`map_dir`、`config_dir`和`model_dir`。

下面是本机已执行的完整最小launch结构；将`vgl.yaml`安装到自己的bringup包后即可迁移。`vgl_pose_relay`是本仓库提供的校验/转发节点，若另一台电脑使用本仓库，正常`colcon build`后会一并安装。

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    vgl = ComposableNode(
        package="isaac_ros_visual_global_localization",
        plugin=(
            "nvidia::isaac_ros::visual_global_localization::"
            "VisualGlobalLocalizationNode"
        ),
        name="visual_global_localization_node",
        parameters=[
            "/absolute/path/to/vgl.yaml",
            {
                "map_dir": LaunchConfiguration("vgl_map_dir"),
                "config_dir": LaunchConfiguration("vgl_config_dir"),
                "model_dir": LaunchConfiguration("vgl_model_dir"),
            },
        ],
        remappings=[
            ("visual_localization/image_0", "/front_stereo_camera/left/image_raw"),
            ("visual_localization/camera_info_0", "/front_stereo_camera/left/camera_info"),
            ("visual_localization/image_1", "/front_stereo_camera/right/image_raw"),
            ("visual_localization/camera_info_1", "/front_stereo_camera/right/camera_info"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("vgl_map_dir"),
        DeclareLaunchArgument("vgl_config_dir"),
        DeclareLaunchArgument("vgl_model_dir"),
        DeclareLaunchArgument("cuvslam_map_dir"),
        ComposableNodeContainer(
            name="vgl_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[vgl],
        ),
        Node(
            package="nova_carter_experiments",
            executable="vgl_pose_relay",
            parameters=[
                {"use_sim_time": True},
                {"cuvslam_map_dir": LaunchConfiguration("cuvslam_map_dir")},
            ],
            output="screen",
        ),
    ])
```

启动时四个目录必须属于同一软件/地图批次：

```bash
ros2 launch your_bringup vgl.launch.py \
  vgl_map_dir:=/data/maps/warehouse_v1/cuvgl \
  vgl_config_dir:=/data/maps/warehouse_v1/config \
  vgl_model_dir:=/data/models/vgl \
  cuvslam_map_dir:=/data/maps/warehouse_v1/cuvslam
```

本项目启动和触发：

```bash
./scripts/run_vgl.sh warehouse_v1
ros2 service call /visual_localization/trigger_localization std_srvs/srv/Trigger '{}'
ros2 topic echo --once /visual_localization/pose
ros2 topic echo /diagnostics
```

正常成功日志包含`Localization completed with status: 1`和`Publish global localization pose`。

## 10. 向cuVSLAM注入位姿

cuVSLAM需配置`enable_localization_n_mapping=true`、`load_map_folder_path=/data/maps/warehouse_v1/cuvslam`、`localize_on_startup=false`及`enable_request_hint=true`。本项目把可靠QoS的`/visual_localization/pose`转发到`/visual_slam/initial_pose`，转发前检查frame为`map`、所有位姿/协方差有限且cuVSLAM地图存在。

cuVSLAM成功日志：

```text
Trying to localize in map '...' around [...]
Map folder '...' exists and contains database files.
Successfully localized at {...}
```

若cuVSLAM发布`/visual_slam/trigger_hint`，说明hint局部搜索失败；状态机应再次触发cuVGL，最多重试3次。正常恢复以`/visual_slam/status.vo_state == 1`连续样本为导航恢复条件。

## 11. 调参顺序

每次只改变一组参数，并固定相机、地图、初始位姿和seed。

1. 先修左右时间戳、CameraInfo baseline和静态TF。同步门槛可从3 ms调到5 ms，不建议超过10 ms。
2. 正式导航保持`localization_precision_level=2`；弱纹理先试1，0只作为恢复兜底并加强一致性检查。
3. 正常导航保持`enable_continuous_localization=false`和`use_initial_guess=false`。
4. 最后才编辑项目复制的`keypoint_creation_config.pb.txt`、`image_retrieval_config.pb.txt`、`localizer_config.pb.txt`和`matching_task_worker_config.pb.txt`，绝不直接改`/opt/ros`。
5. 优先增加地图覆盖和视角，再降低precision，最后才放宽最小对应点、RANSAC或单结果接受策略。

修改ALIKED/LightGlue模型、TensorRT输入维度或FP16后，删除旧engine并重新导出。只改PNP门槛通常无需重导engine；改BoW或特征类型需要重建cuVGL地图。

## 12. 常见故障

| 现象 | 原因和处理 |
|---|---|
| 默认脚本提示二进制目录不存在 | 显式传`/opt/ros/jazzy/lib/isaac_ros_visual_mapping` |
| 反复构建TensorRT后超时 | 显式传项目`--model_dir`，不要依赖4.5内部`--vgl_model_dir`传递 |
| `No pose available for keyframe`很多 | 检查MCAP丢包、双目同步和cuVSLAM tracking |
| VGL无pose | 确认服务已触发、mono8、两份CameraInfo和TF；临时precision 1/0诊断 |
| `add camera_params_id`缺一个 | 某一路图像或CameraInfo没有同步进入节点 |
| engine反序列化失败 | GPU架构/TensorRT变化，删除并重导 |
| VGL成功但cuVSLAM失败 | 两种地图不是同批次，或hint超出cuVSLAM搜索范围 |
| TF跳变/重复 | 关闭VGL两个TF开关，只让cuVSLAM发布主TF |

## 13. 验收

单次测试验证：cuVGL输出frame为`map`且有限、relay发布到`/visual_slam/initial_pose`、cuVSLAM日志出现`Successfully localized`、`vo_state`连续成功，并且没有新的`/visual_slam/trigger_hint`。

```bash
./scripts/run_phase7_tests.sh warehouse_v1
```

本机五次独立仿真重启全部成功，均接收cuVGL位姿、恢复cuVSLAM tracking，失败hint为0。完整证据见[阶段7验证记录](phase7_validation.md)。

## 14. 阶段9前向双目动态导航补充

阶段9继续使用两台前向相机，不启用四向模式：

```yaml
visual_global_localization_node:
  ros__parameters:
    num_cameras: 2
    stereo_localizer_cam_ids: "0,1"
    camera_optical_frames:
      - front_stereo_camera_left_optical
      - front_stereo_camera_right_optical
    enable_continuous_localization: false
    publish_map_to_base_tf: false
    publish_map_to_odom_tf: false
    image_sync_match_threshold_ms: 3.0
    localization_precision_level: 2
```

地图必须使用与cuVSLAM同一份离线轨迹生成的`warehouse_v2_front`。运行入口先把地图冻结的pb.txt复制到`data/runs/phase9_runtime/<map>/vgl_config`，只在副本中把同步窗设为3000 µs，不修改地图或`/opt/ros`文件。

`vgl_pose_relay`在转发前执行平移和yaw创新检查。接受的位姿同时发布到`/visual_slam/initial_pose`和`/vgl_pose_relay/pose`；后者只供`navigation_tf_bridge`锁定全局锚点，不由VGL直接发布TF。每次`/localization/ready`从false变为true时重新计算一次锚点，正常导航期间保持固定，避免连续定位噪声进入局部控制。

失锁恢复时先等待前向遮挡清除，再调用：

```bash
ros2 service call /visual_localization/trigger_localization std_srvs/srv/Trigger '{}'
```

最多三次，接受位姿后还必须取得20个连续健康cuVSLAM状态才恢复Nav2。手动迁移到另一台电脑时，除本手册原有检查外，还要验证`/vgl_pose_relay/accepted`、`/localization/recovery_state`、`/localization/ready`和`/navigation/resilient_status`四个接口。
