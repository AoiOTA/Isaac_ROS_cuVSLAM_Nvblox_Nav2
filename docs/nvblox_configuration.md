# nvblox完整配置、迁移与调参手册

本文给出本项目阶段6在另一台电脑上从零手动配置Isaac ROS nvblox的方法。内容以本机实际安装并运行的Isaac ROS 4.5.0、ROS 2 Jazzy和`nvblox_ros` 4.5.0为准，不套用旧版参数名。

仓库中的权威运行文件是：

- 参数：[nvblox.yaml](../ros2_ws/src/nova_carter_bringup/config/nvblox.yaml)
- nvblox启动：[nvblox.launch.py](../ros2_ws/src/nova_carter_bringup/launch/nvblox.launch.py)
- 完整阶段6启动：[phase6.launch.py](../ros2_ws/src/nova_carter_bringup/launch/phase6.launch.py)
- 保存入口：[save_nvblox_map.sh](../scripts/save_nvblox_map.sh)
- 自动验收：[run_phase6_tests.sh](../scripts/run_phase6_tests.sh)

## 1. 已验证的软件组合和边界

本项目实际验证的组合是：

| 组件 | 版本/配置 |
|---|---|
| Ubuntu | 24.04.4 LTS，x86_64 |
| ROS 2 | Jazzy |
| Isaac ROS | 4.5.0 Debian包 |
| `isaac_ros_nvblox` | 4.5.0 |
| `nvblox_ros` | 4.5.0 |
| Isaac Sim | 6.0.1 Standalone Python |
| GPU | RTX 4090，24 GB |
| CUDA Toolkit | 13.0.3 |
| TensorRT | 10.13.3.9 |
| 深度 | Isaac Sim原生32FC1米制深度，640×400，30 Hz |
| 彩色 | 前左目RGB8，1280×800，30 Hz；nvblox最多5 Hz积分 |
| 位姿 | cuVSLAM发布`odom→base_link`，静态URDF发布相机外参 |
| nvblox模式 | `static_tsdf`，5 cm voxel，2D ESDF |
| 静态 nvblox 全局帧 | `map`（导航动态层另用 `odom`） |

阶段6不使用FoundationStereo、ESS、lidar、语义分割或仿真ground truth位姿。仿真真值深度只替代深度估计，机器人位姿仍来自cuVSLAM。

NVIDIA当前官方入口：

- [Isaac ROS Nvblox包](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html)
- [ROS参数](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/parameters.html)
- [ROS话题与服务](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/topics_and_services.html)
- [Isaac Sim示例](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_isaac_sim.html)

## 2. 数据链和TF所有权

阶段6的数据链为：

```text
Isaac Sim front Hawk left camera
  ├── 32FC1 depth + depth CameraInfo ─────────────┐
  └── RGB8 color + left CameraInfo ───────────────┤
                                                   v
cuVSLAM map→odom→base_link ── TF ─────────── nvblox_node
robot_state_publisher base_link→left_optical ──────┤
                                                   ├── TSDF layer
                                                   ├── color mesh
                                                   ├── 2D static ESDF pointcloud
                                                   ├── static_map_slice
                                                   ├── .nvblx map
                                                   └── .ply mesh
```

TF必须只有以下所有权：

| TF边 | 发布者 |
|---|---|
| `map→odom` | cuVSLAM |
| `odom→base_link` | cuVSLAM |
| `base_link→front_stereo_camera_link` | robot_state_publisher |
| `front_stereo_camera_link→front_stereo_camera_left_optical` | robot_state_publisher |

nvblox不发布机器人定位TF。静态建图按每个深度时间戳查询：

```text
map → odom → base_link → front_stereo_camera_left_optical
```

持久静态地图固定使用`global_frame=map`，使 TSDF、occupancy、cuVSLAM 和 cuVGL
处在同一全局解中。若在 `odom` 中保存静态图，局部轨迹会与回环优化后的视觉地图逐渐
分离。导航的有界动态 nvblox 使用独立的`nvblox_dynamic.yaml`，继续留在连续`odom`，
二者不能混为一份配置。

ground truth不得进入这棵TF树，否则测试看起来会更准，但系统已经不再是视觉定位。

## 3. 安装和发现包

另一台Ubuntu 24.04/Jazzy电脑先完成本项目[裸机环境配置](installation.md)，再安装：

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-isaac-ros-nvblox \
  ros-jazzy-nvblox-nav2 \
  ros-jazzy-nvblox-rviz-plugin \
  ros-jazzy-nav2-bringup \
  ros-jazzy-rviz2
```

新终端检查：

```bash
source /opt/ros/jazzy/setup.bash

ros2 pkg prefix isaac_ros_nvblox
ros2 pkg prefix nvblox_ros
ros2 pkg prefix nvblox_msgs
ros2 pkg prefix nvblox_nav2
ros2 pkg prefix nvblox_rviz_plugin

ros2 component types | grep -A2 nvblox_ros
ros2 interface show nvblox_msgs/msg/DistanceMapSlice
ros2 interface show nvblox_msgs/srv/FilePath
```

必须能看到组件：

```text
nvblox::NvbloxNode
nvblox::NvbloxHumanNode
```

本项目阶段6使用`nvblox::NvbloxNode`。`NvbloxHumanNode`和旧版本教程里的`nvblox_human_node`不属于静态TSDF基线。

若从源码安装，必须检出与Isaac ROS版本一致的`release-4.5`分支，不要把main、3.x或4.4的YAML与4.5二进制混用。

## 4. 输入数据契约

### 4.1 固定话题

| nvblox端口 | 项目话题 | 消息 | 要求 |
|---|---|---|---|
| `camera_0/depth/image` | `/front_stereo_camera/depth/image_raw` | `sensor_msgs/Image` | 640×400，`32FC1`，米 |
| `camera_0/depth/camera_info` | `/front_stereo_camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 640×400，与深度同一模型 |
| `camera_0/color/image` | `/front_stereo_camera/left/image_raw_rgb` | `sensor_msgs/Image` | 1280×800，`rgb8` |
| `camera_0/color/camera_info` | `/front_stereo_camera/left/camera_info` | `sensor_msgs/CameraInfo` | 1280×800 |

深度和彩色可以不同分辨率。每一类图像必须与自己的CameraInfo配对，不能把1280×800内参直接用于640×400深度。

### 4.2 深度单位和无效值

nvblox 4.5支持：

- `32FC1`：单位为米。
- OpenNI风格`16UC1`：单位为毫米。

本项目固定`32FC1`。有效深度必须是有限正数；`NaN`、`Inf`、0和负值应被当作无效。启动前抽检：

```bash
ros2 topic echo --once /front_stereo_camera/depth/image_raw \
  sensor_msgs/msg/Image --field encoding
ros2 topic echo --once /front_stereo_camera/depth/image_raw \
  sensor_msgs/msg/Image --field width
ros2 topic echo --once /front_stereo_camera/depth/camera_info \
  sensor_msgs/msg/CameraInfo
```

预期编码为`32FC1`、宽640、高400，深度消息和CameraInfo的`frame_id`都为`front_stereo_camera_left_optical`。

### 4.3 CameraInfo检查

至少检查：

```text
width > 0
height > 0
K[0] = fx > 0
K[4] = fy > 0
K[2] = cx在图像范围内
K[5] = cy在图像范围内
header.frame_id非空
```

如果把图像缩放一半，内参也必须按相同比例缩放：

```text
fx' = sx * fx
fy' = sy * fy
cx' = sx * cx
cy' = sy * cy
```

深度与CameraInfo尺寸不一致时，nvblox可能持续收到消息却无法正确反投影；这比完全没有消息更难诊断。

### 4.4 时间和QoS

所有节点统一：

```yaml
use_sim_time: true
input_qos: SENSOR_DATA
```

本项目Isaac Sim图像发布者为Best Effort，故`SENSOR_DATA`能匹配。若真实相机明确使用Reliable，可以改为`DEFAULT`或与驱动一致的配置，但必须通过：

```bash
ros2 topic info -v /front_stereo_camera/depth/image_raw
```

确认发布和订阅QoS兼容。

不要在一次运行中让`/clock`回退。每个正式实验使用全新的仿真进程。

## 5. 完整参数文件

将下面内容保存为包内`config/nvblox.yaml`。它与本仓库阶段6实际运行配置一致。

```yaml
nvblox_node:
  ros__parameters:
    use_sim_time: true

    # Persistent static reconstruction shares the optimized visual-map frame.
    mapping_type: static_tsdf
    global_frame: map
    num_cameras: 1
    use_tf_transforms: true
    use_topic_transforms: false
    use_depth: true
    use_color: true
    use_lidar: false
    use_segmentation: false

    # Five-centimetre voxels preserve the Jackal footprint and doorway margins.
    voxel_size: 0.05
    cuda_stream_type: 1
    tick_period_ms: 10
    maximum_input_queue_length: 30
    input_qos: SENSOR_DATA

    # Input and reconstruction rates. Non-positive values disable a stage.
    integrate_depth_rate_hz: 10.0
    integrate_color_rate_hz: 3.0
    integrate_lidar_rate_hz: 0.0
    update_esdf_rate_hz: 10.0
    update_mesh_rate_hz: 1.0
    publish_layer_rate_hz: 1.0
    publish_debug_vis_rate_hz: 1.0
    decay_tsdf_rate_hz: 0.0
    decay_dynamic_occupancy_rate_hz: 0.0
    clear_map_outside_radius_rate_hz: 0.0

    # Preserve the full static map collected during Phase 6.
    map_clearing_radius_m: -1.0
    map_clearing_frame_id: base_link

    # A 2D ESDF slice spanning Jackal's collision body feeds Nav2 later.
    esdf_mode: 2d
    publish_esdf_distance_slice: true
    output_pessimistic_distance_map: true
    esdf_slice_bounds_visualization_attachment_frame_id: base_link
    esdf_slice_bounds_visualization_side_length: 15.0

    # Keep visualization bounded without discarding data from the saved map.
    layer_visualization_min_tsdf_weight: 0.1
    layer_visualization_exclusion_height_m: 2.5
    layer_visualization_exclusion_radius_m: 15.0
    layer_streamer_bandwidth_limit_mbps: -1.0
    max_back_projection_distance: 8.0
    back_projection_subsampling: 2

    print_rates_to_console: false
    print_timings_to_console: false
    print_delays_to_console: false
    print_queue_drops_to_console: false
    print_statistics_on_console_period_ms: 10000

    multi_mapper:
      remove_small_connected_components: true
      connected_mask_component_size_threshold: 2000

    static_mapper:
      do_depth_preprocessing: false
      depth_preprocessing_num_dilations: 3

      # Native simulated depth is already metric and dense.
      projective_integrator_max_integration_distance_m: 8.0
      projective_integrator_truncation_distance_vox: 4.0
      projective_integrator_weighting_mode: inverse_square_tsdf_distance_penalty
      projective_integrator_max_weight: 5.0
      projective_tsdf_integrator_invalid_depth_decay_factor: -1.0
      raycast_subsampling_factor: 2

      # Exclude very high warehouse geometry while retaining floor-to-body obstacles.
      workspace_bounds_type: height_bounds
      workspace_bounds_min_height_m: -0.20
      workspace_bounds_max_height_m: 2.00

      esdf_slice_height: 0.09
      esdf_slice_min_height: 0.09
      esdf_slice_max_height: 0.65
      esdf_integrator_min_weight: 0.1
      esdf_integrator_max_site_distance_vox: 2.0
      esdf_integrator_max_distance_m: 2.0
      unobserved_esdf_policy: ignore
      add_negative_truncation_band_sites: false

      mesh_integrator_min_weight: 0.1
      mesh_integrator_weld_vertices: true
      layer_streamer_exclusion_height_m: 2.5
      layer_streamer_exclusion_radius_m: 15.0
```

### 5.1 参数文件作用域

顶层必须是节点名`nvblox_node`，启动中的组件名也必须一致。若使用通配符`/**`，容易让同一容器内其他节点误收参数；本项目不使用通配符。

### 5.2 参数命名版本差异

4.5使用：

```text
maximum_input_queue_length
layer_streamer_bandwidth_limit_mbps
static_mapper.layer_streamer_exclusion_radius_m
esdf_slice_bounds_visualization_attachment_frame_id
```

不要复制旧配置中的：

```text
maximum_sensor_message_queue_length
mesh_bandwidth_limit_mbps
mesh_streamer_exclusion_radius_m
slice_visualization_attachment_frame_id
```

启动后以实际参数为准：

```bash
ros2 param dump /nvblox_node > /tmp/nvblox-runtime.yaml
grep -E 'mapping_type|global_frame|voxel_size|integrate_|update_' \
  /tmp/nvblox-runtime.yaml
```

## 6. 完整launch组织

将下面内容保存为`launch/nvblox.launch.py`：

```python
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("nova_carter_bringup"))
    config = LaunchConfiguration("nvblox_config")
    nvblox = ComposableNode(
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        name="nvblox_node",
        parameters=[config],
        remappings=[
            ("camera_0/depth/image", "/front_stereo_camera/depth/image_raw"),
            (
                "camera_0/depth/camera_info",
                "/front_stereo_camera/depth/camera_info",
            ),
            ("camera_0/color/image", "/front_stereo_camera/left/image_raw_rgb"),
            (
                "camera_0/color/camera_info",
                "/front_stereo_camera/left/camera_info",
            ),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "nvblox_config",
                default_value=str(share / "config/nvblox.yaml"),
                description="Absolute path to the nvblox parameter file.",
            ),
            ComposableNodeContainer(
                name="nvblox_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[nvblox],
            ),
        ]
    )
```

在`package.xml`至少声明：

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>nvblox_msgs</exec_depend>
<exec_depend>nvblox_ros</exec_depend>
<exec_depend>rclcpp_components</exec_depend>
```

Python包的`setup.py`必须安装参数和launch：

```python
(f"share/{package_name}/config", glob("config/*.yaml")),
(f"share/{package_name}/launch", glob("launch/*.launch.py")),
```

## 7. 构建和干净终端启动

构建：

```bash
cd /absolute/path/to/Isaac_ROS_cuVSLAM_Nvblox_Nav2
source /opt/ros/jazzy/setup.bash
colcon build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --symlink-install
```

终端A启动仿真：

```bash
cd /absolute/path/to/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/run_sim.sh --headless
```

终端B启动控制、TF、cuVSLAM和nvblox：

```bash
cd /absolute/path/to/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/run_phase6.sh
```

只启动nvblox时：

```bash
./scripts/run_nvblox.sh
```

此时必须由别的launch已经提供`/clock`、相机、TF和cuVSLAM。单独启动nvblox并不等于建图链已经成立。

本项目脚本会显式source系统ROS和overlay，并固定ROS domain、RMW和本机发现范围。不要依赖某个终端残留的环境变量。

## 8. 启动后的逐层验证

### 8.1 节点、组件和参数

```bash
ros2 node list | sort
ros2 component list
ros2 param get /nvblox_node mapping_type
ros2 param get /nvblox_node global_frame
ros2 param get /nvblox_node voxel_size
ros2 param get /nvblox_node input_qos
ros2 param get /nvblox_node static_mapper.esdf_slice_min_height
ros2 param get /nvblox_node static_mapper.esdf_slice_max_height
```

期望分别为`static_tsdf`、`odom`、约`0.05`、`SENSOR_DATA`、`0.09`和`0.65`。

### 8.2 输入连接

```bash
ros2 topic info -v /front_stereo_camera/depth/image_raw
ros2 topic info -v /front_stereo_camera/depth/camera_info
ros2 topic info -v /front_stereo_camera/left/image_raw_rgb
ros2 topic info -v /front_stereo_camera/left/camera_info
```

每个话题必须同时有Isaac Sim发布者和`/nvblox_node`订阅者。只有CameraInfo连接而没有Image连接通常是QoS不兼容或remap错误。

### 8.3 TF按消息时间可用

```bash
ros2 run tf2_ros tf2_echo odom front_stereo_camera_left_optical
```

机器人运动时平移和旋转应连续更新。若只能查到`base_link→camera`，但没有`odom→base_link`，先修cuVSLAM，不要调整nvblox TSDF参数。

### 8.4 输出接口

4.5实测输出：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/nvblox_node/tsdf_layer` | `nvblox_msgs/VoxelBlockLayer` | TSDF可视化数据 |
| `/nvblox_node/color_layer` | `nvblox_msgs/VoxelBlockLayer` | 颜色体素 |
| `/nvblox_node/mesh` | `nvblox_msgs/Mesh` | 增量网格 |
| `/nvblox_node/static_esdf_pointcloud` | `sensor_msgs/PointCloud2` | 静态2D ESDF点云 |
| `/nvblox_node/static_map_slice` | `nvblox_msgs/DistanceMapSlice` | Nav2静态代价切片 |
| `/nvblox_node/pessimistic_static_map_slice` | `nvblox_msgs/DistanceMapSlice` | 未观测区域保守处理的切片 |

不要按旧文档假定`tsdf_layer`一定是`PointCloud2`；先执行：

```bash
ros2 topic list -t | grep /nvblox_node
```

检查流量：

```bash
timeout 15 ros2 topic hz /nvblox_node/mesh
timeout 15 ros2 topic hz /nvblox_node/static_map_slice
timeout 15 ros2 topic hz /nvblox_node/static_esdf_pointcloud
ros2 topic echo --once /nvblox_node/static_map_slice \
  nvblox_msgs/msg/DistanceMapSlice --field header
```

所有输出`header.frame_id`应为`odom`。

大型TSDF、Mesh和ESDF话题会产生明显序列化与DDS负载。诊断时一次只观察必要话题；不要同时长时间`echo`所有大型输出。

## 9. 保存、加载和统计

项目一键保存：

```bash
./scripts/save_nvblox_map.sh \
  /absolute/path/to/data/maps/warehouse_v1/nvblox \
  warehouse
```

输出：

```text
warehouse.nvblx
warehouse.ply
nvblox_rates.txt
nvblox_timings.txt
save_report.json
```

等价手动调用：

```bash
ros2 service call /nvblox_node/save_map nvblox_msgs/srv/FilePath \
  "{file_path: '/absolute/path/warehouse.nvblx'}"

ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath \
  "{file_path: '/absolute/path/warehouse.ply'}"

ros2 service call /nvblox_node/save_rates nvblox_msgs/srv/FilePath \
  "{file_path: '/absolute/path/nvblox_rates.txt'}"

ros2 service call /nvblox_node/save_timings nvblox_msgs/srv/FilePath \
  "{file_path: '/absolute/path/nvblox_timings.txt'}"
```

加载会覆盖节点当前地图：

```bash
ros2 service call /nvblox_node/load_map nvblox_msgs/srv/FilePath \
  "{file_path: '/absolute/path/warehouse.nvblx'}"
```

加载前必须确认`voxel_size`、mapping type、全局帧策略和软件大版本与保存时一致。阶段6自动验收实际测试了保存；加载会在阶段7地图工作流中做端到端验收。

## 10. 2D ESDF的关键服务限制

阶段6固定：

```yaml
esdf_mode: 2d
```

在nvblox 4.5中，`/nvblox_node/get_esdf_and_gradient`只用于3D ESDF。对2D配置调用该服务会输出FATAL并结束nvblox进程。本机已经实际触发并确认该行为。

因此2D模式用以下接口验收：

```text
/nvblox_node/static_esdf_pointcloud
/nvblox_node/static_map_slice
```

只有明确改成：

```yaml
esdf_mode: 3d
```

并重新设计内存、更新率和消费者后，才调用`get_esdf_and_gradient`。不能把“服务已列出”误认为“当前模式可安全调用”。

## 11. 参数调优顺序

一次只改一组参数，并为每次实验保存运行时参数、rates、timings、地图尺寸和路径结果。推荐顺序如下。

### 11.1 先冻结输入和TF

在动TSDF参数之前确认：

1. 深度编码和单位正确。
2. 深度CameraInfo与深度尺寸匹配。
3. 深度帧与TF中的光学帧完全同名。
4. `/clock`不回退。
5. cuVSLAM持续跟踪。
6. nvblox没有`Could not transform`。

输入或TF错误时，调高队列、权重和截断距离不会修复地图。

### 11.2 voxel大小

`voxel_size`决定精度、显存、计算量和文件大小。

| voxel | 用途 | 代价 |
|---:|---|---|
| 0.10 m | 快速冒烟、低端GPU | 小障碍和边缘粗糙 |
| 0.05 m | 本项目基线 | 精度和实时性的平衡 |
| 0.03 m | 精细货架腿/窄障碍 | 计算、显存、文件显著增加 |
| 0.01–0.02 m | 特殊精细重建 | 不适合直接作为当前导航基线 |

改变voxel后必须重新建图。旧`.nvblx`不能当作等价结果继续使用。

### 11.3 深度积分距离

```yaml
static_mapper.projective_integrator_max_integration_distance_m: 8.0
```

- 降低：减少远处噪声和运算，视野短。
- 提高：更早看到远处货架，但深度误差和块数量增加。

真实相机先从5 m开始；仿真真值深度可用8 m。不要超过传感器有效量程。

### 11.4 截断距离和权重

```yaml
projective_integrator_truncation_distance_vox: 4.0
projective_integrator_weighting_mode: inverse_square_tsdf_distance_penalty
projective_integrator_max_weight: 5.0
```

截断带物理厚度约为：

```text
truncation_distance = voxel_size × truncation_distance_vox
                    = 0.05 × 4 = 0.20 m
```

- 表面断裂：可小幅增大截断带或检查深度空洞。
- 表面过厚、窄间隙被抹平：减小截断带。
- 地图难以被新观测修正：降低最大权重。
- 静态表面抖动：提高最大权重，但不要掩盖位姿漂移。

### 11.5 raycast下采样

```yaml
static_mapper.raycast_subsampling_factor: 2
```

- `1`：最高投影密度，计算最大。
- `2`：RTX 4090基线。
- `4`：官方常见默认，适合性能优先。

先看`nvblox_timings.txt`中的TSDF积分时间，再决定是否降低精度。

### 11.6 积分和输出频率

```yaml
integrate_depth_rate_hz: 30.0
integrate_color_rate_hz: 5.0
update_esdf_rate_hz: 10.0
update_mesh_rate_hz: 1.0
publish_layer_rate_hz: 1.0
```

这些是目标上限，不保证在所有负载下达到。真实速率以`save_rates`输出为准。

导航优先级通常为：

```text
深度积分 > ESDF更新 > cuVSLAM > 彩色积分 > Mesh/TSDF可视化
```

系统过载时按以下顺序降载：

1. 关闭不必要的RViz大型显示。
2. 降低`publish_layer_rate_hz`和Mesh显示频率。
3. 降低彩色积分频率或`use_color=false`。
4. 把raycast下采样从2改为4。
5. 降低ESDF更新率到5 Hz。
6. 最后才降低深度积分率。

不要通过增大cuVSLAM图像抖动阈值来掩盖系统过载。

### 11.7 ESDF高度带

```yaml
esdf_slice_min_height: 0.09
esdf_slice_max_height: 0.65
esdf_slice_height: 0.09
```

高度在`global_frame`中解释。本项目用Nova Carter官方仿真基线，覆盖轮体可能碰撞的低矮障碍。

- 切片漏掉箱体：提高`max_height`或检查坐标原点。
- 地面被当作障碍：提高`min_height`，同时确认base/odom的Z定义。
- 高架货物错误阻塞：降低`max_height`。

先在RViz叠加RobotModel、TF、Mesh和ESDF，确认高度语义，再调Nav2 inflation。

### 11.8 ESDF距离范围和未知区域

```yaml
esdf_integrator_max_distance_m: 2.0
unobserved_esdf_policy: ignore
output_pessimistic_distance_map: true
```

2 m足以覆盖局部规划的避障梯度。增大距离范围会扩大计算和消息。

本项目同时输出普通和pessimistic切片。后续Nav2选择哪个，必须与“未知区域是否允许通行”的任务策略一致。

### 11.9 workspace高度界限

```yaml
workspace_bounds_type: height_bounds
workspace_bounds_min_height_m: -0.20
workspace_bounds_max_height_m: 2.00
```

它限制被积分的全局高度，避免仓库天花板等不参与地面导航的几何占用资源。若机器人上下坡、电梯或多层运行，应重新设计，不能照搬固定高度。

### 11.10 队列和QoS

```yaml
maximum_input_queue_length: 30
input_qos: SENSOR_DATA
```

- 队列过小：TF稍晚到达时更容易丢帧。
- 队列过大：延迟和内存增加，过期帧仍排队。
- Best Effort发布者+Reliable订阅者：可能完全不匹配。

启用诊断：

```yaml
print_rates_to_console: true
print_timings_to_console: true
print_delays_to_console: true
print_queue_drops_to_console: true
```

诊断完成后关闭，避免高频日志本身影响调度。

### 11.11 地图清理和保存范围

阶段6：

```yaml
map_clearing_radius_m: -1.0
clear_map_outside_radius_rate_hz: 0.0
```

用于保留完整静态地图。长时间在线局部导航若显存持续增长，可启用半径清理，例如15 m；但被清理区域不会出现在最终保存地图中。

`layer_visualization_exclusion_radius_m`只限制流出的可视化范围，不等同于删除内部地图。

### 11.12 深度预处理

仿真真值深度固定：

```yaml
do_depth_preprocessing: false
```

真实深度有孔洞时可尝试开启并设置膨胀次数。它可能填补小孔，也可能让薄障碍变厚；必须用已知尺寸障碍做A/B测试。

## 12. 从静态模式扩展到动态模式

阶段6不要提前切换。阶段9动态基线将在静态流程稳定后使用：

```yaml
mapping_type: dynamic
decay_dynamic_occupancy_rate_hz: 10.0
```

并增加`dynamic_mapper`占用概率、衰减概率及freespace参数。动态模式输出：

```text
dynamic_map_slice
combined_map_slice
dynamic_esdf_pointcloud
combined_esdf_pointcloud
```

Nav2局部代价地图届时从`static_map_slice`切换到`combined_map_slice`。不能只改一个topic而不验证动态occupancy decay，否则移动障碍离开后可能长期留下“鬼影”。

## 13. Nav2接入模板

阶段8局部代价地图使用：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      resolution: 0.05
      plugins: [nvblox_layer, inflation_layer]
      nvblox_layer:
        plugin: nvblox::nav2::NvbloxCostmapLayer
        enabled: true
        nav2_costmap_global_frame: odom
        nvblox_map_slice_topic: /nvblox_node/static_map_slice
        convert_to_binary_costmap: true
```

三处必须一致：

```text
nvblox global_frame
local_costmap global_frame
nvblox_layer nav2_costmap_global_frame
```

本项目都是`odom`。阶段6只保证切片可用，完整Nav2插件验收属于阶段8。

## 14. 故障诊断矩阵

| 现象 | 首要检查 | 常见修复 |
|---|---|---|
| 节点存在但地图全空 | 深度订阅、CameraInfo、TF | 修remap/QoS/frame_id |
| 反复`Could not transform` | 时间戳对应的`odom→optical` | 修cuVSLAM/静态TF/队列 |
| 地图尺度错误 | 深度单位、CameraInfo内参 | 32FC1用米，16UC1用毫米 |
| 地面整片为障碍 | ESDF高度、odom Z、光学帧方向 | 重测高度带和TF |
| Mesh撕裂/重影 | 位姿跳变、深度同步 | 先修定位，不先增权重 |
| TSDF有数据但ESDF空 | `esdf_mode`、高度带、min weight | 检查slice min/max和权重 |
| map slice全是unknown | 视野、ESDF高度、TF | 查看Mesh和ESDF点云定位层级 |
| 内存持续增长 | map clearing关闭、voxel太小 | 启用半径清理或增大voxel |
| DDS/CPU负载高 | 同时订阅大型可视化话题 | 降发布率、关闭多余RViz显示 |
| cuVSLAM出现大量图像间隔警告 | GPU/DDS竞争、测试观察负载 | 先降可视化/彩色/ESDF负载 |
| 保存返回false | 目录权限、绝对路径、磁盘 | 预建目录并检查空间 |
| 调用ESDF服务后进程退出 | 2D模式调用了3D专用服务 | 2D只用pointcloud和map slice |
| 加载地图后表现异常 | 版本/voxel/mapping type不一致 | 使用同版本同配置重新加载 |

建议一次性收集：

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t | grep nvblox
ros2 param dump /nvblox_node
ros2 topic info -v /front_stereo_camera/depth/image_raw
ros2 run tf2_ros tf2_echo odom front_stereo_camera_left_optical
nvidia-smi
df -h
```

## 15. 自动验收与当前实测基线

完整验收：

```bash
./scripts/run_phase6_tests.sh
```

脚本自动完成：

1. 使用隔离ROS domain启动自己管理的Isaac Sim进程。
2. 加载Warehouse和Nova Carter主USD。
3. 启动阶段3控制、阶段5 cuVSLAM和阶段6 nvblox。
4. 审计运行时参数，而不是只检查YAML文本。
5. 以20 Hz控制执行60秒双向S形扫描。
6. 验证CameraInfo流、cuVSLAM状态和仿真时钟。
7. 采样TSDF、Mesh、静态ESDF和map slice，取得证据后主动退订大型话题，避免观察者反压。
8. 保存`.nvblx`、`.ply`、rates和timings。
9. 检查仿真无意外PhysX碰撞、官方USD未修改、进程干净退出。

2026-07-17最终本机实测：

| 指标 | 结果 |
|---|---:|
| 运动轨迹 | 15.48 m |
| cuVSLAM状态 | 495个健康状态，0失锁 |
| 深度回调 | 19.7 Hz |
| 深度积分 | 14.9 Hz |
| 彩色积分 | 3.9 Hz |
| ESDF更新 | 8.7 Hz |
| TSDF采样唯一块 | 7,431 |
| Mesh采样唯一块 | 1,452 |
| Mesh单消息最大顶点 | 67,548 |
| Mesh单消息最大三角形 | 100,081 |
| ESDF/切片最大已知单元 | 21,533 |
| 保存地图 | 104,607,744字节 |
| 保存PLY | 14,769,575字节 |
| TF/输出帧 | 全部`odom` |
| 意外PhysX碰撞 | 0 |

配置中的10/3/10 Hz是处理上限；实际部署应以目标电脑的`nvblox_rates.txt`为准。

详细证据见[阶段6验证报告](phase6_validation.md)。

## 16. 迁移到另一台电脑的最终检查表

- [ ] Ubuntu、ROS、Isaac ROS、CUDA和驱动版本相容。
- [ ] `nvblox::NvbloxNode`可被组件容器发现。
- [ ] 深度是32FC1米或16UC1毫米之一，单位明确。
- [ ] 深度CameraInfo与深度分辨率匹配。
- [ ] 彩色CameraInfo与彩色分辨率匹配。
- [ ] 四个输入remap与实际相机话题一致。
- [ ] 图像发布/订阅QoS兼容。
- [ ] 所有节点使用同一个时钟策略。
- [ ] `map→odom→base_link→camera_optical`在图像时间戳可查。
- [ ] 持久静态 reconstruction 输出 `map`，导航动态层另用 `odom`。
- [ ] runtime param dump与预期一致。
- [ ] TSDF、Mesh、ESDF和map slice全部非空。
- [ ] 静态建图输出 frame 全部为`map`。
- [ ] 2D模式不调用3D专用ESDF查询服务。
- [ ] map、PLY、rates和timings能写入绝对路径。
- [ ] rates达到目标电脑可接受的实时门槛。
- [ ] 长轨迹中cuVSLAM没有失锁，地图没有明显重影。
- [ ] 修改voxel、mapping type或高度带后重新建图。

完成这些检查后，才进入阶段7的自动地图集合和阶段8的Nav2代价地图接入。

## 17. 阶段9 dynamic mapper配置

阶段9使用`config/nvblox_dynamic.yaml`和`nvblox_dynamic.launch.py`，输入仍只有前向左目对齐的32FC1真值深度、CameraInfo和颜色。关键差异为：

```yaml
nvblox_node:
  ros__parameters:
    mapping_type: dynamic
    integrate_depth_rate_hz: 10.0
    integrate_color_rate_hz: 2.0
    decay_dynamic_occupancy_rate_hz: 10.0
    publish_layer_rate_hz: 10.0
    dynamic_mapper:
      projective_integrator_max_integration_distance_m: 5.0
      occupied_region_half_width_m: 0.15
      esdf_slice_min_height: 0.09
      esdf_slice_max_height: 0.65
      free_region_decay_probability: 0.55
      occupied_region_decay_probability: 0.30
```

阶段9必须观测到以下非空输出：

```text
/nvblox_node/dynamic_map_slice
/nvblox_node/combined_map_slice
/nvblox_node/dynamic_esdf_pointcloud
/nvblox_node/combined_esdf_pointcloud
/nvblox_node/dynamic_points
```

Nav2局部NvbloxCostmapLayer改接`combined_map_slice`，而不是阶段6/8的`static_map_slice`。`global_frame`、局部costmap和插件frame仍全部为`odom`。不要为了看到动态点云而把dynamic occupancy decay关闭；移动障碍离开后必须自动清除，否则局部地图会形成永久幽灵障碍。

完整验证入口为：

```bash
./scripts/run_phase9.sh --map warehouse_v2_front --headless --rviz --auto
```

报告必须同时记录dynamic slice、dynamic ESDF、combined slice和combined ESDF的非零元素，并验证Robot与动态物体PhysX接触数为0。完整结果见[阶段9验证记录](phase9_validation.md)。
