# cuVSLAM完整配置、迁移与调参手册

本文记录本仓库阶段5实际跑通的cuVSLAM配置过程，并给出在另一台电脑上从干净终端手动复现、替换传感器和逐步调参的方法。目标版本是：

| 组件 | 已验证版本 |
|---|---|
| Ubuntu | 24.04 Noble |
| ROS 2 | Jazzy |
| Isaac ROS | 4.5.0 |
| `isaac_ros_visual_slam` | 4.5.0 |
| cuVSLAM运行库 | 15.0.0 |
| Isaac Sim | 6.0.1 |
| CUDA Toolkit | 13.0.3 |
| TensorRT | 10.13.3.9 |
| GPU | RTX 4090 |

本文有两种使用方式：

1. **复制本仓库复现Nova Carter仿真：** 使用本文中的固定参数，无需重新编写launch。
2. **迁移到其他双目相机或真实机器人：** 复用节点组织方式，但必须重新确认相机内外参、图像是否真正校正、IMU坐标系和IMU噪声参数。

不要把“节点能够启动”当作配置完成。cuVSLAM可以在相机模型错误时仍报告`vo_state=1`，但输出错误的尺度。本项目曾实测到这种情况：错误输入的1秒平移尺度达到真实值的2.41–3.30倍。

官方参考：

- [Isaac ROS Visual SLAM API与参数](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html)
- [Isaac Sim cuVSLAM教程与地图服务](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/tutorial_isaac_sim.html)
- [cuVSLAM真实机器人验证方法](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/validating_cuvslam_setup.html)
- [RealSense双目+IMU示例](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/tutorial_realsense.html)

## 1. 本项目的数据流和TF所有权

实际数据流如下：

```text
Isaac Sim front Hawk left/right RGB
  -> /front_stereo_camera/{left,right}/image_raw_rgb
  -> Isaac ROS ImageFormatConverterNode x2
  -> /front_stereo_camera/{left,right}/image_raw (mono8)
  -> cuVSLAM

Isaac Sim stereo CameraInfo -----------------------> cuVSLAM
Isaac Sim front Hawk IMU --------------------------> cuVSLAM

cuVSLAM
  -> /visual_slam/status
  -> /visual_slam/tracking/odometry
  -> map -> odom -> base_link
  -> save/load/localize services
```

固定TF所有权：

```text
map                         cuVSLAM发布
└── odom                    cuVSLAM发布
    └── base_link           cuVSLAM发布
        ├── front_stereo_camera_link
        │   ├── front_stereo_camera_left_optical
        │   ├── front_stereo_camera_right_optical
        │   └── front_stereo_camera_imu
        └── wheels/casters  robot_state_publisher发布
```

规则：

- cuVSLAM是`map→odom`和`odom→base_link`的唯一发布者。
- `robot_state_publisher`只发布`base_link`以下的刚性/关节TF。
- wheel odometry和仿真ground truth都不发布主TF。
- VGL以后只向cuVSLAM提供位姿提示，不直接争用主TF。

## 2. 另一台电脑的环境准备

### 2.1 最省事的复现方式

复制或克隆仓库后，先修改[`config/environment.env`](../config/environment.env)中的Isaac Sim和资产路径，然后执行：

```bash
cd ~/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/bootstrap_baremetal.sh --preflight
./scripts/build.sh
```

如果另一台电脑尚未安装Isaac ROS，严格按照[`docs/installation.md`](installation.md)配置NVIDIA APT源、CUDA、TensorRT和Isaac ROS裸机环境。不要在尚未配置Isaac ROS软件源的Ubuntu上直接执行下面的APT安装命令。

### 2.2 cuVSLAM最小软件包

Isaac ROS源已经配置后，最小相关包是：

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-isaac-ros-visual-slam \
  ros-jazzy-isaac-ros-image-proc \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros \
  ros-jazzy-xacro \
  ros-jazzy-rmw-fastrtps-cpp
```

在干净终端验证实际版本：

```bash
source /opt/ros/jazzy/setup.bash

ros2 pkg prefix isaac_ros_visual_slam
ros2 pkg prefix isaac_ros_visual_slam_interfaces
grep '<version>' \
  "$(ros2 pkg prefix isaac_ros_visual_slam --share)/package.xml" | head -1
cat "$(ros2 pkg prefix isaac_ros_visual_slam --share)/version_info.yaml"
nvidia-smi
```

本项目应看到包前缀`/opt/ros/jazzy`和版本`4.5.0`。如果版本不同，先查看该版本的官方参数文档，不要假设所有参数和默认值不变。

### 2.3 每个终端必须一致的ROS环境

本项目脚本会自动设置这些变量。手动运行时，每个终端都要使用相同值：

```bash
source /opt/ros/jazzy/setup.bash
source ~/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2/config/fastdds.xml"
```

跨电脑ROS通信时不能设置`ROS_LOCALHOST_ONLY=1`。此时应删除该变量，并配置相同的`ROS_DOMAIN_ID`、可互通的网络和DDS发现策略。

## 3. cuVSLAM输入必须满足的完整契约

### 3.1 本项目固定输入

| 输入 | 消息类型 | 编码/频率 | `frame_id` |
|---|---|---|---|
| `/front_stereo_camera/left/image_raw` | `sensor_msgs/Image` | `mono8`, 1280×800, 30 Hz源频率 | `front_stereo_camera_left_optical` |
| `/front_stereo_camera/right/image_raw` | `sensor_msgs/Image` | `mono8`, 1280×800, 30 Hz源频率 | `front_stereo_camera_right_optical` |
| `/front_stereo_camera/left/camera_info` | `sensor_msgs/CameraInfo` | 与左图同时间基准 | 左光学帧 |
| `/front_stereo_camera/right/camera_info` | `sensor_msgs/CameraInfo` | 与右图同时间基准 | 右光学帧 |
| `/front_stereo_imu/imu` | `sensor_msgs/Imu` | 60 Hz | `front_stereo_camera_imu` |
| `/clock` | `rosgraph_msgs/Clock` | 60 Hz | 无 |

QoS固定使用`SENSOR_DATA`，即Best Effort传感器数据语义。图像必须是未压缩ROS Image，不要直接把`CompressedImage`或H.264流remap给cuVSLAM。

### 3.2 双目同步

必须满足：

- 同一对左右图像时间戳相同，或差值小于`sync_matching_threshold_ms`。
- CameraInfo使用相同仿真/设备时钟，时间不能回退。
- 左右图的宽、高、编码和发布频率一致。
- 硬件相机优先使用硬件同步，不要依靠两个独立软件定时器“凑同步”。

检查命令：

```bash
ros2 topic type /front_stereo_camera/left/image_raw
ros2 topic hz --window 100 /front_stereo_camera/left/image_raw
ros2 topic hz --window 100 /front_stereo_camera/right/image_raw
ros2 topic echo /front_stereo_camera/left/image_raw --once \
  --field header
ros2 topic echo /front_stereo_camera/right/image_raw --once \
  --field header
```

如果日志出现：

```text
Delta between current and previous frame [...] is above threshold [...]
```

这表示源数据掉帧或到达间隔过大。提高`image_jitter_threshold_ms`只会改变告警阈值，不会修复传输、渲染或同步问题。

### 3.3 CameraInfo、校正和尺度

这是最重要的检查。`rectified_images: true`表示传入像素必须已经符合CameraInfo给出的校正后针孔模型，不只是把参数设为`true`。

对单组水平双目，右CameraInfo应满足：

```text
baseline_m = -P_right[3] / P_right[0]
```

本项目实测：

```text
P_right[0] = 639.0460343210154
P_right[3] = -95.85690514815236
baseline     = 0.15 m
```

检查完整CameraInfo：

```bash
ros2 topic echo /front_stereo_camera/left/camera_info --once
ros2 topic echo /front_stereo_camera/right/camera_info --once
```

至少确认：

- `K[0]`、`K[4]`、主点和图像尺寸合理。
- `P_right[3]`不为零，计算出的baseline与机械/TF测量一致。
- 左右`R`与实际校正过程一致。
- 如果声称图像已校正，畸变应已经从像素中去除。
- CameraInfo的光学帧必须能通过静态TF连接到`base_link`。

#### Isaac Sim 6.0.1 Nova Carter Hawk特殊修复

官方Nova Carter Hawk相机USD使用旧的`fisheyePolynomial`和rational distortion。Isaac Sim 6.0.1会渲染畸变，但ROS CameraInfo helper在本机实测发布`plumb_bob`且`D`全零。直接设置`rectified_images=true`会产生严重尺度错误。

本项目在[`config/sensors.yaml`](../config/sensors.yaml)中设置：

```yaml
front_stereo:
  navigation_projection: pinhole
```

并由[`isaac_sim/nova_carter_sim/sensors.py`](../isaac_sim/nova_carter_sim/sensors.py)在匿名session layer中：

1. 把前向左右相机的`cameraProjectionType`设为`pinhole`。
2. 把`physicalDistortionCoefficients`清零。
3. 再创建render product和ROS发布图。

这不会保存组合stage，也不会修改官方USD。其他Isaac Sim资产不能盲目套用该修复；先比较实际渲染模型和CameraInfo。

### 3.4 光学帧和静态TF

ROS光学帧约定：

- `+z`朝镜头视线前方。
- `+x`朝图像右方。
- `+y`朝图像下方。

本项目使用[`nova_carter.urdf.xacro`](../ros2_ws/src/nova_carter_bringup/urdf/nova_carter.urdf.xacro)，双目baseline为0.15 m。启动前检查：

当前Nova Carter前传感器静态链是：

```xml
<joint name="front_stereo_mount" type="fixed">
  <parent link="base_link"/>
  <child link="front_stereo_camera_link"/>
  <origin xyz="0.1003 -0.000002 0.3459" rpy="0 0 0"/>
</joint>
<joint name="front_stereo_left_optical_joint" type="fixed">
  <parent link="front_stereo_camera_link"/>
  <child link="front_stereo_camera_left_optical"/>
  <origin xyz="0 0.075 0" rpy="-1.57079632679 0 -1.57079632679"/>
</joint>
<joint name="front_stereo_right_optical_joint" type="fixed">
  <parent link="front_stereo_camera_link"/>
  <child link="front_stereo_camera_right_optical"/>
  <origin xyz="0 -0.075 0" rpy="-1.57079632679 0 -1.57079632679"/>
</joint>
<joint name="front_stereo_imu_joint" type="fixed">
  <parent link="front_stereo_camera_link"/>
  <child link="front_stereo_camera_imu"/>
  <origin xyz="0 -0.0197 0.0061" rpy="0 1.57079632679 0"/>
</joint>
```

这些数值只适用于官方Nova Carter资产。其他机器人必须用实际标定值替换。

```bash
ros2 run tf2_ros tf2_echo \
  base_link front_stereo_camera_left_optical
ros2 run tf2_ros tf2_echo \
  base_link front_stereo_camera_right_optical
ros2 run tf2_ros tf2_echo \
  base_link front_stereo_camera_imu
```

典型错误：

- 左右相机名称交换。
- baseline方向或单位错误。
- 把普通camera frame当成optical frame。
- 相机外参来自CAD，但实际安装方向不同。
- 同一静态边同时由Isaac Sim和robot_state_publisher发布。

### 3.5 IMU输入和噪声模型

VIO模式要求：

- `tracking_mode: 1`。
- IMU时间戳与图像处于同一时钟域且单调。
- `frame_id`与`imu_frame`一致。
- `base_link→imu_frame`静态TF准确。
- 静止时加速度模长约为9.81 m/s²。
- 静止时四元数有限且模长接近1；角速度接近0。
- 旋转时角速度轴向和正负号符合TF约定。

检查：

```bash
ros2 topic type /front_stereo_imu/imu
ros2 topic hz --window 200 /front_stereo_imu/imu
ros2 topic echo /front_stereo_imu/imu --once
```

当前仿真参数：

```yaml
gyro_noise_density: 0.000244
gyro_random_walk: 0.000019393
accel_noise_density: 0.001862
accel_random_walk: 0.003
calibration_frequency: 60.0
```

这组噪声值来自Isaac ROS示例初值，并在当前仿真中验证可用。真实IMU必须使用厂家数据手册或静止bag/Allan variance重新估计，`calibration_frequency`必须填写生成噪声参数时使用的IMU频率。不要用调整噪声参数掩盖错误的IMU外参或时间戳。

## 4. 创建ROS包配置

### 4.1 包依赖

本项目的bringup包至少需要：

```xml
<exec_depend>ament_index_python</exec_depend>
<exec_depend>isaac_ros_image_proc</exec_depend>
<exec_depend>isaac_ros_visual_slam</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>rclcpp_components</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>xacro</exec_depend>
```

`setup.py`必须把配置、launch和Xacro安装到share目录：

```python
data_files=[
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/config", glob("config/*.yaml")),
    (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
]
```

### 4.2 本项目完整参数文件

实际文件是[`visual_slam.yaml`](../ros2_ws/src/nova_carter_bringup/config/visual_slam.yaml)：

```yaml
visual_slam_node:
  ros__parameters:
    use_sim_time: true

    tracking_mode: 1
    num_cameras: 2
    min_num_images: 2
    camera_optical_frames:
      - front_stereo_camera_left_optical
      - front_stereo_camera_right_optical
    imu_frame: front_stereo_camera_imu
    base_frame: base_link
    map_frame: map
    odom_frame: odom

    rectified_images: true
    enable_image_denoising: false
    enable_localization_n_mapping: true
    multicam_mode: 1
    sync_matching_threshold_ms: 5.0
    image_jitter_threshold_ms: 34.0
    imu_jitter_threshold_ms: 10.0
    image_buffer_size: 30
    imu_buffer_size: 400
    image_qos: SENSOR_DATA
    imu_qos: SENSOR_DATA

    gyro_noise_density: 0.000244
    gyro_random_walk: 0.000019393
    accel_noise_density: 0.001862
    accel_random_walk: 0.003
    calibration_frequency: 60.0

    publish_map_to_odom_tf: true
    publish_odom_to_base_tf: true
    invert_map_to_odom_tf: false
    invert_odom_to_base_tf: false
    override_publishing_stamp: false

    enable_ground_constraint_in_odometry: true
    enable_ground_constraint_in_slam: true
    slam_max_map_size: 4000
    slam_throttling_time_ms: 500
    save_map_folder_path: ""
    load_map_folder_path: ""
    localize_on_startup: false

    enable_slam_visualization: false
    enable_observations_view: false
    enable_landmarks_view: false
    path_max_size: 10000
    verbosity: 1
```

`multicam_mode`和`slam_throttling_time_ms`在本机Isaac ROS 4.5节点中实际接受并运行，但不是当前官方API页面的主要公开调参项。迁移到不同Isaac ROS版本时，应通过`ros2 param describe`核对后再保留。

### 4.3 参数分组和修改原则

| 参数 | 当前值 | 何时修改 |
|---|---:|---|
| `tracking_mode` | 1 | 无IMU双目用0；双目+IMU用1；RGBD用2 |
| `num_cameras` | 2 | 单组双目固定2；多相机为实际图像数 |
| `min_num_images` | 2 | 通常等于必须同时到达的相机数 |
| `rectified_images` | true | 只有输入像素确实已校正时才为true |
| `sync_matching_threshold_ms` | 5.0 | 硬同步应保持小；不得用大阈值掩盖不同步 |
| `image_buffer_size` | 30 | 短时调度抖动可增大；内存也会增加 |
| `imu_buffer_size` | 400 | 应覆盖若干秒IMU；60 Hz下约6.7秒 |
| `image_qos`/`imu_qos` | SENSOR_DATA | 必须与传感器发布QoS兼容 |
| `enable_localization_n_mapping` | true | 只需VO且不要地图时可设false |
| `slam_max_map_size` | 4000 | 当前 38 m 闭环保留全部优化位姿；更大场景按实测增加 |
| `slam_throttling_time_ms` | 500 | 调整图优化/关键帧节奏前先做基准测试 |
| `enable_image_denoising` | false | 弱光噪声明显时做A/B测试后开启 |
| ground constraints | true | 当前平坦酷家乐地面启用；坡道/明显起伏必须重新 A/B |
| TF publish flags | true/true | 只有外部定位器明确接管相同TF边时才关闭 |
| invert TF flags | false/false | 不要用它修复错误的TF树或外参 |
| `override_publishing_stamp` | false | 仿真/传感器时间正常时保持false |
| visualization flags | false | 调试时短期开启；正式导航关闭以降低负载 |
| `verbosity` | 1 | 常规0–1；诊断时临时提高 |

### 4.4 launch组织

实际launch是[`visual_slam.launch.py`](../ros2_ws/src/nova_carter_bringup/launch/visual_slam.launch.py)。关键设计：

1. `robot_state_publisher`先提供相机和IMU静态TF。
2. 两个`ImageFormatConverterNode`把RGB变为`mono8`。
3. 转换节点和`VisualSlamNode`位于同一个`component_container_mt`。
4. 三个组件都启用`use_intra_process_comms`，减少转换后图像复制。
5. cuVSLAM的五个输入显式remap，避免依赖隐式命名空间。

核心组件定义：

```python
visual_slam = ComposableNode(
    package="isaac_ros_visual_slam",
    plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
    name="visual_slam_node",
    parameters=[str(share / "config/visual_slam.yaml")],
    remappings=[
        ("/visual_slam/image_0", "/front_stereo_camera/left/image_raw"),
        ("/visual_slam/camera_info_0", "/front_stereo_camera/left/camera_info"),
        ("/visual_slam/image_1", "/front_stereo_camera/right/image_raw"),
        ("/visual_slam/camera_info_1", "/front_stereo_camera/right/camera_info"),
        ("/visual_slam/imu", "/front_stereo_imu/imu"),
    ],
    extra_arguments=[{"use_intra_process_comms": True}],
)
```

可直接复制的完整launch模板如下。保存为`launch/visual_slam.launch.py`：

```python
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def mono_converter(name: str, side: str) -> ComposableNode:
    return ComposableNode(
        package="isaac_ros_image_proc",
        plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
        name=name,
        parameters=[{
            "encoding_desired": "mono8",
            "image_width": 1280,
            "image_height": 800,
            "input_qos": "SENSOR_DATA",
            "output_qos": "SENSOR_DATA",
        }],
        remappings=[
            ("image_raw", f"/front_stereo_camera/{side}/image_raw_rgb"),
            ("image", f"/front_stereo_camera/{side}/image_raw"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("nova_carter_bringup"))
    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        str(share / "urdf/nova_carter.urdf.xacro"),
    ])
    visual_slam = ComposableNode(
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        name="visual_slam_node",
        parameters=[str(share / "config/visual_slam.yaml")],
        remappings=[
            ("/visual_slam/image_0", "/front_stereo_camera/left/image_raw"),
            ("/visual_slam/camera_info_0", "/front_stereo_camera/left/camera_info"),
            ("/visual_slam/image_1", "/front_stereo_camera/right/image_raw"),
            ("/visual_slam/camera_info_1", "/front_stereo_camera/right/camera_info"),
            ("/visual_slam/imu", "/front_stereo_imu/imu"),
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_description": robot_description,
            }],
        ),
        ComposableNodeContainer(
            name="front_stereo_vslam_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                mono_converter("left_image_normalizer", "left"),
                mono_converter("right_image_normalizer", "right"),
                visual_slam,
            ],
        ),
    ])
```

迁移时需要替换的只有：包名/Xacro、图像尺寸、五个输入topic和YAML内frame名称。不要删除`robot_state_publisher`后再用不受版本控制的临时`static_transform_publisher`命令拼接长期系统。

如果相机已经直接发布`mono8`，可以删除两个转换节点，把cuVSLAM直接remap到相机图像。转换节点的`image_width`和`image_height`必须与实际输入一致。

## 5. 构建和从干净终端启动

### 5.1 构建

```bash
cd ~/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/build.sh
```

或手动执行：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
colcon build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --symlink-install
```

每次修改YAML、launch或Xacro后都重新构建并重新source overlay：

```bash
source /opt/ros/jazzy/setup.bash
source ~/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2/ros2_ws/install/setup.bash
```

### 5.2 启动Nova Carter仿真

终端1：

```bash
cd ~/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/run_sim.sh --headless
```

终端2：

```bash
cd ~/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
./scripts/run_visual_slam.sh
```

如果还要同时启动Command Guard和wheel odometry，终端2改用：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch nova_carter_bringup phase5.launch.py
```

### 5.3 启动真实相机

顺序应为：

1. 启动相机驱动和IMU。
2. 确认图像、CameraInfo、IMU和静态TF完整。
3. 启动图像校正/灰度转换。
4. 最后启动cuVSLAM。

真实系统使用墙钟时，把所有相关节点的`use_sim_time`设为`false`，并确认没有残留`/clock`依赖。

## 6. 启动后的逐项验证

### 6.1 确认节点、参数和remap

```bash
ros2 component list
ros2 node info /visual_slam_node
ros2 param dump /visual_slam_node
ros2 param get /visual_slam_node tracking_mode
ros2 param get /visual_slam_node camera_optical_frames
```

用下面命令检查当前安装版本是否真的声明某个参数：

```bash
ros2 param describe /visual_slam_node image_buffer_size
ros2 param describe /visual_slam_node slam_throttling_time_ms
```

### 6.2 跟踪状态

```bash
ros2 topic echo /visual_slam/status
ros2 topic hz --window 100 /visual_slam/status
ros2 topic hz --window 100 /visual_slam/tracking/odometry
```

`VisualSlamStatus.vo_state`：

- `0`：未知/尚未初始化。
- `1`：跟踪成功。
- `2`：跟踪失败。

同时记录：

- `node_callback_execution_time`：节点处理完整输入到输出的时间。
- `track_execution_time`：cuVSLAM本次跟踪时间。
- `track_execution_time_mean/max`：累计性能指标。

### 6.3 输出话题

主要输出：

| Topic | 类型 | 用途 |
|---|---|---|
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | `base_frame`连续里程计 |
| `/visual_slam/tracking/vo_pose` | `geometry_msgs/PoseStamped` | 当前VO位姿 |
| `/visual_slam/tracking/vo_pose_covariance` | `PoseWithCovarianceStamped` | 带协方差位姿 |
| `/visual_slam/tracking/vo_path` | `nav_msgs/Path` | VO轨迹 |
| `/visual_slam/tracking/slam_path` | `nav_msgs/Path` | 图优化后的SLAM轨迹 |
| `/visual_slam/status` | `VisualSlamStatus` | 跟踪和性能诊断 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 标准ROS诊断 |

检查消息：

```bash
ros2 topic echo /visual_slam/tracking/odometry --once
ros2 topic echo /diagnostics --once
```

### 6.4 主TF链和唯一发布者

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
```

验收要求：

- 两条动态边持续更新且时间不倒退。
- `odom`只能有父节点`map`。
- `base_link`只能有父节点`odom`。
- 机器人静止时TF不应持续大幅漂移。

## 7. 地图保存、加载和重定位

这些接口要求`enable_localization_n_mapping: true`。

### 7.1 保存地图

地图路径每次使用新的空目录。不要覆盖唯一的有效地图：

```bash
cd ~/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2
mkdir -p data/maps/manual
MAP_DIR="$PWD/data/maps/manual/cuvslam-$(date -u +%Y%m%dT%H%M%SZ)"

ros2 service call /visual_slam/save_map \
  isaac_ros_visual_slam_interfaces/srv/FilePath \
  "{file_path: '${MAP_DIR}'}"

find "${MAP_DIR}" -maxdepth 2 -type f -ls
```

成功时返回`success: true`，当前版本保存非空`data.mdb`。

### 7.2 读取优化位姿

```bash
ros2 service call /visual_slam/get_all_poses \
  isaac_ros_visual_slam_interfaces/srv/GetAllPoses \
  "{max_count: 10000}"
```

返回的是全局优化后的PoseStamped数组，可用于检查闭环优化结果。

### 7.3 加载地图

仅加载已有数据库：

```bash
ros2 service call /visual_slam/load_map \
  isaac_ros_visual_slam_interfaces/srv/FilePath \
  "{file_path: '${MAP_DIR}'}"
```

也可以在YAML中设置：

```yaml
load_map_folder_path: /absolute/path/to/cuvslam_map
localize_on_startup: false
```

### 7.4 在地图内定位

已知近似位置时调用：

```bash
ros2 service call /visual_slam/localize_in_map \
  isaac_ros_visual_slam_interfaces/srv/LocalizeInMap \
  "{map_folder_path: '${MAP_DIR}', pose_hint: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

官方当前说明中，该服务的orientation hint会被忽略，因此首先保证位置提示处于可搜索范围。完全未知初始位姿时，应由Visual Global Localization产生`PoseWithCovarianceStamped`并发布到`/visual_slam/initial_pose`，不能要求cuVSLAM自身完成无先验全局定位。

### 7.5 重置

```bash
ros2 service call /visual_slam/reset \
  isaac_ros_visual_slam_interfaces/srv/Reset '{}'
```

重置会清除当前跟踪/建图状态。正式任务中只应由恢复状态机调用，不能在Nav2仍输出速度时随意重置。

## 8. 推荐调参顺序

每次只修改一组参数，保留相同路线、场景seed和速度，并记录修改前后的JSON/bag。推荐顺序如下。

### 第0步：冻结基线

记录：

- 当前commit和软件版本。
- 相机分辨率、频率、曝光和光照。
- CameraInfo、baseline、所有静态TF。
- 当前YAML完整副本。
- 跟踪成功率、轨迹尺度、闭环误差和GPU负载。

本仓库基线命令：

```bash
./scripts/run_phase5_tests.sh
```

### 第1步：先修几何，不调算法

依次验证：

1. 图像是否真正校正。
2. CameraInfo是否对应当前图像分辨率和裁剪。
3. baseline是否与TF/机械尺寸一致。
4. optical frame方向是否正确。
5. 相机和IMU外参是否准确。

先做1 m直行测试。尺度错误超过5%时，优先检查CameraInfo、baseline和图像校正，不要先改IMU噪声。

### 第2步：Stereo与VIO做A/B对照

同一路线分别运行：

```yaml
tracking_mode: 0  # 纯视觉双目对照
tracking_mode: 1  # 双目+IMU正式配置
```

解释：

- 两种模式都尺度错误：优先检查双目标定/校正。
- stereo正常而VIO异常：检查IMU时间、轴向、外参和噪声模型。
- VIO在快速转弯明显更稳：说明IMU融合发挥作用。
- 不要仅因为VIO有问题就永久关闭IMU；先定位输入错误。

### 第3步：同步、QoS和缓冲

先修复丢帧源头，再考虑缓冲：

- `sync_matching_threshold_ms`：硬同步双目保持5 ms或更小。
- `image_buffer_size`：应覆盖短时调度抖动，但不是无限增大。
- `imu_buffer_size`：至少覆盖图像处理峰值期间的IMU序列。
- QoS必须与发布端兼容。
- 大分辨率RGB跨进程DDS带宽高，尽量在同一组件容器内完成灰度/校正转换。

### 第4步：IMU噪声

只在时间、TF和单位正确后调整：

1. 采集静止IMU bag。
2. 根据datasheet或Allan variance得到四个参数。
3. `calibration_frequency`写入生成该模型的真实采样频率。
4. 运行慢速直线、原地旋转、快速S弯和短时视觉弱纹理段。
5. 比较漂移、尺度、失锁次数和姿态噪声。

### 第5步：平面约束

差速底盘且地面严格平坦时可以分别测试：

```yaml
enable_ground_constraint_in_odometry: true
enable_ground_constraint_in_slam: true
```

不要在坡道、地面起伏、机器人俯仰明显或相机支架振动较大时强行开启。每次先只开一个参数做A/B测试。

### 第6步：建图参数

- `slam_max_map_size`太小会频繁裁剪地图；太大增加内存和图优化成本。
- `slam_throttling_time_ms`影响加入/优化节奏，必须观察闭环和回调P99。
- 长仓库路线建议从1000开始，按地图覆盖和内存实测调整。
- 改完后必须执行保存、重启、加载、重定位测试，而不只检查在线轨迹。

### 第7步：弱光、动态物体和遮挡

- 弱光高噪声：A/B测试`enable_image_denoising`。
- 固定遮挡区域：使用`img_mask_top/bottom/left/right`。
- 人和动态物体：后续可接入对应相机的二值`seg_mask_i`。
- 优先改善曝光、照明、相机朝向和帧率；算法参数不是糟糕图像质量的替代品。

### 第8步：性能参数

正式运行关闭：

```yaml
enable_slam_visualization: false
enable_observations_view: false
enable_landmarks_view: false
enable_debug_mode: false
```

监控：

- callback mean/P99/max。
- 输入和输出频率差。
- GPU显存和利用率。
- DDS丢帧与image jitter。
- 仿真实时因子。

## 9. 量化验收

### 9.1 最小启动验收

- 所有输入topic类型、编码和frame正确。
- cuVSLAM服务全部出现。
- `vo_state`进入1。
- `/visual_slam/tracking/odometry`持续输出。
- `map→odom→base_link`可查询且无竞争发布者。

### 9.2 1 m尺度测试

沿机器人前轴准确移动1 m：

```text
0.95 m <= ||t_measured|| <= 1.05 m
```

旋转误差建议小于15°。这是官方真实机器人验证流程采用的门槛。

### 9.3 闭环测试

- 使用约40 m无交叉闭环。
- 回到精确起点并静止。
- 位置闭环误差应小于2 m，即路线长度5%。
- 方向误差小于15°。
- 测试过程中避免引入未控制的大型动态遮挡。

### 9.4 本项目自动验收

阶段5实测结果：

| 指标 | 结果 |
|---|---:|
| 连续成功跟踪 | 123.10 s |
| 锁定后失败次数 | 0 |
| 最大成功状态间隔 | 0.800 s |
| 平移尺度中位比 | 0.9980 |
| 平移方向余弦 | 0.9966 |
| 旋转方向一致率 | 99.91% |
| `map→odom`/`odom→base_link` | 1191/1190条 |
| TF时间回退/竞争父节点 | 0/0 |
| 保存、读取、加载地图 | 全部成功 |
| 优化位姿 | 1216个 |

详细证据见[`phase5_validation.md`](phase5_validation.md)。

## 10. 常见故障定位

| 现象 | 优先检查 | 不应采取的做法 |
|---|---|---|
| 节点启动但没有odometry | remap、图像编码、CameraInfo、静态TF | 盲目增加buffer |
| `vo_state=2`频繁出现 | 模糊、弱光、低纹理、错误CameraInfo、掉帧 | 只提高verbosity后继续导航 |
| 平移尺度成倍错误 | baseline、P矩阵、图像是否真正rectified | 调IMU噪声掩盖几何错误 |
| 方向或轴向错误 | optical frame旋转、左右相机顺序、base外参 | 使用invert TF参数修正错误URDF |
| stereo正常、VIO异常 | IMU时间、单位、轴向、外参、噪声模型 | 永久关闭IMU而不诊断 |
| image jitter告警 | 渲染实时因子、DDS带宽、发布频率、CPU/GPU负载 | 单纯增大jitter阈值 |
| TF抖动或Nav2跳变 | 重复TF发布者、时间回退、map/odom所有权 | 同时让wheel odom和cuVSLAM发布odom→base |
| 保存地图失败 | SLAM模式、路径权限、目录是否为新的空目录 | 覆盖唯一有效地图 |
| 地图加载成功但无法定位 | 当前图像与地图特征、位姿提示搜索范围 | 把load成功等同于全局定位成功 |
| RViz卡顿影响跟踪 | 关闭landmarks/observations/debug、降低可视化负载 | 正式运行始终开启全部调试点云 |

诊断模式可以临时设置：

```yaml
verbosity: 2
enable_debug_mode: true
debug_dump_path: /tmp/cuvslam
```

debug dump可能占用大量磁盘，只在可控短实验中启用，完成后恢复关闭。

## 11. 迁移到另一台电脑的最终检查表

- [ ] Isaac ROS版本和本手册一致，或已对照该版本官方参数表。
- [ ] GPU、驱动、CUDA和TensorRT能够被Isaac ROS节点使用。
- [ ] 每个终端source相同的ROS和overlay。
- [ ] DDS domain和发现范围一致。
- [ ] 左右图是未压缩灰度图，分辨率和频率一致。
- [ ] 双目硬同步或时间差满足阈值。
- [ ] CameraInfo对应当前图像，baseline由P矩阵计算正确。
- [ ] 输入图像是否校正与`rectified_images`一致。
- [ ] optical frame使用ROS轴约定。
- [ ] `base_link→camera/imu`外参准确且只有一个发布者。
- [ ] IMU频率、单位、轴向、时间戳和噪声模型正确。
- [ ] `use_sim_time`与实际数据源一致。
- [ ] cuVSLAM独占`map→odom→base_link`。
- [ ] 1 m尺度和方向测试通过。
- [ ] 至少2分钟连续跟踪无失败。
- [ ] 闭环误差通过。
- [ ] 地图保存、重启、加载和重定位全部通过。

完成以上检查后，再把cuVSLAM输出接入nvblox和Nav2。否则后级系统会把定位尺度、TF或时间错误放大成代价地图错位、路径振荡和避障失败。
