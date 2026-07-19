# 阶段7验证记录

验证日期：2026-07-17。环境为Ubuntu 24.04.4、ROS 2 Jazzy、Isaac ROS 4.5.0、Isaac Sim 6.0.1、CUDA 13.0.3、TensorRT 10.13.3.9和RTX 4090。

## 已完成能力

- 自动运行双向闭环S形采集路线并录制MCAP。
- 在线保存cuVSLAM地图、nvblox地图、PLY Mesh和nvblox统计。
- 将在线2D ESDF slice保存为阶段性验证图；正式 Nav2 PGM/YAML 由优化位姿离线重融合生成。
- 使用官方`isaac_mapping_ros`生成EDEx、离线cuVSLAM和对齐地图帧。
- 使用官方`isaac_ros_visual_mapping`生成ALIKED关键帧、BoW vocabulary和index。
- 导出并缓存ALIKED与LightGlue TensorRT FP16引擎。
- 启动前向双目cuVGL，关闭其TF发布和连续定位。
- 校验cuVGL位姿并转发到`/visual_slam/initial_pose`。
- 五次独立重启、五个不同yaw初始位姿的全局定位和cuVSLAM恢复。

## 地图产物

| 产物 | 实测 |
|---|---:|
| MCAP | 约1.5 GB |
| cuVGL关键帧 | 142 |
| cuVGL文件 | 151个非空文件 |
| cuVSLAM | `data.mdb`非空 |
| nvblox | 约126 MB |
| Mesh | 约8.9 MB |
| occupancy | 424×624，分辨率0.05 m |
| occupancy自由/障碍/未知格 | 76,685 / 7,804 / 180,087 |
| ALIKED engine | 4,995,380 bytes |
| LightGlue engine | 27,237,308 bytes |

大文件位于`data/maps/warehouse_v1`、`data/bags`和`data/models/vgl`，按`.gitignore`不提交。

## 在线采集和五位姿重定位

40秒采集期间阶段6全部13项检查继续通过，包括Clock单调、CameraInfo持续、cuVSLAM tracking健康、闭环运动完成、TSDF/Mesh/ESDF/map slice非空、nvblox频率达标和持久化成功。

五次测试yaw为`0、+0.08、-0.08、+0.16、-0.16 rad`，每次启动全新的Isaac Sim和ROS bringup进程：成功5/5，pose frame与relay接受5/5，cuVSLAM连续tracking恢复5/5，失败hint全部为0。五次均在第2次周期触发前收到有效位姿；第一轮触发发生在图像缓存刚开始填充时，因此保留自动重试机制。

每次启动后归零的`/ground_truth/odometry`只做同起点诊断，不作为Warehouse绝对坐标；成功门由地图范围、cuVSLAM实际地图定位恢复和无失败hint共同判定。

## Isaac ROS 4.5兼容处理

Debian包内`create_cuvgl_map.py`默认二进制目录与实际布局不同，本项目显式传`/opt/ros/jazzy/lib/isaac_ros_visual_mapping`。此外`create_map_offline.py --vgl_model_dir`没有传递到内部cuVGL步骤，本项目拆分EDEx/cuVSLAM和cuVGL调用，并显式使用项目TensorRT缓存。两项都经过真实地图生成验证。

## 复现命令

```bash
./scripts/build.sh
./scripts/run_mapping.sh --map warehouse_v1
python3 tools/check_phase7_maps.py data/maps/warehouse_v1
./scripts/run_phase7_tests.sh warehouse_v1
```
