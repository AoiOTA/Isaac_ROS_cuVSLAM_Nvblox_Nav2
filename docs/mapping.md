# 酷家乐四组 Hawk（八路图像流）建图

## 输入契约

`mapping_8cam` 固定按以下顺序使用 8 路单目图像：

1. front left / right
2. left left / right
3. right left / right
4. back left / right

四组 Hawk 均为 `1280×800 @ 10 Hz`，front IMU 为 `120 Hz`。nvblox 只接收 front Hawk 左目的 `640×400` 原生模拟深度；视觉建图与 nvblox 不共享合成双目深度。

Jackal LiDAR 明确关闭，模拟器不会创建 LiDAR prim、render product 或 ROS publisher。
导航中名为 `/front_depth/scan[_raw]` 的 `LaserScan` 是由 front Hawk 原生深度投影
得到的二维安全表示，不是雷达数据。

话题清单在 `ros2_ws/src/jackal_bringup/config/mapping_topics_8cam.yaml`。cuVSLAM 建图要求 `num_cameras=8` 且 `min_num_images=8`，因此任一路缺帧都不能悄悄退化为少相机地图。

## 建图命令

```bash
./scripts/run_mapping.sh --map kujiale_jackal_8cam --interactive
```

该脚本有三个硬保护：

- 必须显式传入 `--interactive` 且 stdin 是 TTY；
- 目标地图目录非空时拒绝覆盖；
- 同一时刻只允许一个 mapping workflow。

GUI 出现后用 `W/S/A/D` 驾驶，`Space` 停车，`Q` 保存。建议缓慢遍历所有目标区域、门洞与走廊，并形成闭环。

## 生成流程

```text
8 RGB + 8 CameraInfo + IMU + TF + clock
  -> temporary MCAP
  -> offline cuVSLAM pose/map
  -> ALIKED features + cuVGL vocabulary/BoW index

front native depth + TF
  -> nvblox static map + mesh + 2D ESDF
  -> occupancy map

all runtime groups
  -> manifest.json
  -> delete temporary MCAP and offline workspace
```

cuVSLAM 与 cuVGL 必须来自同一份 MCAP。`create_vgl_map.sh` 默认使用 `40000 µs` 同步窗，并把同一值写入地图内冻结的 cuVGL runtime config；不要在导航时另行使用不匹配的同步配置。

## 运行时地图

```text
data/maps/kujiale_jackal_8cam/
├── config/       cuVGL runtime pb.txt
├── cuvgl/        keyframes, vocabulary, bow_index
├── cuvslam/      cuVSLAM database
├── mesh/         PLY
├── nvblox/       .nvblx, PLY, rates/timings
├── occupancy/    map.yaml, map.pgm
└── manifest.json
```

`manifest.json` 记录：

- 三个源 USD 的路径、default prim 与 SHA-256；
- `mapping_8cam=8`、`navigation_6cam=6`、后向导航 render product 为 false；
- 8 个图像话题的实际消息数；
- 每个运行时目录的文件数、总字节数与 tree hash；
- raw capture 未保留；
- 当前地图的验证状态。

完整检查：

```bash
python3 tools/check_map_manifest.py data/maps/kujiale_jackal_8cam
```

检查器会拒绝缺失的 cuVSLAM DB、cuVGL BoW index、nvblox binary、mesh、occupancy、被改动的 artifact hash、旧资产 hash，以及 `.mcap`/`.db3` 或 capture/offline/online_cuvslam 泄漏。

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
