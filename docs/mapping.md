# Mapping

阶段7已实现统一地图根目录`data/maps/warehouse_v1`：

```text
warehouse_v1/
├── cuvslam/              离线对齐的cuVSLAM data.mdb
├── cuvgl/                ALIKED关键帧、BoW vocabulary/index
├── nvblox/               warehouse.nvblx及统计
├── mesh/warehouse.ply
├── occupancy/map.{yaml,pgm}
├── config/               冻结的cuVGL pb.txt配置
├── online_cuvslam/       在线采集保存的诊断地图
├── offline/              EDEx、轨迹和离线中间产物
└── manifest.json
```

自动生成和检查：

```bash
./scripts/run_mapping.sh --map warehouse_v1
python3 tools/check_phase7_maps.py data/maps/warehouse_v1
```

脚本执行40秒闭环采集、MCAP录制、在线cuVSLAM/nvblox/Mesh/occupancy保存、官方EDEx与离线cuVSLAM计算、ALIKED特征提取、BoW构建及TensorRT引擎缓存。运行时使用的cuVSLAM和cuVGL地图来自同一bag、同一次离线轨迹计算；nvblox、Mesh和occupancy则来自录制该bag的同一在线采集进程。

Isaac Sim原生32FC1米制深度是nvblox输入；基线不运行FoundationStereo或ESS。occupancy由`odom`坐标系的2D ESDF slice保存，障碍距离门槛初值0.28 m。阶段8接入Nav2前必须先验证其与加载后的`map→odom`关系，再决定转换到`map`或仅用于局部代价地图，不能只改YAML的frame名称来伪造对齐。

单独重建视觉地图：

```bash
./scripts/export_vgl_models.sh data/models/vgl
./scripts/create_vgl_map.sh data/bags/<bag> data/maps/warehouse_v1
```

完整cuVGL跨机器过程见[cuVGL配置手册](cuvgl_configuration.md)，nvblox持久化见[nvblox配置手册](nvblox_configuration.md)，实际结果见[阶段7验证](phase7_validation.md)。
