# Operation

Available through Phase 1:

```bash
./scripts/build.sh
./scripts/bootstrap_baremetal.sh --preflight
./scripts/smoke_ros_bridge.sh
python3 tools/check_stage1.py
./scripts/smoke_isaac_ros_nodes.sh
```

`smoke_ros_bridge.sh` starts only its own Isaac Sim process group, receives real `/clock`, image, and CameraInfo messages, and then terminates that process group. It never uses `killall` or `pkill`.

`smoke_isaac_ros_nodes.sh` loads cuVSLAM, nvblox, and VGL one at a time. A component passes only if it remains alive in its waiting-for-input state for the complete smoke interval; an early exit is reported with its log.

Simulation with the warehouse and Nova Carter, mapping, navigation, and acceptance commands will be added and validated in their corresponding phases. No placeholder command should be treated as a working entrypoint.
