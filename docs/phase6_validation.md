# Phase 6 Validation

## Scope

Phase 6 connects native Isaac Sim metric depth and color to Isaac ROS nvblox 4.5, using the Phase 5 cuVSLAM TF chain. The baseline mapper is `static_tsdf` with a 5 cm voxel and a 2D ESDF in `odom`.

Implemented entry points:

```bash
./scripts/run_nvblox.sh
./scripts/run_phase6.sh
./scripts/save_nvblox_map.sh
./scripts/run_phase6_tests.sh
```

The test owns and cleans only its process groups and uses ROS domain 46 by default. It does not stop the other Isaac Sim project on the machine.

## Runtime configuration evidence

The acceptance script dumps `/nvblox_node` after component construction and checks the effective values:

```text
mapping_type=static_tsdf
global_frame=odom
num_cameras=1
use_tf_transforms=true
use_depth=true
use_color=true
use_lidar=false
voxel_size=0.05
integrate_depth_rate_hz=30.0
integrate_color_rate_hz=5.0
update_esdf_rate_hz=10.0
update_mesh_rate_hz=1.0
```

The latest complete dump is generated at `data/reports/phase6/nvblox-runtime-latest.yaml`.

## Final automated run

Command executed from a clean project shell:

```bash
./scripts/run_phase6_tests.sh
```

Final accepted result: `data/reports/phase6/run-20260717T070617Z.json`.

| Check | Result |
|---|---:|
| CameraInfo streams sustained | pass |
| Clock monotonic | pass |
| cuVSLAM tracking healthy | 495 status samples, 0 failures |
| Automated bidirectional S course | 15.477 m |
| TSDF nonempty | 7,431 sampled unique blocks |
| Mesh nonempty | 1,452 sampled unique blocks |
| Maximum sampled mesh vertices | 67,548 |
| Maximum sampled mesh triangles | 100,081 |
| Static ESDF nonempty | 21,533 points |
| Static map slice nonempty | 21,533 known cells |
| All reconstruction output frames | `odom` |
| nvblox map save | 104,607,744 bytes |
| PLY save | 14,769,575 bytes |
| Rates/timings save | pass |
| Unexpected PhysX overlaps | 0 |

The mapper's own saved rate statistics reported:

```text
depth image callback  19.7 Hz
depth integration     14.9 Hz
color integration      3.9 Hz
ESDF update             8.7 Hz
TSDF layer stream       1.0 Hz
mesh layer stream       1.0 Hz
```

Configured integration rates are maxima. The measured values include Isaac Sim rendering, cuVSLAM, nvblox, DDS and the acceptance observer running together while another out-of-repository Isaac Sim project remained untouched.

## Observer-load control

TSDF, mesh, ESDF pointcloud and map slice messages are large. The acceptance runner subscribes with latest-only SensorData QoS and automatically removes each subscription once it has enough nonempty samples. This proves the interface while avoiding sustained test-induced serialization load.

The motion command is rate-limited to 20 Hz, matching the future Nav2 controller rate. A prior diagnostic loop sent commands as fast as callbacks arrived; that was corrected before the final accepted result.

## 2D ESDF service finding

An initial diagnostic called `/nvblox_node/get_esdf_and_gradient` while `esdf_mode=2d`. nvblox 4.5 reports that this service is 3D-only and terminates the node with a FATAL message. The production test therefore does not call this service in 2D mode and validates ESDF through:

```text
/nvblox_node/static_esdf_pointcloud
/nvblox_node/static_map_slice
```

This limitation and its safe operating rule are recorded in the nvblox configuration guide.

## Persistence evidence

The final run invoked these real services:

```text
/nvblox_node/save_map
/nvblox_node/save_ply
/nvblox_node/save_rates
/nvblox_node/save_timings
```

Every response returned success and every output file was nonempty. Map loading is deliberately deferred to the Phase 7 map lifecycle acceptance, where it can be verified after a fresh process restart.

## Regression gates

The final run also reused the Phase 4 simulator report checker, proving that the project still created four control graphs and three visual sensor graphs, used the fixed official assets without modifying them, advanced simulation time monotonically, and exited without unexpected overlap.

Static package tests cover the nvblox YAML contract and launch remappings. The full workspace result after Phase 6 is 10 tests with zero errors, failures or skips.
