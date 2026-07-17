# Troubleshooting

## Phase-0 build failure

- Confirm `/opt/ros/jazzy/setup.bash` exists.
- Confirm `/home/lyb/miniconda3/envs/isaacsim/bin/python` exists.
- Confirm `nvidia-smi` can access the RTX 4090.
- Confirm the two paths in `config/assets.yaml` exist.

`scripts/build.sh` is read-only with respect to system packages and official USD assets. It does not use sudo and does not terminate any running process.

## Bare-metal installation interruption

Rerun the same command:

```bash
./scripts/bootstrap_baremetal.sh --install
```

Repository and rosdep files are installed idempotently. Each run writes its APT simulation and installed-package manifest under `data/logs/stage1/`.

## APT proposes removals

The installer stops before the real target install. Inspect the newest `data/logs/stage1/apt-simulate-*.log`; do not bypass this check. Existing ROS 2 Jazzy, Nav2, OpenCV, and NVIDIA driver packages must remain installed.

## ROS Bridge smoke failure

Inspect the newest `data/logs/stage1/ros-bridge-*.log`. The smoke script sources the system Jazzy environment and uses the project DDS settings before launching Isaac Sim. It controls only the PID/PGID it created.

## `pip check` reports PyNaCl/cffi

Ubuntu's `python3-nacl` package declares the compiled `python3-cffi-backend` Debian dependency, while its Python package metadata still names `cffi`. This can make `pip check` report `pynacl 1.5.0 requires cffi`. On this host, both `nacl` and `nacl.bindings` import successfully; this warning is unrelated to the Isaac ROS runtime.
