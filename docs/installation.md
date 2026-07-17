# 阶段1：Ubuntu 24.04裸机环境完整配置手册

本文档用于在另一台x86_64工作站上，从可用的Ubuntu 24.04系统开始，逐步复现本项目阶段1的软件环境。安装过程全部拆成可检查的命令，不依赖本仓库的`bootstrap_baremetal.sh`。

文档对应的固定目标组合如下。

| 项目 | 目标 |
|---|---|
| 操作系统 | Ubuntu 24.04 LTS，x86_64 |
| GPU | NVIDIA Ampere或更新架构，建议RTX 4090或同等级 |
| NVIDIA驱动 | 580或更新；本机验证版本595.71.05 |
| ROS 2 | Jazzy |
| Nav2 | Jazzy发行版 |
| 系统Python | Ubuntu自带Python 3.12 |
| Isaac Sim Python | Python 3.12独立环境 |
| Isaac Sim | 6.0.1.0 |
| CUDA Toolkit | 13.0系列；本机安装13.0.3-1 |
| TensorRT | 10.13.3.9，CUDA 13.0构建 |
| Isaac ROS | release-4.5 / ROS包版本4.5.0 |
| OpenCV | 4.6.0 |
| 安装模式 | Bare Metal，不使用Docker |

NVIDIA将bare-metal模式标记为高级用法：APT包直接安装到主机，Python依赖直接安装到系统Python。执行前应确认这台机器允许系统级CUDA、TensorRT和Python包变更。

## 1. 安装前准备

### 1.1 确认系统版本和架构

执行：

```bash
cat /etc/os-release
dpkg --print-architecture
uname -m
locale charmap
```

应满足：

- `VERSION_ID="24.04"`；
- Debian架构为`amd64`；
- 内核架构为`x86_64`；
- locale字符集为`UTF-8`。

如果locale不是UTF-8，执行：

```bash
sudo apt-get update
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

重新检查：

```bash
locale charmap
```

### 1.2 检查磁盘、内存和GPU

```bash
free -h
df -h /
lspci | grep -i nvidia
nvidia-smi
```

建议至少预留80 GiB磁盘空间。本项目完整目标包会间接安装CUDA开发工具、TensorRT开发包、Triton和CUDA PyTorch，实际下载和解包体积明显大于几个核心ROS包本身。

驱动版本单独读取：

```bash
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
```

驱动主版本必须不低于580。

### 1.3 仅在驱动缺失或低于580时安装驱动

如果`nvidia-smi`正常且版本已经满足要求，不要重复安装驱动。如果驱动缺失：

```bash
sudo apt-get update
sudo apt-get install -y ubuntu-drivers-common
ubuntu-drivers devices
sudo ubuntu-drivers install
```

驱动安装后需要重启：

```bash
sudo reboot
```

系统回来后重新执行：

```bash
nvidia-smi
```

只有该命令能够正常显示GPU、驱动和显存时，才继续后面的步骤。

### 1.4 安装基础工具

```bash
sudo apt-get update
sudo apt-get install -y \
  ca-certificates \
  curl \
  wget \
  gnupg \
  software-properties-common \
  build-essential \
  cmake \
  git \
  git-lfs \
  ccache
```

验证：

```bash
curl --version
gpg --version
git --version
cmake --version
```

## 2. 安装ROS 2 Jazzy和Nav2

如果`/opt/ros/jazzy/setup.bash`已经存在，可以先执行第2.5节的检查；全部通过时不必重复安装。

### 2.1 启用Ubuntu Universe仓库

```bash
sudo add-apt-repository universe
sudo apt-get update
```

### 2.2 安装ROS官方APT源配置包

读取当前ROS APT源配置包版本：

```bash
ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F 'tag_name' \
  | awk -F'"' '{print $4}')"
echo "${ROS_APT_SOURCE_VERSION}"
```

下载Ubuntu Noble对应的配置包：

```bash
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
```

安装并刷新索引：

```bash
sudo dpkg -i /tmp/ros2-apt-source.deb
rm -f /tmp/ros2-apt-source.deb
sudo apt-get update
```

### 2.3 安装ROS、Nav2和开发工具

```bash
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-rosbag2-storage-mcap \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

### 2.4 加载ROS环境

当前终端立即加载：

```bash
source /opt/ros/jazzy/setup.bash
```

需要每个Bash终端自动加载时，先检查是否已经写入：

```bash
grep -F 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || \
  echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

### 2.5 验证ROS和Nav2

```bash
echo "${ROS_DISTRO}"
ros2 pkg prefix rclcpp
ros2 pkg prefix nav2_bringup
ros2 pkg prefix nav2_controller
ros2 pkg prefix rviz2
```

预期：

- `ROS_DISTRO`为`jazzy`；
- 上述包前缀均为`/opt/ros/jazzy`；
- 任意命令出现`Package not found`时，不要继续安装Isaac ROS。

## 3. 准备Isaac Sim 6.0.1 Python环境

Isaac ROS裸机包安装在系统Python中；Isaac Sim应放在独立Python 3.12环境中，避免两边的Python包互相覆盖。本项目实测使用Conda环境。

如果另一台机器已经具有可运行的Isaac Sim 6.0.1，可以跳到第3.3节。

### 3.0 仅在Conda尚未安装时安装Miniconda

下载Linux x86_64安装器：

```bash
curl -L -o /tmp/miniconda.sh \
  https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
```

安装到当前用户目录：

```bash
bash /tmp/miniconda.sh -b -p "$HOME/miniconda3"
rm -f /tmp/miniconda.sh
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda init bash
```

检查：

```bash
conda --version
```

### 3.1 创建Conda环境

以下命令假定已经安装Miniconda或Anaconda：

```bash
conda create -n isaacsim python=3.12 -y
conda activate isaacsim
python --version
python -m pip install --upgrade pip
```

Python必须是3.12。

### 3.2 安装固定版本Isaac Sim

Isaac Sim 6.0.1使用CUDA 13版本的PyTorch。先在Conda环境中安装官方指定版本：

```bash
conda activate isaacsim
python -m pip install 'torch==2.11.0' \
  --index-url https://download.pytorch.org/whl/cu130
```

检查PyTorch版本和CUDA构建版本：

```bash
python -c 'import torch; print(torch.__version__); print(torch.version.cuda)'
```

预期PyTorch版本为`2.11.0`，CUDA构建版本为`13.0`。

接受NVIDIA Omniverse EULA后，再安装Isaac Sim：

```bash
export OMNI_KIT_ACCEPT_EULA=YES
python -m pip install 'isaacsim[all,extscache]==6.0.1.0' \
  --extra-index-url https://pypi.nvidia.com
```

安装完成后检查：

```bash
python -c "import importlib.metadata as m; print(m.version('isaacsim'))"
python -c "from isaacsim import SimulationApp; print('SimulationApp import OK')"
```

预期版本是`6.0.1.0`。

### 3.3 记录Isaac Sim解释器绝对路径

```bash
conda activate isaacsim
which python
```

本机验证路径为：

```text
/home/lyb/miniconda3/envs/isaacsim/bin/python
```

另一台电脑的用户名或Conda安装目录可能不同，后续项目配置必须使用那台机器实际输出的路径。

### 3.4 准备Isaac Sim 6.0.1资产

Isaac Sim Python包不包含完整的本地内容资产。本项目固定使用Isaac Sim 6.0.1完整资产包，官方将其拆成五个分卷，五个文件必须全部下载。

安装下载和解压工具：

```bash
sudo apt-get update
sudo apt-get install -y aria2 unzip
mkdir -p "$HOME/Downloads"
cd "$HOME/Downloads"
```

逐个下载并按官方MD5校验。`aria2c -c`支持断点续传；校验不通过时命令会失败，不要继续拼接：

```bash
aria2c -c \
  --checksum=md5=92149a1f50a21c0f04cca6507ab00653 \
  'https://downloads.isaacsim.nvidia.com/isaac-sim-assets-complete-6.0.1.001.zip'

aria2c -c \
  --checksum=md5=9b4b924e2d31bce41712d7637a0d6e42 \
  'https://downloads.isaacsim.nvidia.com/isaac-sim-assets-complete-6.0.1.002.zip'

aria2c -c \
  --checksum=md5=b1c62924beda91251d3f5318ffec2b00 \
  'https://downloads.isaacsim.nvidia.com/isaac-sim-assets-complete-6.0.1.003.zip'

aria2c -c \
  --checksum=md5=6bd7aa4d9b6c4161c2302e4c9418ade7 \
  'https://downloads.isaacsim.nvidia.com/isaac-sim-assets-complete-6.0.1.004.zip'

aria2c -c \
  --checksum=md5=c4a17942014be6b50492ae860496fef7 \
  'https://downloads.isaacsim.nvidia.com/isaac-sim-assets-complete-6.0.1.005.zip'
```

把五个分卷按编号顺序拼成完整ZIP，然后解压到固定目录：

```bash
mkdir -p "$HOME/isaacsim_assets"
cat \
  isaac-sim-assets-complete-6.0.1.001.zip \
  isaac-sim-assets-complete-6.0.1.002.zip \
  isaac-sim-assets-complete-6.0.1.003.zip \
  isaac-sim-assets-complete-6.0.1.004.zip \
  isaac-sim-assets-complete-6.0.1.005.zip \
  > isaac-sim-assets-complete-6.0.1.zip

unzip -q isaac-sim-assets-complete-6.0.1.zip \
  -d "$HOME/isaacsim_assets"
```

确认资产根目录同时包含`Isaac`和`NVIDIA`内容树：

```bash
test -d "$HOME/isaacsim_assets/Assets/Isaac/6.0/Isaac"
test -d "$HOME/isaacsim_assets/Assets/Isaac/6.0/NVIDIA"
```

最后确认本项目使用的两个文件实际存在，而不是只相信逻辑路径：

```bash
find ~/isaacsim_assets -type f \
  -path '*/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd' -print

find ~/isaacsim_assets -type f \
  -path '*/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd' -print
```

本机验证路径为：

```text
/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd
/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd
```

另一台机器只需把`/home/lyb`替换为它自己的`$HOME`。资产包确认解压正确后，可以删除`~/Downloads`中的五个分卷和拼接后的ZIP以释放磁盘空间；不要删除`~/isaacsim_assets`。

## 4. 添加Isaac ROS 4.5软件源

以下步骤固定`release-4.5`，不会自动漂移到未来的4.x版本。

### 4.1 创建keyring目录

```bash
sudo install -d -m 0755 /usr/share/keyrings /etc/apt/keyrings
```

### 4.2 安装Isaac ROS仓库密钥

中国网络使用：

```bash
curl -fsSL https://isaac.download.nvidia.cn/isaac-ros/repos.key \
  | gpg --dearmor \
  | sudo tee /usr/share/keyrings/nvidia-isaac-ros.gpg >/dev/null
```

中国以外网络可把域名替换为：

```text
https://isaac.download.nvidia.com/isaac-ros/repos.key
```

### 4.3 写入固定release-4.5源

中国CDN：

```bash
echo 'deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.cn/isaac-ros/release-4.5 noble main' \
  | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list
```

美国CDN对应内容为：

```text
deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4.5 noble main
```

刷新并检查：

```bash
sudo apt-get update
apt-cache policy isaac-ros-cli
```

`isaac-ros-cli`必须出现来自`release-4.5`源的Candidate。

## 5. 添加CUDA 13仓库

### 5.1 下载CUDA keyring包

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
```

### 5.2 安装keyring并清理临时文件

```bash
sudo dpkg -i cuda-keyring_1.1-1_all.deb
rm -f cuda-keyring_1.1-1_all.deb
sudo apt-get update
```

### 5.3 检查CUDA 13.0候选版本

```bash
apt-cache policy cuda-toolkit-13-0
```

包必须存在候选版本，版本号应以`13.0`开头。

## 6. 添加Jetson x86 Noble仓库

即使目标机器是RTX 4090 x86工作站，Isaac ROS 4.5裸机依赖仍需要NVIDIA Jetson x86 Noble仓库。

### 6.1 安装仓库密钥

```bash
curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
  | gpg --dearmor \
  | sudo tee /etc/apt/keyrings/nvidia-jetson.gpg >/dev/null
```

### 6.2 写入r38.4源

```bash
echo 'deb [signed-by=/etc/apt/keyrings/nvidia-jetson.gpg] https://repo.download.nvidia.com/jetson/x86_64/noble r38.4 main' \
  | sudo tee /etc/apt/sources.list.d/nvidia-jetson-apt-source.list
```

刷新索引：

```bash
sudo apt-get update
```

检查VPI候选包，确认该源生效：

```bash
apt-cache policy libnvvpi4
```

## 7. 配置ROS开发工具和Isaac ROS rosdep定义

### 7.1 初始化rosdep

先检查默认文件：

```bash
test -f /etc/ros/rosdep/sources.list.d/20-default.list && echo 'rosdep already initialized'
```

如果文件不存在，执行一次：

```bash
sudo rosdep init
```

### 7.2 下载Isaac ROS 4.5额外规则

```bash
sudo curl -L \
  -o /etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml \
  https://raw.githubusercontent.com/NVIDIA-ISAAC-ROS/isaac-ros-cli/release-4.5/docker/rosdep/extra_rosdeps.yaml
```

写入rosdep源声明：

```bash
echo 'yaml file:///etc/ros/rosdep/sources.list.d/nvidia-isaac.yaml' \
  | sudo tee /etc/ros/rosdep/sources.list.d/00-nvidia-isaac.list
```

更新索引：

```bash
rosdep update
```

### 7.3 验证ROS开发命令

```bash
colcon version-check
vcs --help >/dev/null
rosdep --version
```

## 8. 检查OpenCV，禁止盲目删除

Isaac ROS 4.5测试组合要求OpenCV 4.6.0。先检查Python和Debian包：

```bash
python3 -c 'import cv2; print(cv2.__version__)'
dpkg-query -W -f='${Package}\t${Version}\n' 'libopencv*' 2>/dev/null | sort
```

如果Python输出`4.6.0`，保持现状，不执行任何OpenCV卸载命令。

如果不是4.6.0，先模拟评估可能删除的包：

```bash
apt-get --simulate remove 'libopencv*' 'opencv*'
```

只有确认不会破坏其他必须保留的软件后，才按NVIDIA官方说明处理冲突版本。不要把`sudo apt-get remove -y libopencv* opencv*`作为无条件步骤。

## 9. 安装并初始化Isaac ROS CLI

### 9.1 安装CLI

```bash
sudo apt-get install -y isaac-ros-cli
```

检查版本和命令：

```bash
isaac-ros --help
dpkg-query -W -f='${Package}\t${Version}\n' isaac-ros-cli
```

### 9.2 初始化bare-metal模式

交互式方式：

```bash
sudo isaac-ros init baremetal
```

阅读警告并确认后继续。需要非交互初始化时使用：

```bash
sudo isaac-ros init baremetal --yes
```

该命令会安装APT preference并配置系统Python依赖的固定版本。完成后刷新APT：

```bash
sudo apt-get update
```

### 9.3 验证固定版本优先级

```bash
apt-cache policy tensorrt libnvinfer10 python3-libnvinfer
```

目标Candidate必须为：

```text
10.13.3.9-1+cuda13.0
```

同时检查：

```bash
apt-cache policy cuda-toolkit-13-0
```

## 10. 安装前执行APT安全模拟

这是不可跳过的安全门。先运行完整模拟，不带`sudo`也可以：

```bash
apt-get --simulate install \
  cuda-toolkit-13-0 \
  tensorrt \
  git-lfs \
  ccache \
  ros-jazzy-isaac-ros-visual-slam \
  ros-jazzy-isaac-ros-nvblox \
  ros-jazzy-isaac-ros-visual-global-localization \
  ros-jazzy-isaac-ros-visual-mapping \
  ros-jazzy-isaac-mapping-ros \
  ros-jazzy-isaac-ros-image-pipeline \
  ros-jazzy-depthimage-to-laserscan \
  | tee ~/isaac_ros_phase1_apt_simulate.log
```

检查模拟结果是否提出删除包：

```bash
grep -E '^Remv |The following packages will be REMOVED:' \
  ~/isaac_ros_phase1_apt_simulate.log
```

该命令应当没有输出。有任何`Remv`或`REMOVED`时都停止，不执行真实安装。

检查CUDA、TensorRT和Isaac ROS候选版本：

```bash
grep -E '^Inst cuda-toolkit-13-0 \(13\.0\.' ~/isaac_ros_phase1_apt_simulate.log
grep -E '^Inst tensorrt \(10\.13\.3\.9-' ~/isaac_ros_phase1_apt_simulate.log
grep -E '^Inst ros-jazzy-isaac-ros-visual-slam \(4\.5\.0-' ~/isaac_ros_phase1_apt_simulate.log
grep -E '^Inst ros-jazzy-isaac-ros-nvblox \(4\.5\.0-' ~/isaac_ros_phase1_apt_simulate.log
grep -E '^Inst ros-jazzy-isaac-ros-visual-global-localization \(4\.5\.0-' ~/isaac_ros_phase1_apt_simulate.log
```

每条命令都必须有匹配输出。

## 11. 执行真实安装

确认第10节全部通过后执行：

```bash
sudo apt-get install -y \
  cuda-toolkit-13-0 \
  tensorrt \
  git-lfs \
  ccache \
  ros-jazzy-isaac-ros-visual-slam \
  ros-jazzy-isaac-ros-nvblox \
  ros-jazzy-isaac-ros-visual-global-localization \
  ros-jazzy-isaac-ros-visual-mapping \
  ros-jazzy-isaac-mapping-ros \
  ros-jazzy-isaac-ros-image-pipeline \
  ros-jazzy-depthimage-to-laserscan
```

说明：

- 完整nvblox和image-pipeline元包会间接安装Triton、CUDA PyTorch、模型安装器和大量GPU开发库；
- TensorRT开发包本身也很大；
- 下载和配置可能持续几十分钟；
- `python3-*-pip-shim`的post-install会向系统Python安装NVIDIA约束的包；
- 只要`apt-get`、`dpkg`或`pip3`仍有CPU、磁盘或网络活动，就不要中断。

安装结束后检查dpkg完整性：

```bash
dpkg --audit
```

正常情况无输出。

重新执行APT模拟：

```bash
apt-get --simulate install \
  cuda-toolkit-13-0 \
  tensorrt \
  ros-jazzy-isaac-ros-visual-slam \
  ros-jazzy-isaac-ros-nvblox \
  ros-jazzy-isaac-ros-visual-global-localization \
  ros-jazzy-isaac-ros-visual-mapping \
  ros-jazzy-isaac-mapping-ros
```

应出现类似：

```text
0 upgraded, 0 newly installed, 0 to remove
```

## 12. 配置每个终端使用的环境

当前终端执行：

```bash
source /opt/ros/jazzy/setup.bash
export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/src/tensorrt/bin:${PATH}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

`ROS_LOCALHOST_ONLY`在Jazzy中会产生deprecated警告，但当前项目固定使用该配置且功能正常。后续若统一迁移到新发现范围接口，可改用：

```bash
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

不要同时设置两套发现范围变量。

需要永久保存PATH时，可将下面一行加入`~/.bashrc`，加入前先检查是否已经存在：

```bash
grep -F '/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/src/tensorrt/bin' ~/.bashrc || \
  echo 'export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/src/tensorrt/bin:${PATH}"' >> ~/.bashrc
```

## 13. 逐项验证版本和共享库

### 13.1 CUDA

```bash
which nvcc
nvcc --version
```

预期路径为`/usr/local/cuda/bin/nvcc`，输出包含`release 13.0`。

### 13.2 TensorRT

```bash
python3 -c 'import tensorrt as trt; print(trt.__version__)'
```

预期：

```text
10.13.3.9
```

检查动态库：

```bash
ldconfig -p | grep -E 'libcudart\.so|libnvinfer\.so'
```

两种库都必须找到。

### 13.3 系统Python CUDA包

```bash
python3 -c 'import cuda; print("cuda-python import OK")'
python3 -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
```

本机验证的Torch版本为`2.9.0+cu130`，`torch.cuda.is_available()`应为`True`。

### 13.4 Isaac ROS包

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix isaac_ros_visual_slam
ros2 pkg prefix isaac_ros_nvblox
ros2 pkg prefix isaac_ros_visual_global_localization
ros2 pkg prefix isaac_ros_visual_mapping
ros2 pkg prefix isaac_mapping_ros
```

全部应返回`/opt/ros/jazzy`。

查看ROS包版本：

```bash
grep '<version>' /opt/ros/jazzy/share/isaac_ros_visual_slam/package.xml | head -1
grep '<version>' /opt/ros/jazzy/share/isaac_ros_nvblox/package.xml | head -1
grep '<version>' /opt/ros/jazzy/share/isaac_ros_visual_global_localization/package.xml | head -1
```

均应为`4.5.0`。

### 13.5 Debian包版本清单

```bash
dpkg-query -W -f='${Package}\t${Version}\n' \
  cuda-toolkit-13-0 \
  tensorrt \
  isaac-ros-cli \
  ros-jazzy-isaac-ros-visual-slam \
  ros-jazzy-isaac-ros-nvblox \
  ros-jazzy-isaac-ros-visual-global-localization \
  ros-jazzy-isaac-ros-visual-mapping \
  ros-jazzy-isaac-mapping-ros
```

## 14. 实际运行CUDA内核

创建临时CUDA测试源码：

```bash
tee /tmp/cuda_smoke.cu >/dev/null <<'CU'
#include <cuda_runtime.h>
#include <cmath>
#include <iostream>

__global__ void add_one(float* value) { value[0] += 1.0F; }

int main() {
  float* device_value = nullptr;
  float host_value = 41.0F;
  if (cudaMalloc(&device_value, sizeof(float)) != cudaSuccess) return 1;
  if (cudaMemcpy(device_value, &host_value, sizeof(float), cudaMemcpyHostToDevice) != cudaSuccess) return 2;
  add_one<<<1, 1>>>(device_value);
  if (cudaDeviceSynchronize() != cudaSuccess) return 3;
  if (cudaMemcpy(&host_value, device_value, sizeof(float), cudaMemcpyDeviceToHost) != cudaSuccess) return 4;
  cudaFree(device_value);
  if (std::fabs(host_value - 42.0F) > 1.0e-6F) return 5;
  std::cout << "CUDA smoke result: " << host_value << '\n';
  return 0;
}
CU
```

编译并运行：

```bash
nvcc /tmp/cuda_smoke.cu -o /tmp/cuda_smoke
/tmp/cuda_smoke
```

预期：

```text
CUDA smoke result: 42
```

清理：

```bash
rm -f /tmp/cuda_smoke /tmp/cuda_smoke.cu
```

## 15. 实际构建TensorRT引擎

直接在终端执行下面的Python片段：

```bash
python3 - <<'PY'
import tensorrt as trt

logger = trt.Logger(trt.Logger.ERROR)
builder = trt.Builder(logger)
network = builder.create_network(
    1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
)
input_tensor = network.add_input("input", trt.float32, (1, 1))
identity = network.add_identity(input_tensor)
network.mark_output(identity.get_output(0))
config = builder.create_builder_config()
engine = builder.build_serialized_network(network, config)
if engine is None or engine.nbytes == 0:
    raise RuntimeError("TensorRT engine serialization failed")
print("TensorRT", trt.__version__, "engine bytes", engine.nbytes)
PY
```

命令应输出TensorRT 10.13.3.9和一个大于0的引擎字节数。

## 16. 实际装载三个Isaac ROS核心组件

所有命令都应在已执行`source /opt/ros/jazzy/setup.bash`的终端运行。

### 16.1 cuVSLAM

```bash
timeout --signal=INT --kill-after=5s 12s \
  ros2 component standalone --no-daemon --use-sim-time \
  isaac_ros_visual_slam \
  nvidia::isaac_ros::visual_slam::VisualSlamNode \
  -p enable_image_denoising:=false
echo "exit_status=$?"
```

没有传感器输入时，组件应保持运行直到被`timeout`终止；正常验收状态码为124。

### 16.2 nvblox

```bash
timeout --signal=INT --kill-after=5s 12s \
  ros2 component standalone --no-daemon --use-sim-time \
  nvblox_ros \
  nvblox::NvbloxNode \
  -p num_cameras:=1 \
  -p use_depth:=true \
  -p use_lidar:=false \
  -p global_frame:=odom
echo "exit_status=$?"
```

正常验收状态码同样为124。

### 16.3 Visual Global Localization

```bash
timeout --signal=INT --kill-after=5s 12s \
  ros2 component standalone --no-daemon --use-sim-time \
  isaac_ros_visual_global_localization \
  nvidia::isaac_ros::visual_global_localization::VisualGlobalLocalizationNode \
  -p num_cameras:=2 \
  -p stereo_localizer_cam_ids:='0,1' \
  -p enable_continuous_localization:=false \
  -p publish_map_to_base_tf:=false \
  -p map_frame:=map \
  -p base_frame:=base_link
echo "exit_status=$?"
```

正常验收状态码为124。此时没有VGL地图和相机输入，阶段1只验证组件能够装载并等待输入。

确认组件注册名：

```bash
ros2 component types | grep -E 'VisualSlamNode|NvbloxNode|VisualGlobalLocalizationNode'
```

## 17. 验证Isaac Sim和系统ROS 2能够通信

该测试使用两个独立终端，避免把Isaac Sim进程和系统ROS节点混在同一个Python环境中。

### 17.1 终端A：Isaac Sim Standalone环境

```bash
conda activate isaacsim
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

启动一个能够发布`/clock`、图像和`CameraInfo`的Isaac Sim Standalone程序。程序必须遵循以下顺序：

1. 首先导入并创建`SimulationApp`；
2. 然后启用`isaacsim.ros2.bridge`；
3. 运行时创建`ROS2PublishClock`、`ROS2CameraHelper`和`ROS2CameraInfoHelper`节点；
4. 播放timeline并保持仿真循环运行。

本仓库中已验证的最小实现位于`isaac_sim/smoke_ros_bridge.py`，它不是安装器，只是上述接口的可读参考实现。

在仓库根目录执行这个兼容性测试，持续90秒供另一个终端检查消息：

```bash
cd "$HOME/Workspace/Isaac_ROS_cuVSLAM_Nvblox_Nav2"
"$HOME/miniconda3/envs/isaacsim/bin/python" \
  isaac_sim/smoke_ros_bridge.py --duration 90
```

程序完成初始化后应输出`STAGE1_BRIDGE_READY`。如果仓库或Conda环境不在上述默认位置，应使用第3.3节记录的解释器绝对路径，并替换仓库路径。

### 17.2 终端B：系统ROS环境

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

分别检查消息：

```bash
ros2 topic echo /clock --once
ros2 topic echo /stage1/camera/camera_info --once
ros2 topic hz /stage1/camera/image_raw
```

检查图像格式：

```bash
ros2 topic echo /stage1/camera/image_raw --once
```

在输出中检查`width`、`height`、`encoding`和`header`字段。

阶段1通过条件：

- `/clock`持续前进；
- 图像非空；
- `CameraInfo`尺寸与图像相同；
- 图像与`CameraInfo`使用相同frame ID；
- 同一帧对应的时间戳一致；
- Isaac Sim进程退出时能够正常关闭ROS Bridge和`SimulationApp`。

## 18. 检查Isaac Sim固定资产

设置另一台机器的实际路径：

```bash
export ISAAC_SIM_PYTHON="$HOME/miniconda3/envs/isaacsim/bin/python"
export WAREHOUSE_USD="$HOME/isaacsim_assets/Assets/Isaac/6.0/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
export NOVA_CARTER_USD="$HOME/isaacsim_assets/Assets/Isaac/6.0/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
```

确认解释器和文件存在：

```bash
test -x "${ISAAC_SIM_PYTHON}" && echo 'Isaac Sim Python OK'
test -f "${WAREHOUSE_USD}" && echo 'Warehouse USD OK'
test -f "${NOVA_CARTER_USD}" && echo 'Nova Carter USD OK'
```

使用Isaac Sim Python实际打开USD：

```bash
"${ISAAC_SIM_PYTHON}" - "${WAREHOUSE_USD}" "${NOVA_CARTER_USD}" <<'PY'
from pathlib import Path
import sys
from pxr import Usd

for raw_path in sys.argv[1:]:
    path = Path(raw_path)
    if not path.is_file():
        raise SystemExit(f"missing: {path}")
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise SystemExit(f"cannot open: {path}")
    print(path)
    print("  default_prim:", stage.GetDefaultPrim().GetPath())
    print("  prim_count:", sum(1 for _ in stage.Traverse()))
    print("  used_layers:", len(stage.GetUsedLayers()))
PY
```

本机基线：

| 资产 | default prim | prim数量 | layer数量 |
|---|---:|---:|---:|
| Warehouse | `/World` | 3481 | 62 |
| Nova Carter | `/nova_carter` | 1292 | 19 |

不同资产补丁可能改变计数，但文件必须可打开、default prim必须合理且不得存在未解析layer。

## 19. 新终端最终复现检查

关闭用于安装的终端，打开一个全新终端，然后执行：

```bash
source /opt/ros/jazzy/setup.bash
export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/src/tensorrt/bin:${PATH}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

按顺序检查：

```bash
nvidia-smi
nvcc --version
python3 -c 'import tensorrt as trt; print(trt.__version__)'
python3 -c 'import torch; print(torch.__version__, torch.cuda.is_available())'
ros2 pkg prefix isaac_ros_visual_slam
ros2 pkg prefix isaac_ros_nvblox
ros2 pkg prefix isaac_ros_visual_global_localization
dpkg --audit
```

全部通过后，阶段1环境配置完成。

## 20. 常见问题和恢复方法

### 20.1 `sudo`认证

只在系统级APT、keyring和rosdep配置步骤输入密码。不要把sudo密码写入命令、环境变量、文件或聊天记录。

### 20.2 APT提出删除ROS、OpenCV或驱动

立即停止。保留第10节生成的`~/isaac_ros_phase1_apt_simulate.log`，检查候选源和pin优先级。不得使用`--allow-downgrades`、`--allow-remove-essential`或直接跳过模拟。

### 20.3 安装被网络中断

先执行：

```bash
sudo dpkg --configure -a
sudo apt-get --fix-broken install
sudo apt-get update
```

再次运行第10节模拟，确认安全后重新执行第11节相同的安装命令。APT会复用已经下载的归档。

### 20.4 `nvcc`找不到

```bash
export PATH="/usr/local/cuda/bin:${PATH}"
ls -l /usr/local/cuda
ls -l /usr/local/cuda/bin/nvcc
```

如果`/usr/local/cuda`未指向13.0安装目录，检查：

```bash
update-alternatives --display cuda
```

### 20.5 TensorRT版本不是10.13.3.9

```bash
apt-cache policy tensorrt libnvinfer10 python3-libnvinfer
grep -R "10.13.3.9" /etc/apt/preferences.d /etc/apt/preferences 2>/dev/null
```

确认已经执行`sudo isaac-ros init baremetal --yes`，并且Isaac ROS源固定在`release-4.5`。

### 20.6 `pip check`报告PyNaCl缺少cffi

Ubuntu的`python3-nacl`依赖编译后的`python3-cffi-backend`，但其Python元数据仍可能让`pip check`显示：

```text
pynacl 1.5.0 requires cffi, which is not installed
```

先验证实际运行：

```bash
python3 -c 'import nacl; import nacl.bindings; print(nacl.__version__)'
```

本机该导入正常，这个元数据警告与Isaac ROS运行时无关。

### 20.7 系统提示需要重启

```bash
test -e /var/run/reboot-required && cat /var/run/reboot-required.pkgs
```

如果涉及新内核或驱动，在没有运行关键任务时安排重启。重启后必须重新运行第19节检查。不要在无人确认时自动重启远程机器。

## 21. 迁移到另一台电脑时需要修改的项目配置

克隆仓库后，至少检查`config/environment.env`中的：

```text
ISAAC_SIM_ENV
ISAAC_SIM_PYTHON
ISAAC_ASSET_ROOT
WAREHOUSE_USD
NOVA_CARTER_USD
```

用户名、Conda路径和资产根目录通常会变化。ROS 2路径固定为`/opt/ros/jazzy`，CUDA路径固定为`/usr/local/cuda`。

## 22. 官方参考

- [Isaac ROS Getting Started / Bare Metal](https://nvidia-isaac-ros.github.io/getting_started/)
- [Isaac ROS release notes](https://nvidia-isaac-ros.github.io/releases/index.html)
- [ROS 2 Jazzy Ubuntu deb installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Isaac Sim 6.0.1 Python installation](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_python.html)
- [Isaac Sim downloads](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html)
- [Isaac Sim local asset packs](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html#isaac-sim-setup-local-assets-pack)

## 23. 本机已经验证的最终结果

| 检查项 | 结果 |
|---|---|
| GPU | RTX 4090，24,564 MiB |
| 驱动 | 595.71.05 |
| CUDA | nvcc release 13.0，Debian包13.0.3-1 |
| TensorRT | 10.13.3.9-1+cuda13.0 |
| Isaac ROS CLI | 2.4.0 |
| Isaac ROS包 | 4.5.0 |
| CUDA测试 | 实际内核运行结果42 |
| TensorRT测试 | 成功序列化非空引擎 |
| cuVSLAM | 组件装载并等待输入12秒 |
| nvblox | 组件装载并等待输入12秒 |
| VGL | 组件装载并等待输入12秒 |
| Isaac Sim Bridge | Clock、Image、CameraInfo实际收发成功 |
| APT最终状态 | 目标集合0新增、0删除、0升级 |
| dpkg审计 | 无未完成配置 |
