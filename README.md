# PX4-ROS2-Gazebo-YOLOv8

Aerial object detection using a drone with PX4 Autopilot, ROS 2, Gazebo Sim,
YOLOv8, or PyTorch SSD.

## Demo

https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/assets/58460889/fab19f49-0be6-43ea-a4e4-8e9bc8d59af9

## JetPack 7 compatibility

This guide targets the current JetPack 7 release as of 2026-09-04:

| Component | Version used by this guide |
| --- | --- |
| NVIDIA JetPack | 7.2.1 / Jetson Linux 39.2.1 |
| Ubuntu | 24.04 Noble, ARM64 |
| ROS 2 | Jazzy LTS |
| Gazebo Sim | Harmonic LTS (`gz-sim8`) |
| PX4 | v1.17.0 |
| Micro XRCE-DDS Agent | v2.4.3 |

The old Humble/Garden combination in this README is not a supported native
JetPack 7 combination. ROS 2 Humble binary packages target Ubuntu 22.04, while
JetPack 7 uses Ubuntu 24.04. Gazebo Garden reached end of life in November 2024
and is not the ROS/Gazebo pairing for Noble. The recommended Noble combination
is ROS 2 Jazzy with Gazebo Harmonic. See the official
[JetPack release information](https://developer.nvidia.com/embedded/jetpack/downloads),
[ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
and [ROS/Gazebo compatibility table](https://gazebosim.org/docs/latest/ros_installation/).

ROS 2 Jazzy supports Ubuntu Noble on 64-bit ARM. Gazebo Harmonic publishes
ARM packages, but the Gazebo project classifies ARM as a best-effort platform,
not a Tier 1 platform. If rendering is unreliable on a particular Jetson image,
try the headless or OGRE 1 fallbacks below, or run Gazebo on a supported amd64
Ubuntu 24.04 host. See [Gazebo Harmonic supported platforms](https://gazebosim.org/docs/harmonic/install/).

Do not add Ubuntu Jammy repositories or install `ros-humble-*` /
`ros-humble-ros-gzgarden` packages on JetPack 7.

## Installation on JetPack 7

### Confirm the platform

```bash
cat /etc/nv_tegra_release
. /etc/os-release && echo "$PRETTY_NAME ($VERSION_CODENAME)"
uname -m
```

For JetPack 7.2.1 the output should identify Jetson Linux `R39.2.1`, Ubuntu
`noble`, and `aarch64`. JetPack 7.0/7.1 also use Noble, but this guide is pinned
to the newer baseline above.

### Clone this repository

```bash
cd ~
git clone https://github.com/vdoom/PX4-ROS2-Gazebo-YOLOv8-PyTorchSSD.git
```

### Install PX4 and Gazebo Harmonic

PX4 v1.17 supports Ubuntu 24.04 and Gazebo Harmonic. Pinning the release keeps
the PX4 firmware and `px4_msgs` definitions reproducible.

```bash
cd ~
git clone --branch v1.17.0 --recursive \
  https://github.com/PX4/PX4-Autopilot.git
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh

# Reboot after the setup script completes, then build SITL.
cd ~/PX4-Autopilot
make px4_sitl
```

The PX4 setup script installs `gz-harmonic` on Ubuntu 24.04. Do not separately
install Garden. See the [PX4 Ubuntu setup guide](https://docs.px4.io/v1.17/en/dev_setup/dev_env_linux_ubuntu).

### Install ROS 2 Jazzy and the Harmonic bridge

These commands use the current ROS repository bootstrap package rather than a
manually managed repository key.

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe

export ROS_APT_SOURCE_VERSION=$(curl -s \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F 'tag_name' | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-dev-tools \
  python3-venv

grep -qxF 'source /opt/ros/jazzy/setup.bash' ~/.bashrc \
  || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

These are the ROS/Gazebo packages needed by this project. The larger
`ros-jazzy-desktop` and `ros-jazzy-ros-gz` metapackages include demos and GUI
tools that can pull Ubuntu's OpenCV and binary `cv_bridge` into the installation.
The packages above still use Jazzy's Harmonic vendor-package pairing and are
available as Noble ARM64 packages.

Choose one OpenCV / `cv_bridge` setup:

- For Ubuntu's OpenCV, install the matching binary bridge:

  ```bash
  sudo apt install -y ros-jazzy-cv-bridge python3-opencv
  ```

- For a custom OpenCV build, do not install those two packages. Build
  `cv_bridge` against the same OpenCV installation by following
  [Rebuild ROS 2 Jazzy cv_bridge for a custom OpenCV](rebuild_ros2_cv_bridge.md).
  This project's custom-build path assumes OpenCV was compiled against NumPy
  1.x and keeps the runtime on `numpy<2`. The guide also avoids installing the
  `opencv-python` wheel into the Python environment.

### Install Micro XRCE-DDS Agent

ROS 2 Jazzy / Fast DDS 2.14 requires Micro XRCE-DDS Agent v2.4.3 with the
default PX4 v1.17 client. Do not clone an unpinned Agent `main` branch, which
may select the incompatible v3 protocol.

```bash
cd ~
git clone --branch v2.4.3 \
  https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd ~/Micro-XRCE-DDS-Agent
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig /usr/local/lib/
```

The version pairing is documented in the
[PX4 uXRCE-DDS guide](https://docs.px4.io/v1.17/en/middleware/uxrce_dds).

### Build the ROS 2 workspace

Use the `px4_msgs` branch that matches PX4 v1.17. One workspace is sufficient
for both the listener and offboard examples.

```bash
mkdir -p ~/ros2_px4_ws/src
cd ~/ros2_px4_ws/src
git clone --branch release/1.17 https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git

cd ~/ros2_px4_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### Create the Python environment

Use the JetPack system Python. `--system-site-packages` lets the environment
load Jazzy's Python 3.12 packages and the selected OpenCV Python binding. When
using custom OpenCV, source its `cv_bridge` overlay before creating or using the
environment, as shown below.

```bash
source /opt/ros/jazzy/setup.bash
# Custom OpenCV only: source ~/ros2_custom_ws/install/local_setup.bash
python3 -m venv --system-site-packages ~/px4-venv
source ~/px4-venv/bin/activate
python -m pip install --upgrade pip
python -m pip install "numpy<2" mavsdk aioconsole pygame

# Install every non-OpenCV dependency for the pinned Ultralytics release.
python -m pip install \
  filelock matplotlib pillow pyyaml requests psutil polars nvidia-ml-py \
  ultralytics-thop ultralytics-platform
python -m pip install --no-deps "ultralytics==8.4.138"
```

Do not install `opencv-python`, `opencv-python-headless`, or their contrib
variants in this environment. `--no-deps` prevents Ultralytics from adding a
bundled OpenCV or replacing the JetPack-compatible PyTorch build. Install
PyTorch and torchvision versions compatible with the active JetPack release,
and keep NumPy on 1.x to match the custom OpenCV build. Verify the combined ROS
and inference environment before continuing:

```bash
python - <<'PY'
import cv2
import numpy
import rclpy
import torch
from cv_bridge import CvBridge
from ultralytics import YOLO

print(f"NumPy: {numpy.__version__}")
print(f"OpenCV: {cv2.__version__} ({cv2.__file__})")
print(f"PyTorch: {torch.__version__}; CUDA available: {torch.cuda.is_available()}")
print("ROS 2, cv_bridge, and Ultralytics imports succeeded")
PY
```

This installs the application dependencies, but does not guarantee CUDA-enabled
PyTorch. A plain PyPI PyTorch build may be CPU-only on Jetson. Check the result:

```bash
python -c 'import torch; print(torch.__version__); print(torch.cuda.is_available())'
```

If CUDA reports `False`, use a PyTorch build or container explicitly compatible
with the installed JetPack release; consult NVIDIA's
[PyTorch for Jetson compatibility table](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html).
The ROS/Gazebo setup does not depend on CUDA-enabled PyTorch.

### Configure the custom world and models

Keep the project's world under a distinct name instead of overwriting PX4's
upstream `default.sdf`.

```bash
grep -qxF 'export GZ_SIM_RESOURCE_PATH="$HOME/PX4-ROS2-Gazebo-YOLOv8-PyTorchSSD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"' ~/.bashrc \
  || echo 'export GZ_SIM_RESOURCE_PATH="$HOME/PX4-ROS2-Gazebo-YOLOv8-PyTorchSSD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"' >> ~/.bashrc
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-ROS2-Gazebo-YOLOv8-PyTorchSSD/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

cp ~/PX4-ROS2-Gazebo-YOLOv8-PyTorchSSD/worlds/default.sdf \
  ~/PX4-Autopilot/Tools/simulation/gz/worlds/yolo_demo.sdf
sed -i "s/<world name='default'>/<world name='yolo_demo'>/" \
  ~/PX4-Autopilot/Tools/simulation/gz/worlds/yolo_demo.sdf
grep -n "<world name=" \
  ~/PX4-Autopilot/Tools/simulation/gz/worlds/yolo_demo.sdf
```

The filename and the SDF's internal world name must both be `yolo_demo`. PX4
uses that name when it waits for Gazebo services and spawns the vehicle.

Optionally angle the x500 depth camera down for a better view. In
`~/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf`, change both
camera pose values from:

```xml
<pose>.12 .03 .242 0 0 0</pose>
<pose relative_to="base_link">.12 .03 .242 0 0 0</pose>
```

to:

```xml
<pose>.15 .029 .21 0 0.7854 0</pose>
<pose relative_to="base_link">.15 .029 .21 0 0.7854 0</pose>
```

## Run

The current x500 depth model does not publish RGB images on a short `/camera`
Gazebo topic. Discover the fully qualified topic and remap the detector's
`camera` subscription as shown below.

### Fly using the keyboard

Use five terminals.

Terminal 1 — DDS agent:

```bash
MicroXRCEAgent udp4 -p 8888
```

Terminal 2 — PX4 and Gazebo:

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=yolo_demo \
PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" \
make px4_sitl gz_x500_depth
```

Terminal 3 — discover and bridge the RGB camera:

```bash
source /opt/ros/jazzy/setup.bash
GZ_IMAGE_TOPIC=$(gz topic -l | grep '/sensor/IMX214/image$' | head -n 1)
printf 'Bridging %s\n' "$GZ_IMAGE_TOPIC"
ros2 run ros_gz_image image_bridge "$GZ_IMAGE_TOPIC"
```

Terminal 4 — run YOLO detection, remapped to the bridged topic:

```bash
source /opt/ros/jazzy/setup.bash
# Custom OpenCV only: source ~/ros2_custom_ws/install/local_setup.bash
source ~/px4-venv/bin/activate
cd ~/PX4-ROS2-Gazebo-YOLOv8-PyTorchSSD
ROS_IMAGE_TOPIC=$(ros2 topic list | grep '/sensor/IMX214/image$' | head -n 1)
python3 uav_camera_det.py --ros-args -r camera:="$ROS_IMAGE_TOPIC"
```

Terminal 5 — keyboard controller:

```bash
source ~/px4-venv/bin/activate
cd ~/PX4-ROS2-Gazebo-YOLOv8-PyTorchSSD
python3 keyboard-mavsdk-test.py
```

The last command opens a blank input window. Click it, press `r` to arm, use
WASD and the arrow keys to fly, and press `l` to land.

### Fly using ROS 2 offboard control

Run terminals 1, 3, and 4 as above. In terminal 2 use the ROS example pose:

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=yolo_demo \
PX4_GZ_MODEL_POSE="283.08,-136.22,3.86,0.00,0,-0.7" \
make px4_sitl gz_x500_depth
```

Then run the offboard example in terminal 5:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_px4_ws/install/local_setup.bash
ros2 run px4_ros_com offboard_control
```

The example arms the simulated vehicle and commands a takeoff. Use it only with
SITL unless you have reviewed and adapted the control code for real hardware.

### Jetson troubleshooting

- If the Gazebo GUI fails or is too slow, prefix the terminal 2 command with
  `HEADLESS=1`.
- If OGRE 2 rendering fails, also try
  `PX4_GZ_SIM_RENDER_ENGINE=ogre` before the `make` command.
- If the camera topic variable is empty, wait until Gazebo is running and use
  `gz topic -l | grep -E 'IMX214|camera|image'` to inspect the actual name.
- If `ros-jazzy-*` packages cannot be found, verify that the system reports
  `noble` and `arm64` and that the ROS apt-source package was installed. Do not
  work around it by adding a Jammy repository.
- Verify the selected stack with `printenv ROS_DISTRO`, `gz sim --versions`, and
  `ros2 pkg prefix ros_gz_image`.

## Acknowledgements

- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- [Ultralytics](https://github.com/ultralytics/ultralytics)
- [ROS](https://www.ros.org/)
- [Gazebo](https://gazebosim.org/)


# Appendix for ArduPilot

> [!WARNING]
> This appendix describes the standalone ArduPilot Gazebo plugin. The plugin
> does not depend on ROS 2, but its upstream documentation does not list Ubuntu
> 24.04 ARM64 as a tested platform. ArduPilot's ROS 2 documentation currently
> supports Humble, which is not a supported native JetPack 7 combination. Treat
> the appendix as experimental on JetPack 7; it is not the setup used by the
> PX4 instructions above.

## Install and setup ArduPilot

### Clone ArduPilot repository

Developers should clone the main ArduPilot repository (if they simply want to download and compile the latest code) or their own fork (if they want to make changes to the source code and potentially submit changes back).

Instructions for commonly used tools are below but whichever tool is used, the URL for the source repo will be required. This can be found on the right side of the screen on each Github repository home page but in general the URL is:

https://github.com/ArduPilot/ardupilot.git for the main ardupilot repo

https://github.com/your-github-account/ardupilot for your fork of the ardupilot repo

```commandline
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

### Install some required packages¶
If you are on a debian based system (such as Ubuntu or Mint), we provide a script that will do it for you. From the cloned ardupilot directory :

```commandline
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
Reload the path (log-out and log-in to make it permanent):
```commandline
. ~/.profile
```

waf should always be called from the locally cloned ardupilot root directory for the local branch you are trying to build from.

Note Do not run waf with sudo! This leads to permission and environment problems.

Check this for more info https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md

### Basic usage
There are several commands in the build system for advanced usage, but here we list some basic and more used commands as example.

- Build ArduCopter

Below shows how to build ArduCopter for the Pixhawk2/Cube. Many other boards are supported and the next section shows how to get a full list of them.

```commandline
./waf configure --board CubeBlack
./waf copter
```
The first command should be called only once or when you want to change a configuration option. One configuration often used is the --board option to switch from one board to another one. For example we could switch to SkyViper GPS drone and build again:

```commandline
./waf configure --board skyviper-v2450
./waf copter
```
If building for the bebop2 the binary must be built statically:
```commandline
./waf configure --board bebop --static
./waf copter
```
The "arducopter" binary should appear in the build/<board-name>/bin directory.

- List available boards

It's possible to get a list of supported boards on ArduPilot with the command below
```commandline
./waf list_boards
```
Here are some commands to configure waf for commonly used boards:
```commandline
./waf configure --board bebop --static # Bebop or Bebop2
./waf configure --board edge           # emlid edge
./waf configure --board fmuv3          # 3DR Pixhawk 2 boards
./waf configure --board navio2         # emlid navio2
./waf configure --board Pixhawk1       # Pixhawk1
./waf configure --board CubeBlack      # Hex/ProfiCNC Cube Black (formerly known as Pixhawk 2.1)
./waf configure --board Pixracer       # Pixracer
./waf configure --board skyviper-v2450 # SkyRocket's SkyViper GPS drone using ChibiOS
./waf configure --board sitl           # software-in-the-loop simulator
./waf configure --board sitl --debug   # software-in-the-loop simulator with debug symbols
```
- List of available vehicle types

Here is a list of the most common vehicle build targets:

```commandline
./waf copter                            # All multirotor types
./waf heli                              # Helicopter types
./waf plane                             # Fixed wing airplanes including VTOL
./waf rover                             # Ground-based rovers and surface boats
./waf sub                               # ROV and other submarines
./waf antennatracker                    # Antenna trackers
./waf AP_Periph                         # AP Peripheral
```

- Clean the build

Commands clean and distclean can be used to clean the objects produced by the build. The first keeps the configure information, cleaning only the objects for the current board. The second cleans everything for every board, including the saved configure information.

Cleaning the build is very often not necessary and discouraged. We do incremental builds reducing the build time by orders of magnitude.

If submodules are failing to be synchronized, submodulesync may be used to resync the submodules. This is usually necessary when shifting development between stable releases or a stable release and the master branch.

In some some cases submodule_force_clean may be necessary. This removes all submodules and then performs a submodulesync. (Note whitelisted modules like esp_idf is not removed.)

- Upload or install

Build commands have a --upload option in order to upload the binary built to a connected board. This option is supported by Pixhawk and Linux-based boards. The command below uses the --targets option that is explained in the next item.

```commandline
./waf --targets bin/arducopter --upload
```
For Linux boards you need first to configure the IP of the board you are going to upload to. This is done on configure phase with:

```commandline
./waf configure --board <board> --rsync-dest <destination>
```
The commands below give a concrete example (board and destination IP will change according to the board used):

```commandline
./waf configure --board navio2 --rsync-dest root@192.168.1.2:/
./waf --target bin/arducopter --upload
```
This allows to set a destination to which the --upload option will upload the binary. Under the hood it installs to a temporary location and calls rsync <temp_install_location>/ <destination>.

On Linux boards there's also an install command, which will install to a certain directory, just like the temporary install above does. This can be used by distributors to create .deb, .rpm or other package types:

```commandline
./waf copter
DESTDIR=/my/temporary/location ./waf install
```


## Install the ArduPilot Gazebo Plugin¶

We use a standard version of ArduPilot with a custom plugin for Gazebo which is hosted on GitHub at: https://github.com/ArduPilot/ardupilot_gazebo.

### Install additional dependencies

- Ubuntu

For Gazebo Harmonic on JetPack 7:

```commandline
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
```

- macOS

```commandline
brew update
brew install rapidjson
```

### Create a workspace folder and clone the repository

Clone to the home directory. Otherwise configure correct paths on your own.

```commandline
mkdir -p gz_ws/src && cd gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo
```

### Build the plugin

Select Gazebo Harmonic:

```commandline
export GZ_VERSION=harmonic
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

### Configure the Gazebo environment¶
Gazebo uses a number of environment variables to locate plugins and models at run time. These may be set in the terminal used to run Gazebo, or set in your .bashrc or .zshrc files:

- In a terminal

```commandline
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

- In .bashrc or .zshrc

```commandline
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

### Using Gazebo with ArduPilot¶
Two models are provided as examples with the plugin: an Iris quadcopter and a Zephyr delta-wing.

#### Iris quadcopter¶
Run Gazebo
```commandline
gz sim -v4 -r iris_runway.sdf
```

Run SITL
```commandline
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

Arm and takeoff
```commandline
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
```


#### Zephyr delta-wing¶
The Zephyr is positioned for vertical takeoff.

Run Gazebo
```commandline
gz sim -v4 -r zephyr_runway.sdf
```

Run SITL
```commandline
sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --map --console
```

Arm, takeoff and circle
```commandline
MANUAL> mode fbwa
FBWA> arm throttle
FBWA> rc 3 1800
FBWA> mode circle
```
