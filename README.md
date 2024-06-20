# PX4-ROS2-Gazebo-YOLOv8
Aerial Object Detection using a Drone with PX4 Autopilot and ROS 2. PX4 SITL and Gazebo Garden used for Simulation. YOLOv8 used for Object Detection.

## Demo
https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/assets/58460889/fab19f49-0be6-43ea-a4e4-8e9bc8d59af9

## Installation
### Create a virtual environment
```commandline
# create
python -m venv ~/px4-venv

# activate
source ~/px4-venv/bin/activate
```
### Clone repository
```commandline
git clone https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8.git
```
### Install PX4
tested with px4-autopilot (1.14.0)
```commandline
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```
### Install ROS 2
```commandline
cd ~
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```
### Setup Micro XRCE-DDS Agent & Client
```commandline
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
### Build ROS 2 Workspace
```commandline
mkdir -p ~/ws_sensor_combined/src/
cd ~/ws_sensor_combined/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ..
source /opt/ros/humble/setup.bash
colcon build

mkdir -p ~/ws_offboard_control/src/
cd ~/ws_offboard_control/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ..
source /opt/ros/humble/setup.bash
colcon build
```
### Install MAVSDK
```commandline
pip install mavsdk
pip install aioconsole
pip install pygame
sudo apt install ros-humble-ros-gzgarden
pip install numpy
pip install opencv-python
```
### Install YOLO
```commandline
pip install ultralytics
```
### Additional Configs
- Put below lines in your bashrc:
```commandline
source /opt/ros/humble/setup.bash
export GZ_SIM_RESOURCE_PATH=~/.gz/models
```
- Copy the content of models from main repo to ~/.gz/models
- Copy default.sdf from worlds folder in the main repo to ~/PX4-Autopilot/Tools/simulation/gz/worlds/
- Change the angle of Drone's camera for better visual:
```commandline
# Go to ~/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf then change <pose> tag in line 9 from:
<pose>.12 .03 .242 0 0 0</pose>
to:
<pose>.15 .029 .21 0 0.7854 0</pose>
```

## Run
### Fly using Keyboard
You need several terminals.
```commandline
Terminal #1:
cd ~/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888

Terminal #2:
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4

Terminal #3:
ros2 run ros_gz_image image_bridge /camera

Terminal #4:
source ~/px4-venv/bin/activate
cd ~/PX4-ROS2-Gazebo-YOLOv8
python uav_camera_det.py

Terminal #5:
source ~/px4-venv/bin/activate
cd ~/PX4-ROS2-Gazebo-YOLOv8
python keyboard-mavsdk-test.py
```
When you run the last command a blank window will open for reading inputs from keyboard. focus on that window by clicking on it, then hit "r" on keyboard to arm the drone, and use WASD and Up-Down-Left-Right on the keyboard for flying, and use "l" for landing.

### Fly using ROS 2
You need several terminals.
```commandline
Terminal #1:
cd ~/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888

Terminal #2:
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="283.08,-136.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4

Terminal #3:
ros2 run ros_gz_image image_bridge /camera

Terminal #4:
source ~/px4-venv/bin/activate
cd ~/PX4-ROS2-Gazebo-YOLOv8
python uav_camera_det.py

Terminal #5:
cd ~/ws_offboard_control
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run px4_ros_com offboard_control
```

## Acknowledgement
- https://github.com/PX4/PX4-Autopilot
- https://github.com/ultralytics/ultralytics
- https://www.ros.org/
- https://gazebosim.org/


# Appendix for ArduPilot

## Install and setup ArduPilot

### Clone ArduPilot repository

Developers should clone the main ArduPilot repository (if they simply want to download and compile the latest code) or their own fork (if they want to make changes to the source code and potentially submit changes back).

Instructions for commonly used tools are below but whichever tool is used, the URL for the source repo will be required. This can be found on the right side of the screen on each Github repository home page but in general the URL is:

https://github.com/ArduPilot/ardupilot.git for the main ardupilot repo

https://github.com/your-github-account/ardupilot for your fork of the ardupilot repo

```commandline
git clone --recurse-submodules https://github.com/your-github-userid/ardupilot
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

For Gazebo garden

```commandline
sudo apt update
sudo apt install libgz-sim7-dev rapidjson-dev
```

For Gazebo Harmonic

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

```commandline
mkdir -p gz_ws/src && cd gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo
```

### Build the plugin

Set GZ_VERSION environment variable according to installed gazebo version (replace harmonic with garden if required):

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
