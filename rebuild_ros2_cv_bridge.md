# Rebuild ROS 2 Jazzy `cv_bridge` for a custom OpenCV

JetPack 7 is based on Ubuntu 24.04, so use ROS 2 Jazzy. The former `humble`
branch instructions target the Ubuntu 22.04 generation and should not be used
for a native JetPack 7 installation.

Most users should install the binary package instead:

```bash
sudo apt install ros-jazzy-cv-bridge python3-opencv
```

Only rebuild `cv_bridge` if this project must use an OpenCV installation built
at a non-default path (for example, a custom CUDA-enabled OpenCV).

Create a workspace:

```bash
mkdir -p ~/ros2_custom_ws/src
cd ~/ros2_custom_ws/src
```

Clone the source tag released for ROS 2 Jazzy:

```bash
git clone --branch 4.1.0 https://github.com/ros-perception/vision_opencv.git
```

Set this to the directory containing the custom OpenCV's `OpenCVConfig.cmake`:

```bash
export OpenCV_DIR=/usr/local/lib/cmake/opencv4
```

Install dependencies and build against ROS 2 Jazzy:

```bash
cd ~/ros2_custom_ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select cv_bridge \
  --cmake-args -DOpenCV_DIR="$OpenCV_DIR"
```

Verify which OpenCV libraries the result uses:

```bash
ldd ~/ros2_custom_ws/install/cv_bridge/lib/libcv_bridge.so | grep opencv
```

Before running this repository's Python nodes, source the custom overlay after
Jazzy and before activating the Python virtual environment:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_custom_ws/install/local_setup.bash
source ~/px4-venv/bin/activate
```
