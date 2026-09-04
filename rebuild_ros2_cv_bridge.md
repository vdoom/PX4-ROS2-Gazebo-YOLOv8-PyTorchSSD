# Rebuild ROS 2 Jazzy `cv_bridge` for a custom OpenCV

JetPack 7 is based on Ubuntu 24.04, so use ROS 2 Jazzy. The former `humble`
branch instructions target the Ubuntu 22.04 generation and should not be used
for a native JetPack 7 installation.

Most users should install the binary package instead:

```bash
sudo apt install ros-jazzy-cv-bridge python3-opencv
```

Only rebuild `cv_bridge` if this project must use an OpenCV installation built
at a non-default path (for example, a custom CUDA-enabled OpenCV). Do not run
the binary-package command above in that case. Also avoid the
`ros-jazzy-desktop` and `ros-jazzy-ros-gz` metapackages: their demo and GUI
dependencies can install the binary bridge and Ubuntu OpenCV. The main README
installs the smaller ROS/Gazebo package set needed by this project.

The commands below assume the custom OpenCV Python binding was compiled against
NumPy 1.x. The build and application environment therefore use `numpy<2`; do
not upgrade that environment to NumPy 2 without rebuilding OpenCV and
`cv_bridge` against the same NumPy ABI.

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

Confirm both the CMake package and custom Python binding before building:

```bash
test -f "$OpenCV_DIR/OpenCVConfig.cmake"
python3 - <<'PY'
import cv2
import numpy

print(f"NumPy: {numpy.__version__}")
print(f"OpenCV: {cv2.__version__} ({cv2.__file__})")
assert numpy.__version__.split('.', 1)[0] == '1', \
    "This guide expects the custom OpenCV build to use NumPy 1.x"
PY
```

Install dependencies and build against ROS 2 Jazzy:

```bash
cd ~/ros2_custom_ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src/vision_opencv/cv_bridge \
  --ignore-src -r -y \
  --skip-keys "libopencv-dev python3-opencv"
colcon build --packages-select cv_bridge \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DOpenCV_DIR="$OpenCV_DIR" \
    -DPython3_EXECUTABLE=/usr/bin/python3
```

The skipped rosdep keys are intentional: satisfying them through apt would add
a second OpenCV installation. If this workspace was previously configured
against another OpenCV, remove its `build/cv_bridge` and `install/cv_bridge`
directories before rebuilding.

Verify which OpenCV libraries the result uses:

```bash
ldd ~/ros2_custom_ws/install/cv_bridge/lib/libcv_bridge.so | grep opencv
```

Every reported OpenCV library should resolve to the custom installation, not
`/usr/lib/aarch64-linux-gnu`. If the custom libraries are under `/usr/local/lib`
but are not found, run `sudo ldconfig` and check again.

Create the application environment without installing PyPI's OpenCV wheel.
Keep NumPy on 1.x, matching the custom OpenCV build:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_custom_ws/install/local_setup.bash
python3 -m venv --system-site-packages ~/px4-venv
source ~/px4-venv/bin/activate
python -m pip install --upgrade pip "setuptools>=77,<80"

# Stop here unless a JetPack-compatible PyTorch installation is already visible.
python - <<'PY'
import torch
import torchvision

print(f"PyTorch: {torch.__version__} ({torch.__file__})")
print(f"torchvision: {torchvision.__version__} ({torchvision.__file__})")
print(f"CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    test_tensor = torch.ones(1, device="cuda")
    print(f"CUDA tensor test: {(test_tensor + 1).item()}")
PY

python -m pip install "numpy<2" mavsdk aioconsole pygame

# Resolve only dependencies that cannot replace OpenCV or PyTorch.
python -m pip install \
  filelock matplotlib pillow pyyaml requests psutil polars nvidia-ml-py httpx

# These packages depend on OpenCV or PyTorch, so never resolve their dependencies.
python -m pip install --no-deps \
  "ultralytics-thop==2.1.6" \
  "ultralytics-platform==0.1.21" \
  "ultralytics==8.4.138"
```

Applying `--no-deps` to both Ultralytics and `ultralytics-thop` prevents pip
from adding `opencv-python`, replacing a JetPack-compatible PyTorch build, or
downloading a separate multi-gigabyte CUDA stack. PyTorch and torchvision must
already be installed in versions compatible with the active JetPack release.

Before running this repository's Python nodes, source the custom overlay after
Jazzy and before activating the Python virtual environment:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_custom_ws/install/local_setup.bash
source ~/px4-venv/bin/activate
```

Finally, verify that Python and the bridge resolve to the intended builds:

```bash
python - <<'PY'
import cv2
import cv_bridge
import torch
from cv_bridge import CvBridge

print(f"OpenCV: {cv2.__version__} ({cv2.__file__})")
print(f"PyTorch: {torch.__version__}; CUDA available: {torch.cuda.is_available()}")
print(f"cv_bridge: {cv_bridge.__file__}")
PY
```
