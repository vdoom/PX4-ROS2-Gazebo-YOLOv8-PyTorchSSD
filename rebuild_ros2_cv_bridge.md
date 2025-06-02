make workspace folder

```
mkdir -p ~/ros2_custom_ws/src
cd ~/ros2_custom_ws/src
```

```
git clone -b humble https://github.com/ros-perception/vision_opencv.git
```

This path might be different for you.
```
export OpenCV_DIR=/usr/local/lib/cmake/opencv4
```

```
cd ~/ros2_custom_ws
```

Build
```
colcon build --packages-select cv_bridge --cmake-args -DOpenCV_DIR=/usr/local/lib/cmake/opencv4
```

verify
```
ldd ~/ros2_custom_ws/install/cv_bridge/lib/libcv_bridge.so | grep opencv
```

