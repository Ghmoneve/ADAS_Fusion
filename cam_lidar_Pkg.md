# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS 2 workspace containing two independent projects. There is no unified top-level build configuration -- each subproject is built and run separately.

## Projects

### 1. `depthai-ros` -- Luxonis OAK Stereo/RGB-D Camera Driver

A ROS 2 monorepo (version 3.1.0) that brings OAK camera data into ROS 2. It contains 7 ROS packages:

| Package | Purpose |
|---------|---------|
| `depthai_ros_msgs` | Custom ROS 2 msg/srv definitions (ImuWithMagneticField, SpatialDetection, HandLandmark, etc.) |
| `depthai_bridge` | Core C++ shared library that converts DepthAI SDK types to ROS 2 messages. Contains `BridgePublisher<RosMsg, DaiMsg>` template class |
| `depthai_ros_driver` | Main driver node (composable component). Creates `dai::Pipeline`, wraps sensors/nn nodes, publishes via `BridgePublisher`. Registered as a pluginlib plugin with pipeline types: RGB, RGBD, Stereo, Depth, CamArray, ToF, Thermal |
| `depthai_filters` | Composable post-processing nodes: Detection2DOverlay, SegmentationOverlay, WLSFilter, SpatialBB, etc. |
| `depthai_examples` | Standalone demonstrator executables (rgb_publisher, imu_publisher, odom_publisher, etc.) |
| `depthai_descriptions` | URDF/Xacro models and STL meshes for OAK camera bodies. Used by `robot_state_publisher` for TF publishing |
| `depthai-ros` | Metapackage; no source code, depends on all above |

**Architecture:** Data flows DepthAI SDK → `depthai_bridge` (converters) → `depthai_ros_driver` (driver node) → ROS 2 topics → `depthai_filters` (optional post-processing).

The driver supports calibration loading, dynamic parameter changes, RealSense topic-name compatibility, compressed transport (MJPEG, FFMPEG), and VI odometry.

**Build:**
```bash
cd depthai-ros
./build.sh                   # sequential Release with symlink-install (default)
./build.sh -s 0              # parallel build
./build.sh -r 1              # Debug (RelWithDebInfo)
./build.sh -t 1              # with tests enabled
```

Under the hood this wraps `colcon build` with `--symlink-install`, `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`, and `-DBUILD_SHARED_LIBS=ON`.

**Run:**
```bash
ros2 launch depthai_ros_driver driver.launch.py             # main driver with RVIZ
ros2 launch depthai_ros_driver rgbd_pcl.launch.py           # RGB-D with point cloud
ros2 launch depthai_ros_driver vio.launch.py                # visual-inertial odometry
ros2 launch depthai_filters example_det2d_overlay.launch.py # detection overlay
```

**Tests:**
```bash
./build.sh -t 1                   # build with -DBUILD_TESTING=ON -DTEST_DEPTHAI_ROS_DRIVER=ON
colcon test --packages-select depthai_bridge      # unit tests (GTest, 11 files)
colcon test --packages-select depthai_ros_driver  # integration tests (Python launch_testing, 11 files)
```

Integration tests require physical OAK hardware connected. They use a `TestHelper` class defined in `depthai_ros_driver/test_helper.py`.

**Docker:**
```bash
docker build -t depthai-ros --build-arg ROS_DISTRO=kilted .
```
The Dockerfile builds from `ros:<distro>-ros-base`, clones `depthai-core` (tag `ros-v3.2.1`), and runs `build.sh`. CI runs hardware-in-the-loop tests on self-hosted runners with OAK-D / OAK4 hardware.

### 2. `rplidar_ros` -- Slamtec RPLIDAR Laser Scanner Driver

A single ROS 2 package (version 2.1.0) that publishes `sensor_msgs/msg/LaserScan` from RPLIDAR A1/A2/A3/S1 sensors.

**Architecture:** A single composable node (`rplidar_ros::rplidar_node`) that wraps the vendored RPLIDAR SDK (v1.12.0, in `sdk/`). A 1ms wall timer drives `publish_loop()` which grabs full 360-degree scan data, optionally applies angle compensation, and publishes `LaserScan` messages.

Key features:
- Supports serial and TCP channel types
- Auto-standby mode: motor starts/stops based on subscriber count
- Angle compensation for uniform 360-degree output
- `flip_x_axis` for alternative mounting orientations
- Registered as a composable node -- can be loaded into a component container

**Build:**
```bash
cd <colcon_workspace>
colcon build --symlink-install --packages-select rplidar_ros
```

**Run:**
```bash
ros2 launch rplidar_ros view_rplidar.launch.py      # A1/A2 with RViz
ros2 launch rplidar_ros view_rplidar_a3.launch.py   # A3 with RViz
ros2 launch rplidar_ros view_rplidar_s1.launch.py   # S1 with RViz
ros2 launch rplidar_ros rplidar.launch.py            # headless A1/A2
ros2 run rplidar_ros rplidar_composition             # standalone executable
```

Key parameters: `serial_port` (default `/dev/ttyUSB0`), `serial_baudrate`, `frame_id`, `scan_mode`, `channel_type`, `auto_standby`.

**Tests:** Built into the SDK itself; no separate ROS test suite. A `test_rplidar_a3.launch.py` exists but references a `rplidarNodeClient` executable not in the current CMakeLists.txt (likely stale).

## General Notes

- Both projects use ROS 2 `ament_cmake` build system and `colcon` as the build tool.
- The workspace follows the standard colcon `src/` layout convention despite having no top-level `src/` directory. When building, clone into or symlink to `<workspace>/src/`.
- No pre-commit hooks, linters, or formatters are configured at the workspace level. CI linting (clang-format, xmllint, pep257, lint_cmake) exists only in `depthai-ros/.github/workflows/`.
- The `.gitignore` at `depthai-ros/` level covers common ROS build artifacts (`build/`, `__pycache__`, `.DS_Store`).
