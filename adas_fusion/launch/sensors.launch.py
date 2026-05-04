#!/usr/bin/env python3
"""
sensors.launch.py -- 启动所有传感器驱动节点
============================================

启动:
  - depthai-ros (OAK-D 相机, YOLO + 双目深度)  [需要 libdepthai]
  - rplidar_ros (RPLIDAR 激光雷达)
  - mmw_radar_driver (毫米波雷达)

用法:
  ros2 launch adas_fusion sensors.launch.py
  ros2 launch adas_fusion sensors.launch.py enable_camera:=false
  ros2 launch adas_fusion sensors.launch.py lidar_port:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory

try:
    from ament_index_python.packages import PackageNotFoundError
except ImportError:
    # 兼容旧版 ament_index_python (< 1.4.0)
    PackageNotFoundError = LookupError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _try_get_camera_actions(enable_camera, camera_cfg, camera_name,
                            cam_pos_x, cam_pos_y, cam_pos_z,
                            cam_roll, cam_pitch, cam_yaw):
    """
    尝试创建相机启动动作。如果 depthai_ros_driver 未安装, 返回警告信息。
    """
    actions = [
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('camera_cfg', default_value=''),
        DeclareLaunchArgument('camera_name', default_value='oak'),
        DeclareLaunchArgument('cam_pos_x', default_value='0.15'),
        DeclareLaunchArgument('cam_pos_y', default_value='0.0'),
        DeclareLaunchArgument('cam_pos_z', default_value='0.2'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
    ]

    try:
        depthai_dir = get_package_share_directory('depthai_ros_driver')
        default_cfg = os.path.join(depthai_dir, 'config', 'yolo_spatial.yaml')
    except PackageNotFoundError:
        actions.append(LogInfo(
            msg='[sensors] depthai_ros_driver not found — camera disabled'))
        return actions

    container = ComposableNodeContainer(
        condition=IfCondition(enable_camera),
        name='oak_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depthai_ros_driver',
                plugin='depthai_ros_driver::Driver',
                name=camera_name,
                parameters=[camera_cfg if camera_cfg else default_cfg, {
                    'pipeline_gen.i_pipeline_type': 'RGBD',
                    'pipeline_gen.i_nn_type': 'spatial',
                    'driver.i_publish_tf_from_calibration': True,
                    'driver.i_tf_tf_prefix': 'oak',
                    'driver.i_tf_base_frame': 'oak',
                    'driver.i_tf_parent_frame': 'oak_parent_frame',
                    'driver.i_tf_cam_pos_x': cam_pos_x,
                    'driver.i_tf_cam_pos_y': cam_pos_y,
                    'driver.i_tf_cam_pos_z': cam_pos_z,
                    'driver.i_tf_cam_roll': cam_roll,
                    'driver.i_tf_cam_pitch': cam_pitch,
                    'driver.i_tf_cam_yaw': cam_yaw,
                }],
            ),
        ],
        output='screen',
    )
    actions.append(container)
    return actions


def _try_get_lidar_actions(enable_lidar, lidar_port, lidar_baud, lidar_frame):
    """尝试创建 LiDAR 启动动作。"""
    actions = [
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baud', default_value='115200'),
        DeclareLaunchArgument('lidar_frame', default_value='laser'),
    ]

    try:
        # 检查 rplidar_ros 是否存在
        get_package_share_directory('rplidar_ros')
    except PackageNotFoundError:
        actions.append(LogInfo(
            msg='[sensors] rplidar_ros not found — LiDAR disabled'))
        return actions

    actions.append(Node(
        condition=IfCondition(enable_lidar),
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': lidar_baud,
            'frame_id': lidar_frame,
            'inverted': False,
            'angle_compensate': True,
        }],
    ))
    return actions


def generate_launch_description():

    # ---- Launch Configurations ----
    enable_camera = LaunchConfiguration('enable_camera', default='false')
    camera_cfg = LaunchConfiguration('camera_cfg', default='')
    camera_name = LaunchConfiguration('camera_name', default='oak')
    cam_pos_x = LaunchConfiguration('cam_pos_x', default='0.15')
    cam_pos_y = LaunchConfiguration('cam_pos_y', default='0.0')
    cam_pos_z = LaunchConfiguration('cam_pos_z', default='0.2')
    cam_roll = LaunchConfiguration('cam_roll', default='0.0')
    cam_pitch = LaunchConfiguration('cam_pitch', default='0.0')
    cam_yaw = LaunchConfiguration('cam_yaw', default='0.0')

    enable_lidar = LaunchConfiguration('enable_lidar', default='true')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')
    lidar_baud = LaunchConfiguration('lidar_baud', default='115200')
    lidar_frame = LaunchConfiguration('lidar_frame', default='laser')

    enable_radar = LaunchConfiguration('enable_radar', default='true')
    radar_port = LaunchConfiguration('radar_port', default='/dev/ttyUSB1')

    # ---- 构建 LaunchDescription ----
    ld = LaunchDescription()

    # Camera (graceful skip if package missing)
    ld.add_action(LogInfo(msg='[sensors] Starting sensor drivers...'))
    for action in _try_get_camera_actions(enable_camera, camera_cfg,
                                          camera_name, cam_pos_x, cam_pos_y,
                                          cam_pos_z, cam_roll, cam_pitch,
                                          cam_yaw):
        ld.add_action(action)

    # LiDAR (graceful skip)
    for action in _try_get_lidar_actions(enable_lidar, lidar_port,
                                         lidar_baud, lidar_frame):
        ld.add_action(action)

    # mmWave Radar
    ld.add_action(DeclareLaunchArgument('enable_radar', default_value='true'))
    ld.add_action(DeclareLaunchArgument('radar_port', default_value='/dev/ttyUSB1'))

    try:
        radar_dir = get_package_share_directory('mmw_radar_driver')
        radar_params = os.path.join(radar_dir, 'config', 'radar_params.yaml')
        ld.add_action(Node(
            condition=IfCondition(enable_radar),
            package='mmw_radar_driver',
            executable='mmw_radar_node',
            name='mmw_radar_node',
            output='screen',
            parameters=[radar_params, {'serial_port': radar_port}],
        ))
    except PackageNotFoundError:
        ld.add_action(LogInfo(
            msg='[sensors] mmw_radar_driver not found — radar disabled'))

    ld.add_action(LogInfo(msg='[sensors] Sensor drivers ready'))
    return ld
