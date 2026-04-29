#!/usr/bin/env python3
"""
sensors.launch.py -- 启动所有传感器驱动节点
============================================

启动:
  - depthai-ros (OAK-D 相机, YOLO + 双目深度)
  - rplidar_ros (RPLIDAR 激光雷达)
  - mmw_radar_driver (毫米波雷达)

用法:
  ros2 launch adas_fusion sensors.launch.py
  ros2 launch adas_fusion sensors.launch.py enable_camera:=false
  ros2 launch adas_fusion sensors.launch.py lidar_port:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # ======================================================================
    # 相机参数 (depthai-ros OAK-D)
    # ======================================================================
    depthai_dir = get_package_share_directory('depthai_ros_driver')
    default_cfg = os.path.join(depthai_dir, 'config', 'yolo_spatial.yaml')

    enable_camera = LaunchConfiguration('enable_camera', default='true')
    camera_cfg = LaunchConfiguration('camera_cfg', default=default_cfg)
    camera_name = LaunchConfiguration('camera_name', default='oak')
    cam_pos_x = LaunchConfiguration('cam_pos_x', default='0.15')
    cam_pos_y = LaunchConfiguration('cam_pos_y', default='0.0')
    cam_pos_z = LaunchConfiguration('cam_pos_z', default='0.2')
    cam_roll = LaunchConfiguration('cam_roll', default='0.0')
    cam_pitch = LaunchConfiguration('cam_pitch', default='0.0')
    cam_yaw = LaunchConfiguration('cam_yaw', default='0.0')

    # ======================================================================
    # 激光雷达参数 (rplidar_ros)
    # ======================================================================
    enable_lidar = LaunchConfiguration('enable_lidar', default='true')
    # Jetson USB: /dev/ttyUSB0 或 /dev/ttyACM0
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')
    lidar_baud = LaunchConfiguration('lidar_baud', default='115200')
    lidar_frame = LaunchConfiguration('lidar_frame', default='laser')

    # ======================================================================
    # 毫米波雷达参数 (mmw_radar_driver)
    # ======================================================================
    enable_radar = LaunchConfiguration('enable_radar', default='true')
    radar_dir = get_package_share_directory('mmw_radar_driver')
    radar_params = os.path.join(radar_dir, 'config', 'radar_params.yaml')
    # 雷达使用 CH340 USB-UART, 在 Jetson 上通常为 /dev/ttyUSB1
    radar_port = LaunchConfiguration('radar_port', default='/dev/ttyUSB1')

    return LaunchDescription([

        # ---- Camera (depthai-ros compable node) ----
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('camera_cfg', default_value=default_cfg),
        DeclareLaunchArgument('camera_name', default_value='oak'),
        DeclareLaunchArgument('cam_pos_x', default_value='0.15'),
        DeclareLaunchArgument('cam_pos_y', default_value='0.0'),
        DeclareLaunchArgument('cam_pos_z', default_value='0.2'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),

        ComposableNodeContainer(
            condition=IfCondition(enable_camera),
            name='oak_container',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depthai_ros_driver',
                    plugin='depthai_ros_driver::Driver',
                    name=camera_name,
                    parameters=[camera_cfg, {
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
        ),

        # ---- LiDAR ----
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baud', default_value='115200'),
        DeclareLaunchArgument('lidar_frame', default_value='laser'),

        Node(
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
        ),

        # ---- mmWave Radar ----
        DeclareLaunchArgument('enable_radar', default_value='true'),
        DeclareLaunchArgument('radar_port', default_value='/dev/ttyUSB2'),

        Node(
            condition=IfCondition(enable_radar),
            package='mmw_radar_driver',
            executable='mmw_radar_node',
            name='mmw_radar_node',
            output='screen',
            parameters=[radar_params, {'serial_port': radar_port}],
        ),
    ])
