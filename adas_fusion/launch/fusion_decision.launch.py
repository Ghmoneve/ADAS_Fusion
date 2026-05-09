#!/usr/bin/env python3
"""
fusion_decision.launch.py -- 启动适配器 + 融合 + 决策 + 串口桥接
=============================================================

数据流:
  depthai SpatialDetectionArray → detection_adapter → /detections ──┐
  mmw RadarTargetArray          → radar_adapter      → /radar_objects ┤
  rplidar LaserScan              → (直接)            → /scan ────────┤
  Joystick                       → /joy ─────────────────────────────┤
                                                                     ↓
                                                                fusion_node
                                                                     ↓
                                                              /tracked_objects
                                                                     ↓
                                                               decision_node
                                                              (joystick + 紧急接管)
                                                                     ↓
                                                                 /cmd_vel
                                                                     ↓
                                                              serial_bridge
                                                                     ↓
                                                           STM32 (UART)

用法:
  ros2 launch adas_fusion fusion_decision.launch.py
  ros2 launch adas_fusion fusion_decision.launch.py enable_joystick:=true
  ros2 launch adas_fusion fusion_decision.launch.py serial_port:=/dev/ttyTHS2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('adas_fusion')
    default_params = os.path.join(pkg_dir, 'config', 'fusion_params.yaml')

    params_file = LaunchConfiguration('params_file', default=default_params)

    # ---- 传感器 Topic 重映射 ----
    depthai_det_topic = LaunchConfiguration(
        'depthai_det_topic', default='/oak/color/yolov4_Spatial_detections')
    detection_topic = LaunchConfiguration(
        'detection_topic', default='/detections')
    radar_topic = LaunchConfiguration(
        'radar_topic', default='/radar/targets')
    radar_objects_topic = LaunchConfiguration(
        'radar_objects_topic', default='/radar_objects')
    scan_topic = LaunchConfiguration(
        'scan_topic', default='/scan')
    tracked_topic = LaunchConfiguration(
        'tracked_topic', default='/tracked_objects')
    cmd_vel_topic = LaunchConfiguration(
        'cmd_vel_topic', default='/cmd_vel')

    # ---- 手柄参数 ----
    enable_joy = LaunchConfiguration('enable_joystick', default='true')
    joy_topic = LaunchConfiguration('joy_topic', default='/joy')

    # ---- TTC / 决策参数 (用于 system.launch.py 透传) ----
    ttc_warning = LaunchConfiguration('ttc_warning', default='5.0')
    ttc_slowdown = LaunchConfiguration('ttc_slowdown', default='3.0')
    ttc_emergency = LaunchConfiguration('ttc_emergency', default='1.0')
    max_linear_vel = LaunchConfiguration('max_linear_vel', default='0.3')
    max_angular_vel = LaunchConfiguration('max_angular_vel', default='0.5')
    cooldown_seconds = LaunchConfiguration('cooldown_seconds', default='3.0')

    # ---- 毫米波雷达模式 ----
    enable_radar_sim = LaunchConfiguration(
        'enable_radar_sim', default='true')

    # ---- 串口桥接参数 ----
    serial_port = LaunchConfiguration(
        'serial_port', default='/dev/ttyTHS2')
    serial_baud = LaunchConfiguration(
        'serial_baud', default='115200')

    return LaunchDescription([

        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Path to YAML params file'),
        DeclareLaunchArgument('depthai_det_topic',
                              default_value='/oak/color/yolov4_Spatial_detections'),
        DeclareLaunchArgument('detection_topic', default_value='/detections'),
        DeclareLaunchArgument('radar_topic', default_value='/radar/targets'),
        DeclareLaunchArgument('radar_objects_topic',
                              default_value='/radar_objects'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('tracked_topic',
                              default_value='/tracked_objects'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('enable_joystick', default_value='true',
                              description='Enable joystick control'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyTHS2',
                              description='Jetson UART device for STM32'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('enable_radar_sim', default_value='true',
                              description='Use simulated radar (derived from fusion). Set false for real mmWave hardware.'),
        DeclareLaunchArgument('ttc_warning', default_value='5.0'),
        DeclareLaunchArgument('ttc_slowdown', default_value='3.0'),
        DeclareLaunchArgument('ttc_emergency', default_value='1.0'),
        DeclareLaunchArgument('max_linear_vel', default_value='0.3'),
        DeclareLaunchArgument('max_angular_vel', default_value='0.5'),
        DeclareLaunchArgument('cooldown_seconds', default_value='3.0'),

        # ---- 手柄驱动 (标准 ROS 2 joy 包) ----
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }],
        ),

        # ---- 视觉适配器 ----
        Node(
            package='adas_fusion',
            executable='detection_adapter',
            name='detection_adapter',
            output='screen',
            parameters=[params_file, {
                'depthai_detection_topic': depthai_det_topic,
                'publish_topic': detection_topic,
            }],
        ),

        # ---- 毫米波雷达: 模拟器 (默认) 或真实适配器 ----
        Node(
            package='adas_fusion',
            executable='radar_simulator',
            name='radar_simulator',
            output='screen',
            condition=IfCondition(enable_radar_sim),
            parameters=[params_file, {
                'tracked_topic': tracked_topic,
                'radar_objects_topic': radar_objects_topic,
            }],
        ),
        Node(
            package='adas_fusion',
            executable='radar_adapter',
            name='radar_adapter',
            output='screen',
            condition=UnlessCondition(enable_radar_sim),
            parameters=[params_file, {
                'radar_topic': radar_topic,
                'publish_topic': radar_objects_topic,
            }],
        ),

        # ---- 融合节点 ----
        Node(
            package='adas_fusion',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[params_file, {
                'detection_topic': detection_topic,
                'scan_topic': scan_topic,
                'radar_topic': radar_objects_topic,
                'tracked_objects_topic': tracked_topic,
            }],
        ),

        # ---- 决策节点 (手柄 + 紧急接管) ----
        Node(
            package='adas_fusion',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[params_file, {
                'tracked_objects_topic': tracked_topic,
                'cmd_vel_topic': cmd_vel_topic,
                'joy_topic': joy_topic,
                'enable_joystick': enable_joy,
                'ttc_warning': ttc_warning,
                'ttc_slowdown': ttc_slowdown,
                'ttc_emergency': ttc_emergency,
                'max_linear_vel': max_linear_vel,
                'max_angular_vel': max_angular_vel,
                'cooldown_seconds': cooldown_seconds,
            }],
        ),

        # ---- 串口桥接 (Jetson → STM32) ----
        Node(
            package='adas_fusion',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[params_file, {
                'serial_port': serial_port,
                'baud_rate': serial_baud,
                'cmd_vel_topic': cmd_vel_topic,
            }],
        ),
    ])
