#!/usr/bin/env python3
"""
system.launch.py -- 完整系统启动文件 (Jetson ORIN NX)
====================================================

启动全部节点:
  传感器驱动 → 适配器 → 融合 → 决策 → 串口桥接 → STM32

用法:
  ros2 launch adas_fusion system.launch.py
  ros2 launch adas_fusion system.launch.py max_linear_vel:=0.5 ttc_emergency:=0.8
  ros2 launch adas_fusion system.launch.py enable_radar:=false
  ros2 launch adas_fusion system.launch.py serial_port:=/dev/ttyTHS2

可传参数:
  -- 传感器使能 --
  enable_camera:   true/false (default: true)
  enable_lidar:    true/false (default: true)
  enable_radar:    true/false (default: true)

  -- 传感器端口 (Jetson 设备路径) --
  lidar_port:      RPLIDAR USB 端口 (default: /dev/ttyUSB0)
  radar_port:      毫米波雷达 USB-UART (default: /dev/ttyUSB1)
  serial_port:     STM32 串口 (default: /dev/ttyTHS2, Jetson USART3)

  -- 相机 --
  camera_model:    OAK-D-PRO / OAK-D / OAK-D-LITE
  nn_type:         spatial (YOLO+双目深度) / rgb / none
  camera_name:     相机名 (决定 depthai topic 前缀, default: oak)

  -- 串口桥接 --
  serial_baud:     STM32 波特率 (default: 115200)

  -- TTC 阈值 --
  ttc_warning:     预警阈值秒 (default: 5.0)
  ttc_slowdown:    减速阈值秒 (default: 3.0)
  ttc_emergency:   紧急停车阈值秒 (default: 1.0)

  -- 速度限制 --
  max_linear_vel:  最大线速度 m/s (default: 0.3)
  max_angular_vel: 最大角速度 rad/s (default: 0.5)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('adas_fusion')

    return LaunchDescription([

        # ==================================================================
        # 声明全局参数
        # ==================================================================
        DeclareLaunchArgument('enable_camera', default_value='false',
                              description='Enable OAK-D camera (requires libdepthai + hardware)'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_radar', default_value='true'),

        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('radar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyTHS2'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),

        DeclareLaunchArgument('camera_model', default_value='OAK-D-PRO'),
        DeclareLaunchArgument('camera_name', default_value='oak'),
        DeclareLaunchArgument('nn_type', default_value='spatial'),

        DeclareLaunchArgument('ttc_warning', default_value='5.0'),
        DeclareLaunchArgument('ttc_slowdown', default_value='3.0'),
        DeclareLaunchArgument('ttc_emergency', default_value='1.0'),

        DeclareLaunchArgument('max_linear_vel', default_value='0.3'),
        DeclareLaunchArgument('max_angular_vel', default_value='0.5'),

        DeclareLaunchArgument('enable_joystick', default_value='true',
                              description='Enable joystick gamepad control'),
        DeclareLaunchArgument('cooldown_seconds', default_value='3.0',
                              description='Emergency recovery cooldown (seconds)'),

        # ==================================================================
        # 1. 传感器驱动
        # ==================================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'sensors.launch.py')
            ),
            launch_arguments={
                'enable_camera': LaunchConfiguration('enable_camera'),
                'enable_lidar': LaunchConfiguration('enable_lidar'),
                'enable_radar': LaunchConfiguration('enable_radar'),
                'camera_model': LaunchConfiguration('camera_model'),
                'camera_name': LaunchConfiguration('camera_name'),
                'nn_type': LaunchConfiguration('nn_type'),
                'lidar_port': LaunchConfiguration('lidar_port'),
                'radar_port': LaunchConfiguration('radar_port'),
            }.items(),
        ),

        # ==================================================================
        # 2. 适配器 + 融合 + 决策 + 串口桥接
        # ==================================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'fusion_decision.launch.py')
            ),
            launch_arguments={
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baud': LaunchConfiguration('serial_baud'),
                'ttc_warning': LaunchConfiguration('ttc_warning'),
                'ttc_slowdown': LaunchConfiguration('ttc_slowdown'),
                'ttc_emergency': LaunchConfiguration('ttc_emergency'),
                'max_linear_vel': LaunchConfiguration('max_linear_vel'),
                'max_angular_vel': LaunchConfiguration('max_angular_vel'),
                'enable_joystick': LaunchConfiguration('enable_joystick'),
                'cooldown_seconds': LaunchConfiguration('cooldown_seconds'),
            }.items(),
        ),

        LogInfo(msg='[ADAS Fusion] System started successfully!'),
    ])
