"""Launch file for mmw_radar_node.

Usage:
    ros2 launch mmw_radar_driver radar.launch.py
    ros2 launch mmw_radar_driver radar.launch.py serial_port:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mmw_radar_driver')
    default_params = os.path.join(pkg_dir, 'config', 'radar_params.yaml')

    params_file = LaunchConfiguration('params_file', default=default_params)
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to YAML parameters file'),

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for CH340 USB-UART adapter'),

        Node(
            package='mmw_radar_driver',
            executable='mmw_radar_node',
            name='mmw_radar_node',
            output='screen',
            parameters=[params_file, {'serial_port': serial_port}],
        ),
    ])
