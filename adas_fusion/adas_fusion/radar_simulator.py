#!/usr/bin/env python3
"""
radar_simulator.py -- 毫米波雷达数据模拟器
==========================================

从融合跟踪目标反算雷达数据, 添加雷达特征噪声后发布,
使下游节点 (fusion_node, data_collector) 感知如同真实雷达在运行。

数据流:
  /tracked_objects (TrackedObjectArray)  →  反算  →  /radar_objects (RadarObjectArray)
                                                  →  /radar/data   (String)

反算公式:
  distance   = sqrt(px² + py²)                   # 径向距离
  bearing    = atan2(py, px)                     # 方位角 (rad → deg)
  velocity   = (vx·px + vy·py) / distance        # 径向速度

噪声模型 (体现雷达传感器特征):
  σ_position ≈ 0.2 m     (雷达测距不如 LiDAR)
  σ_bearing  ≈ 2°        (雷达角分辨率较低)
  σ_velocity ≈ 0.1 m/s   (雷达测速优势, 噪声较小)
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Header, String

from adas_fusion_msgs.msg import RadarObject, RadarObjectArray, TrackedObjectArray


class RadarSimulator(Node):
    """从 /tracked_objects 反算雷达数据, 加噪后发布。"""

    def __init__(self):
        super().__init__('radar_simulator')

        self.declare_parameter('tracked_topic', '/tracked_objects')
        self.declare_parameter('radar_objects_topic', '/radar_objects')
        self.declare_parameter('radar_data_topic', '/radar/data')
        self.declare_parameter('noise_position', 0.2)
        self.declare_parameter('noise_velocity', 0.1)
        self.declare_parameter('noise_bearing_deg', 2.0)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('min_confidence', 0.65)

        self._sigma_pos = self.get_parameter('noise_position').value
        self._sigma_vel = self.get_parameter('noise_velocity').value
        self._sigma_bearing = math.radians(
            self.get_parameter('noise_bearing_deg').value)
        self._max_range = self.get_parameter('max_range').value
        self._min_conf = self.get_parameter('min_confidence').value

        # Subscriptions
        tracked_topic = self.get_parameter('tracked_topic').value
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self._sub = self.create_subscription(
            TrackedObjectArray, tracked_topic, self._callback, qos)

        # Publishers
        self._pub_objects = self.create_publisher(
            RadarObjectArray,
            self.get_parameter('radar_objects_topic').value, 10)
        self._pub_data = self.create_publisher(
            String,
            self.get_parameter('radar_data_topic').value, 10)

        self.get_logger().info(
            f'RadarSimulator: {tracked_topic} -> '
            f'{self.get_parameter("radar_objects_topic").value} '
            f'(sigma_pos={self._sigma_pos}m, sigma_vel={self._sigma_vel}m/s)')

    def _callback(self, msg: TrackedObjectArray):
        now = self.get_clock().now()
        out = RadarObjectArray()
        out.header = Header(stamp=now.to_msg(), frame_id='radar_link')

        # 收集所有目标的雷达反算数据
        radar_lines = []
        for obj in msg.objects:
            px = obj.position.x
            py = obj.position.y
            vx = obj.vx
            vy = obj.vy

            dist = math.sqrt(px * px + py * py)
            if dist < 0.01 or dist > self._max_range:
                continue

            # 加噪声
            noisy_dist = dist + np.random.normal(0.0, self._sigma_pos)
            noisy_dist = max(0.05, noisy_dist)

            true_bearing = math.atan2(py, px)
            noisy_bearing = true_bearing + np.random.normal(0.0, self._sigma_bearing)

            # 径向速度 (正 = 接近)
            radial_vel = -(vx * px + vy * py) / dist
            noisy_radial_vel = radial_vel + np.random.normal(0.0, self._sigma_vel)

            # 置信度随距离衰减
            confidence = max(self._min_conf,
                             0.9 - 0.02 * noisy_dist)

            # 反算含噪位置
            noisy_px = noisy_dist * math.cos(noisy_bearing)
            noisy_py = noisy_dist * math.sin(noisy_bearing)

            ro = RadarObject()
            ro.position.x = noisy_px
            ro.position.y = noisy_py
            ro.position.z = 0.0
            ro.vx = noisy_radial_vel * math.cos(noisy_bearing)
            ro.vy = noisy_radial_vel * math.sin(noisy_bearing)
            ro.confidence = float(confidence)
            out.objects.append(ro)

            # 收集供 String 发布
            bearing_deg = math.degrees(noisy_bearing)
            radar_lines.append(
                f'{bearing_deg:.3f},{noisy_dist:.4f},{noisy_radial_vel:.3f}')

        if out.objects:
            self._pub_objects.publish(out)

        # String 格式: 多条数据用 "; " 分隔, 匹配 data_collector 的逐行写入
        if radar_lines:
            data_msg = String()
            data_msg.data = radar_lines[0]  # 主目标 (最近/最危险)
            self._pub_data.publish(data_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RadarSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
