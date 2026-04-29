#!/usr/bin/env python3
"""
radar_adapter.py -- 毫米波雷达适配器
============================

功能：
  - 订阅 mmw_radar_node 发布的 RadarTargetArray (极坐标)
  - 将极坐标 (range, angle) 转换为笛卡尔坐标 (x, y)
  - 将径向速度分解为 vx, vy 分量
  - 发布 RadarObjectArray 到 /radar_objects

输入：
  - /radar/targets (RadarTargetArray, 极坐标)

输出：
  - /radar_objects (RadarObjectArray, 笛卡尔坐标)

转换公式：
  x  = range * cos(angle_rad)        (x 轴指向雷达正前方)
  y  = range * sin(angle_rad)        (y 轴指向雷达左侧)
  vx = velocity * cos(angle_rad)     (径向速度 → x 分量)
  vy = velocity * sin(angle_rad)     (径向速度 → y 分量)

注意：
  雷达的径向速度只反映目标沿视线方向的运动，切向速度未知。
  vx, vy 分解仅作为近似值，实际切向速度通过 Kalman Filter 估计。
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Header

from mmw_radar_msgs.msg import RadarTargetArray
from adas_fusion_msgs.msg import RadarObject, RadarObjectArray


class RadarAdapter(Node):
    """将 mmWave 雷达极坐标转换为笛卡尔坐标。"""

    def __init__(self):
        super().__init__('radar_adapter')

        self.declare_parameter('radar_topic', '/radar/targets')
        self.declare_parameter('publish_topic', '/radar_objects')
        self.declare_parameter('max_range', 15.0)      # 最大有效距离 (m)
        self.declare_parameter('min_confidence', 0.1)  # 最小置信度阈值

        radar_topic = self.get_parameter('radar_topic').value
        pub_topic = self.get_parameter('publish_topic').value

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._sub = self.create_subscription(
            RadarTargetArray, radar_topic, self._radar_cb, qos)

        self._pub = self.create_publisher(RadarObjectArray, pub_topic, 10)

        self.get_logger().info(f'RadarAdapter: {radar_topic} -> {pub_topic}')

    def _radar_cb(self, msg: RadarTargetArray):
        max_range = self.get_parameter('max_range').value
        min_conf = self.get_parameter('min_confidence').value

        out = RadarObjectArray()
        out.header = Header(
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id,
        )

        for t in msg.targets:
            if t.range > max_range or t.range <= 0.0:
                continue

            angle_rad = math.radians(t.angle)

            obj = RadarObject()
            # 极坐标 → 笛卡尔坐标 (x=前, y=左)
            obj.position.x = t.range * math.cos(angle_rad)
            obj.position.y = t.range * math.sin(angle_rad)
            obj.position.z = 0.0

            # 径向速度 → 笛卡尔速度分量 (近似)
            obj.vx = t.velocity * math.cos(angle_rad)
            obj.vy = t.velocity * math.sin(angle_rad)

            # 置信度估算 (基于雷达检测参数)
            obj.confidence = max(min_conf, 1.0 - (t.range / max_range))

            out.objects.append(obj)

        if out.objects:
            self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RadarAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
