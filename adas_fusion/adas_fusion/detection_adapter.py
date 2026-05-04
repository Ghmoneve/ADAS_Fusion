#!/usr/bin/env python3
"""
detection_adapter.py -- 视觉检测适配器
============================

功能：
  - 订阅 depthai-ros 发布的 SpatialDetectionArray（OAK-D 内置 YOLO + 双目深度）
  - 可选订阅深度图，对每个检测框取深度
  - 转换为统一的 Detection2DArray 格式发布到 /detections

输入：
  - /color/yolov4_Spatial_detections (SpatialDetectionArray)
  - /stereo/depth (Image，可选)

输出：
  - /detections (Detection2DArray)

设计说明:
  该节点不修改 depthai-ros 包，仅做消息格式适配。
  深度图模式默认不启用，因为 OAK-D 的 SpatialDetection 已自带 3D position。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from adas_fusion_msgs.msg import Detection2D, Detection2DArray

try:
    from depthai_ros_msgs.msg import SpatialDetectionArray
    HAS_DEPTHAI_MSGS = True
except ImportError:
    HAS_DEPTHAI_MSGS = False
    SpatialDetectionArray = None


class DetectionAdapter(Node):
    """将 depthai-ros SpatialDetectionArray 转换为 Detection2DArray。"""

    def __init__(self):
        super().__init__('detection_adapter')

        if not HAS_DEPTHAI_MSGS:
            self.get_logger().fatal(
                'depthai_ros_msgs not available. '
                'Build depthai-ros first or install depthai_ros_msgs.')
            raise RuntimeError('depthai_ros_msgs required')

        # ---- 参数 ----
        self.declare_parameter('depthai_detection_topic',
                               '/color/yolov4_Spatial_detections')
        self.declare_parameter('depthai_depth_topic', '/stereo/depth')
        self.declare_parameter('use_depth_image', False)
        self.declare_parameter('publish_topic', '/detections')

        det_topic = self.get_parameter('depthai_detection_topic').value
        pub_topic = self.get_parameter('publish_topic').value

        # ---- 订阅 ----
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._det_sub = self.create_subscription(
            SpatialDetectionArray, det_topic, self._det_cb, qos)

        # ---- 发布 ----
        self._pub = self.create_publisher(Detection2DArray, pub_topic, 10)

        self.get_logger().info(
            f'DetectionAdapter: {det_topic} -> {pub_topic}')

    def _det_cb(self, msg: SpatialDetectionArray):
        """将 SpatialDetectionArray 转换为 Detection2DArray。"""
        out = Detection2DArray()
        out.header = msg.header

        for sd in msg.detections:
            d = Detection2D()
            # BoundingBox2D: center 是 Pose2D (float64 x, y, theta)
            d.x = max(0, int(sd.bbox.center.x - sd.bbox.size_x / 2.0))
            d.y = max(0, int(sd.bbox.center.y - sd.bbox.size_y / 2.0))
            d.width = int(sd.bbox.size_x)
            d.height = int(sd.bbox.size_y)
            # ObjectHypothesis: class_id 是 string, score 是 float64
            if sd.results:
                cls_str = sd.results[0].hypothesis.class_id
                try:
                    d.class_id = int(cls_str)
                except (ValueError, TypeError):
                    d.class_id = -1
                d.confidence = float(sd.results[0].hypothesis.score)
            else:
                d.class_id = -1
                d.confidence = 0.0
            d.depth = sd.position.z if sd.position.z > 0.0 else -1.0
            d.position = sd.position
            out.detections.append(d)

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
