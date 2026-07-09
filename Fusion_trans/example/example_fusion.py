#!/usr/bin/env python3
"""
example_fusion.py — 最小可运行示例

展示如何:
  1. 注册三种传感器 (camera, lidar, radar)
  2. 模拟真实数据输入 (由集成方替换为真实传感器回调)
  3. 周期性调用 engine.step() 获取融合跟踪结果
  4. 发布 ROS 2 TrackedObjectArray

运行:
  python3 example_fusion.py
"""

import math
import time
import threading
import numpy as np

# --- fusion_core (零 ROS 依赖) ---
from fusion_core import FusionEngine, FusionConfig, Observation

# --- ROS 2 (仅本示例需要) ---
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Header
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


class FusionDemoNode(Node if HAS_ROS else object):
    """演示: 注册传感器 → 喂入观测 → 获取跟踪结果。"""

    def __init__(self):
        if HAS_ROS:
            Node.__init__(self, 'fusion_demo')

        # ---- 1. 配置融合引擎 ----
        config = FusionConfig(
            dt=0.1,
            process_noise_q=0.5,
            association_gate=3.0,
            chi2_threshold=5.991,
            time_window=0.1,
        )
        self.engine = FusionEngine(config)

        # ---- 2. 注册传感器 ----
        # LiDAR: 预置模型, 无需写 noise_model 回调
        self.engine.register_sensor(
            "lidar", type="lidar", sigma_0=0.03, n_ref=30,
            frame_id="laser")

        # Camera: 预置模型
        self.engine.register_sensor(
            "camera", type="camera", sigma_0=0.05, sigma_c=5.0,
            frame_id="camera_optical_frame")

        # Radar: 预置模型
        self.engine.register_sensor(
            "radar", type="radar", sigma_0=0.2, alpha=0.5, v_ref=10.0,
            frame_id="radar_link")

        # ---- 4. 周期步进 ----
        self._run = True
        self._timer = threading.Thread(target=self._loop, daemon=True)
        self._timer.start()

    def _loop(self):
        while self._run:
            tracks = self.engine.step()
            for t in tracks:
                if t.status == 'CONFIRMED':
                    print(f"  Track #{t.id}: pos=({t.position[0]:.2f},{t.position[1]:.2f}) "
                          f"vel=({t.velocity[0]:.2f},{t.velocity[1]:.2f}) "
                          f"conf={t.confidence:.2f} src={t.source_mask:03b}")
            time.sleep(0.1)

    def on_lidar(self, range_m, angle_rad, cluster_size=15):
        """LiDAR 数据回调 (由集成方实现)。"""
        x = range_m * math.cos(angle_rad)
        y = range_m * math.sin(angle_rad)
        self.engine.add_observation(
            "lidar", position=[x, y], velocity=[0, 0],
            confidence=min(1.0, cluster_size / 30.0),
            cluster_size=cluster_size)

    def on_camera(self, x_m, y_m, z_m, confidence=0.8):
        """相机检测回调 (由集成方实现)。"""
        dist = math.sqrt(x_m * x_m + z_m * z_m)
        self.engine.add_observation(
            "camera", position=[x_m, z_m], velocity=[0, 0],
            confidence=confidence, dist=dist)

    def on_radar(self, x_m, y_m, vx, vy, confidence=0.7):
        """雷达检测回调 (由集成方实现)。"""
        v_radial = math.sqrt(vx * vx + vy * vy)
        self.engine.add_observation(
            "radar", position=[x_m, y_m], velocity=[vx, vy],
            confidence=confidence, v_radial=v_radial)

    def shutdown(self):
        self._run = False


if __name__ == '__main__':
    node = FusionDemoNode()
    print("Fusion demo running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        node.shutdown()
