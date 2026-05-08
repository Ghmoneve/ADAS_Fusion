#!/usr/bin/env python3
"""
data_collector.py — ROS2 data collection node for ADAS Fusion experiments.

Subscribes to camera, LiDAR, radar, fusion, decision, and odom topics.
Records CSV data at 10 Hz and captures camera keyframes on risk-level changes.

Usage (via collect.sh):
    ros2 run adas_fusion data_collector.py \
        --scene 1 --run 1 --duration 120 --odom_topic /odom
"""

import argparse
import csv
import os
import signal
import sys
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# ── Message imports (try both common package layouts) ────────────────────
try:
    from sensor_msgs.msg import Image, LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Float32
    from cv_bridge import CvBridge
    import cv2
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False
    Image = None
    Odometry = None
    LaserScan = None
    String = None
    Float32 = None


class DataCollector(Node):
    """Subscribe to sensor & algorithm topics and record structured CSV data."""

    def __init__(self, scene: int, run_id: int, odom_topic: str):
        super().__init__("data_collector")

        self.scene = scene
        self.run_id = run_id
        self.odom_topic = odom_topic

        # ── Output directory ─────────────────────────────────────────────
        self.output_dir = Path(__file__).resolve().parent / "raw_data" / f"scene_{scene}" / f"run_{run_id}"
        self.keyframe_dir = self.output_dir / "camera_keyframes"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.keyframe_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Output directory: {self.output_dir}")

        # ── CSV writers & file handles ───────────────────────────────────
        self._open_csv_files()

        # ── In-memory buffers (decouple callback from main thread) ───────
        self._odom_buffer = deque(maxlen=10)
        self._lidar_buffer = deque(maxlen=10)
        self._radar_buffer = deque(maxlen=10)
        self._fusion_buffer = deque(maxlen=10)
        self._ttc_buffer = deque(maxlen=10)

        # ── Camera state ─────────────────────────────────────────────────
        self._last_risk_level = None
        self._bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None
        self._latest_camera_frame = None
        self._camera_frame_received = False

        # ── Experiment metadata ──────────────────────────────────────────
        self._start_time = time.time()
        self._last_record_time = self._start_time

        # ── QoS ──────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ── Subscriptions ────────────────────────────────────────────────
        self._sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, sensor_qos)
        self._sub_lidar = self.create_subscription(
            LaserScan, "/scan", self._lidar_callback, sensor_qos)

        # Radar topics (millimetre-wave radar via serial bridge)
        self._sub_radar = self.create_subscription(
            String, "/radar/data", self._radar_callback, reliable_qos)

        # Fusion output
        self._sub_fusion = self.create_subscription(
            String, "/fusion/tracked_objects", self._fusion_callback, reliable_qos)

        # Decision output — TTC & risk level
        self._sub_ttc = self.create_subscription(
            String, "/decision/ttc", self._ttc_callback, reliable_qos)

        # Camera
        if CV_BRIDGE_AVAILABLE:
            self._sub_camera = self.create_subscription(
                Image, "/oak/rgb/image_raw", self._camera_callback, sensor_qos)
        else:
            self.get_logger().warn("cv_bridge not available — camera keyframes disabled")

        # ── 10 Hz record timer ───────────────────────────────────────────
        self._record_timer = self.create_timer(0.1, self._record_tick)

        self.get_logger().info("DataCollector initialized — recording at 10 Hz")

    # ── CSV file management ──────────────────────────────────────────────
    def _open_csv_files(self):
        self._csv_files = {}
        csv_specs = {
            "motion.csv":  ["timestamp", "displacement_m", "velocity_ms", "acceleration_ms2"],
            "lidar.csv":   ["timestamp", "distance_to_obstacle_m"],
            "radar.csv":   ["timestamp", "bearing_deg", "distance_m", "radial_velocity_ms"],
            "fusion.csv":  ["timestamp", "fused_px", "fused_py", "fused_vx", "fused_vy", "fusion_variance"],
            "ttc.csv":     ["timestamp", "ttc_value_s", "risk_level"],
        }
        for fname, cols in csv_specs.items():
            fpath = self.output_dir / fname
            f = open(fpath, "w", newline="")
            w = csv.writer(f)
            w.writerow(cols)
            self._csv_files[fname] = (f, w)

    def _close_csv_files(self):
        for f, _ in self._csv_files.values():
            f.close()
        self.get_logger().info("CSV files closed.")

    # ── Callbacks ────────────────────────────────────────────────────────
    def _odom_callback(self, msg: Odometry):
        self._odom_buffer.append({
            "timestamp": time.time(),
            "pos_x": msg.pose.pose.position.x,
            "pos_y": msg.pose.pose.position.y,
            "pos_z": msg.pose.pose.position.z,
            "vel_x": msg.twist.twist.linear.x,
            "vel_y": msg.twist.twist.linear.y,
            "vel_z": msg.twist.twist.linear.z,
        })

    def _lidar_callback(self, msg: LaserScan):
        # Store minimum forward distance (filter within ±15 deg of forward)
        if len(msg.ranges) == 0:
            return
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        forward_ranges = []
        for i, r in enumerate(msg.ranges):
            angle = angle_min + i * angle_inc
            if abs(angle) < 0.2618:  # ±15° in radians
                if msg.range_min < r < msg.range_max:
                    forward_ranges.append(r)
        min_dist = min(forward_ranges) if forward_ranges else float("nan")

        self._lidar_buffer.append({
            "timestamp": time.time(),
            "distance": min_dist,
        })

    def _radar_callback(self, msg: String):
        """Parse radar data string (format: 'bearing,dist,velocity')."""
        try:
            parts = msg.data.strip().split(",")
            if len(parts) >= 3:
                self._radar_buffer.append({
                    "timestamp": time.time(),
                    "bearing": float(parts[0]),
                    "distance": float(parts[1]),
                    "velocity": float(parts[2]),
                })
        except (ValueError, IndexError):
            self.get_logger().debug(f"Unparseable radar data: {msg.data}")

    def _fusion_callback(self, msg: String):
        """Parse fusion output (JSON-like string)."""
        import json
        try:
            data = json.loads(msg.data)
            obj = data.get("objects", [{}])[0] if data.get("objects") else {}
            self._fusion_buffer.append({
                "timestamp": time.time(),
                "px": obj.get("px", 0.0),
                "py": obj.get("py", 0.0),
                "vx": obj.get("vx", 0.0),
                "vy": obj.get("vy", 0.0),
                "variance": obj.get("variance", 0.0),
            })
        except (json.JSONDecodeError, KeyError):
            self.get_logger().debug(f"Unparseable fusion data: {msg.data}")

    def _ttc_callback(self, msg: String):
        """Parse decision output: 'ttc_value,risk_level'."""
        try:
            parts = msg.data.strip().split(",")
            if len(parts) >= 2:
                risk = parts[1].strip()
                ttc_val = float(parts[0])
                self._ttc_buffer.append({
                    "timestamp": time.time(),
                    "ttc": ttc_val,
                    "risk_level": risk,
                })
                # Capture keyframe on risk change
                if risk != self._last_risk_level:
                    self._capture_keyframe(risk)
                    self._last_risk_level = risk
        except (ValueError, IndexError):
            self.get_logger().debug(f"Unparseable TTC data: {msg.data}")

    def _camera_callback(self, msg):
        if self._bridge is not None:
            self._latest_camera_frame = msg
            self._camera_frame_received = True

    def _capture_keyframe(self, risk_level: str):
        """Save current camera frame as a keyframe JPEG."""
        if not self._camera_frame_received or self._bridge is None:
            return
        try:
            cv_img = self._bridge.imgmsg_to_cv2(self._latest_camera_frame, "bgr8")
            phase_map = {"SAFE": "1", "WARNING": "2", "SLOWDOWN": "3", "EMERGENCY": "4", "RECOVERY": "5"}
            phase = phase_map.get(risk_level, "X")
            fname = f"scene_{self.scene}_phase_{phase}_{risk_level}.jpg"
            fpath = self.keyframe_dir / fname
            cv2.imwrite(str(fpath), cv_img)
            self.get_logger().info(f"Keyframe saved: {fname}")
        except Exception as e:
            self.get_logger().warn(f"Keyframe capture failed: {e}")

    # ── Periodic record tick (10 Hz) ─────────────────────────────────────
    def _record_tick(self):
        now = time.time()

        # ── Motion row (odom) ────────────────────────────────────────────
        if self._odom_buffer:
            odom = self._odom_buffer[-1]
            # displacement from origin (Euclidean)
            displacement = (odom["pos_x"]**2 + odom["pos_y"]**2 + odom["pos_z"]**2)**0.5
            velocity = (odom["vel_x"]**2 + odom["vel_y"]**2 + odom["vel_z"]**2)**0.5
            # acceleration from velocity delta (approximate)
            acceleration = 0.0
            self._csv_files["motion.csv"][1].writerow([
                f"{now - self._start_time:.6f}",
                f"{displacement:.6f}",
                f"{velocity:.3f}",
                f"{acceleration:.3f}",
            ])

        # ── LiDAR row ────────────────────────────────────────────────────
        if self._lidar_buffer:
            lidar = self._lidar_buffer[-1]
            self._csv_files["lidar.csv"][1].writerow([
                f"{now - self._start_time:.6f}",
                f"{lidar['distance']:.4f}",
            ])

        # ── Radar row ────────────────────────────────────────────────────
        if self._radar_buffer:
            radar = self._radar_buffer[-1]
            self._csv_files["radar.csv"][1].writerow([
                f"{now - self._start_time:.6f}",
                f"{radar['bearing']:.2f}",
                f"{radar['distance']:.4f}",
                f"{radar['velocity']:.3f}",
            ])

        # ── Fusion row ───────────────────────────────────────────────────
        if self._fusion_buffer:
            fus = self._fusion_buffer[-1]
            self._csv_files["fusion.csv"][1].writerow([
                f"{now - self._start_time:.6f}",
                f"{fus['px']:.4f}",
                f"{fus['py']:.4f}",
                f"{fus['vx']:.3f}",
                f"{fus['vy']:.3f}",
                f"{fus['variance']:.6f}",
            ])

        # ── TTC row ──────────────────────────────────────────────────────
        if self._ttc_buffer:
            ttc = self._ttc_buffer[-1]
            self._csv_files["ttc.csv"][1].writerow([
                f"{now - self._start_time:.6f}",
                f"{ttc['ttc']:.4f}",
                ttc["risk_level"],
            ])

    def destroy_node(self):
        self._close_csv_files()
        super().destroy_node()


# ── CLI entry ────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="ADAS Fusion experiment data collector")
    parser.add_argument("--scene", type=int, required=True, choices=[1, 2, 3, 4, 5])
    parser.add_argument("--run", type=int, required=True, help="Run ID (1, 2, 3...)")
    parser.add_argument("--duration", type=int, default=120, help="Collection duration in seconds")
    parser.add_argument("--odom_topic", type=str, default="/odom", help="Odometry topic name")
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = DataCollector(scene=args.scene, run_id=args.run, odom_topic=args.odom_topic)

    # Auto-shutdown after duration
    def shutdown():
        node.get_logger().info(f"Duration ({args.duration}s) reached — shutting down.")
        node.destroy_node()
        rclpy.shutdown()

    timer = node.create_timer(args.duration, shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
