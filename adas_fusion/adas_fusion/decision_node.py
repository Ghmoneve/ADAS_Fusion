#!/usr/bin/env python3
"""
decision_node.py -- TTC 分级避障决策 (turtlebot4 共存版)
=========================================================

与 turtlebot4 原生系统共存:
  - 订阅 /joy (原生 joy_linux_node), 读取手柄意图
  - 订阅 /tracked_objects (融合结果), 评估 TTC 风险
  - SAFE:      不干预 (turtlebot4 原生 teleop 正常工作)
  - WARNING:   发布限速 /cmd_vel (可能被 Create3 接受)
  - SLOWDOWN:  大幅限速
  - EMERGENCY: 发布零速 /cmd_vel
  - 同时发布 /decision/ttc 供 data_collector 记录
"""

import math
import time
import numpy as np
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from adas_fusion_msgs.msg import TrackedObjectArray


class ControlMode(Enum):
    JOYSTICK = 0
    AUTONOMOUS = 1


class DecisionNode(Node):
    """turtlebot4 共存: 监听 /joy + /tracked_objects, 紧急时输出 /cmd_vel。"""

    def __init__(self):
        super().__init__('decision_node')

        # ---- 参数 ----
        self.declare_parameter('ttc_warning', 5.0)
        self.declare_parameter('ttc_slowdown', 3.0)
        self.declare_parameter('ttc_emergency', 1.0)
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('slowdown_factor', 0.5)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('min_safe_distance', 0.3)
        self.declare_parameter('cooldown_seconds', 3.0)
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('tracked_objects_topic', '/tracked_objects')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('ttc_topic', '/decision/ttc')
        self.declare_parameter('joy_axis_linear', 1)
        self.declare_parameter('joy_axis_angular', 0)
        self.declare_parameter('joy_deadzone', 0.1)
        self.declare_parameter('enable_joystick', True)

        # ---- 订阅 ----
        joy_topic = self.get_parameter('joy_topic').value
        self._joy_sub = self.create_subscription(
            Joy, joy_topic, self._joy_cb, 10)

        tracked_topic = self.get_parameter('tracked_objects_topic').value
        self._tracked_sub = self.create_subscription(
            TrackedObjectArray, tracked_topic, self._tracked_cb, 10)

        # ---- 发布 ----
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        ttc_topic = self.get_parameter('ttc_topic').value
        self._ttc_pub = self.create_publisher(String, ttc_topic, 10)

        # ---- 状态 ----
        self._tracks: Optional[TrackedObjectArray] = None
        self._latest_joy: Optional[Joy] = None
        self._joy_active = False
        self._last_joy_time = 0.0

        self._mode = ControlMode.JOYSTICK
        self._emergency_trigger_time = 0.0
        self._cooldown = self.get_parameter('cooldown_seconds').value
        self._actual_v = 0.0

        # ---- 定时器 (20Hz) ----
        self._timer = self.create_timer(0.05, self._decision_cycle)

        self.get_logger().info(
            'DecisionNode v4.0 (turtlebot4 coexist). '
            f'WARN={self.get_parameter("ttc_warning").value}s '
            f'SLOW={self.get_parameter("ttc_slowdown").value}s '
            f'EMERG={self.get_parameter("ttc_emergency").value}s '
            f'cooldown={self._cooldown}s')

    # ======================================================================
    # 回调
    # ======================================================================

    def _joy_cb(self, msg: Joy):
        self._latest_joy = msg
        self._joy_active = True
        self._last_joy_time = time.time()

    def _tracked_cb(self, msg: TrackedObjectArray):
        self._tracks = msg

    # ======================================================================
    # 决策主循环 (20Hz)
    # ======================================================================

    def _decision_cycle(self):
        now = time.time()

        # 手柄超时检测
        enable_joy = self.get_parameter('enable_joystick').value
        if enable_joy and now - self._last_joy_time > 1.0:
            if self._joy_active:
                self.get_logger().warn('Joystick timeout.')
                self._joy_active = False
                self._latest_joy = None

        # 风险评估
        risk_level, min_ttc, closest_dist = self._assess_risk()

        # 发布 TTC 日志
        ttc_msg = String()
        ttc_msg.data = f'{min_ttc:.4f},{risk_level}'
        self._ttc_pub.publish(ttc_msg)

        v_max = self.get_parameter('max_linear_vel').value
        w_max = self.get_parameter('max_angular_vel').value
        slowdown = self.get_parameter('slowdown_factor').value

        is_emergency = (risk_level == 'EMERGENCY' or
                        closest_dist < self.get_parameter('min_safe_distance').value)

        if is_emergency:
            if self._mode != ControlMode.AUTONOMOUS:
                self.get_logger().warn(
                    f'EMERGENCY: TTC={min_ttc:.2f}s dist={closest_dist:.2f}m — STOP')
            self._mode = ControlMode.AUTONOMOUS
            self._emergency_trigger_time = now
            self._publish_cmd(0.0, 0.0)

        elif self._mode == ControlMode.AUTONOMOUS:
            if now - self._emergency_trigger_time >= self._cooldown:
                self.get_logger().info('Cooldown expired, returning control.')
                self._mode = ControlMode.JOYSTICK
            else:
                self._publish_cmd(0.0, 0.0)

        elif risk_level == 'SLOWDOWN':
            joy_v, joy_w = self._get_joy_cmd()
            v = np.clip(joy_v * slowdown, -v_max * slowdown, v_max * slowdown)
            tracks_data = self._extract_tracks_data()
            closest_pos = tracks_data[0][1] if tracks_data else np.array([1.0, 0.0])
            safe_w = self._find_safe_direction(closest_pos, tracks_data)
            w = safe_w if safe_w is not None else np.clip(joy_w, -w_max * 0.3, w_max * 0.3)
            self._publish_cmd(v, w)
            self.get_logger().warn(f'SLOWDOWN: TTC={min_ttc:.2f}s v={v:.3f}')

        elif risk_level == 'WARNING':
            joy_v, joy_w = self._get_joy_cmd()
            v = np.clip(joy_v * 0.7, -v_max * 0.7, v_max * 0.7)
            w = np.clip(joy_w, -w_max, w_max)
            self._publish_cmd(v, w)
            self.get_logger().info(f'WARNING: TTC={min_ttc:.2f}s limit 70%')

        # SAFE: 不发布 /cmd_vel (让 turtlebot4 原生 teleop 工作)

    # ======================================================================
    # 手柄 → cmd_vel 映射
    # ======================================================================

    def _get_joy_cmd(self) -> Tuple[float, float]:
        if not self._joy_active or self._latest_joy is None:
            return (0.0, 0.0)
        ax_lin = self.get_parameter('joy_axis_linear').value
        ax_ang = self.get_parameter('joy_axis_angular').value
        deadzone = self.get_parameter('joy_deadzone').value
        v_max = self.get_parameter('max_linear_vel').value
        w_max = self.get_parameter('max_angular_vel').value

        raw_lin = self._latest_joy.axes[ax_lin] if ax_lin < len(self._latest_joy.axes) else 0.0
        raw_ang = self._latest_joy.axes[ax_ang] if ax_ang < len(self._latest_joy.axes) else 0.0
        if abs(raw_lin) < deadzone:
            raw_lin = 0.0
        if abs(raw_ang) < deadzone:
            raw_ang = 0.0

        return (raw_lin * v_max, raw_ang * w_max)

    # ======================================================================
    # 发布
    # ======================================================================

    def _publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self._cmd_pub.publish(msg)
        self._actual_v = v

    # ======================================================================
    # 风险评估
    # ======================================================================

    def _assess_risk(self) -> Tuple[str, float, float]:
        if self._tracks is None or not self._tracks.objects:
            return ('SAFE', float('inf'), float('inf'))

        robot_pos = np.array([0.0, 0.0])
        robot_vel = np.array([self._actual_v, 0.0])

        t_warn = self.get_parameter('ttc_warning').value
        t_slow = self.get_parameter('ttc_slowdown').value
        t_emerg = self.get_parameter('ttc_emergency').value

        min_ttc = float('inf')
        min_dist = float('inf')

        for obj in self._tracks.objects:
            target_pos = np.array([obj.position.x, obj.position.y])
            rel_pos = target_pos - robot_pos
            dist = float(np.linalg.norm(rel_pos))
            min_dist = min(min_dist, dist)

            if dist < 1e-6:
                min_ttc = 0.0
                continue

            target_vel = np.array([obj.vx, obj.vy])
            rel_vel = target_vel - robot_vel
            radial_vel = float(np.dot(rel_pos / dist, rel_vel))

            if radial_vel > 0.01:
                ttc = dist / radial_vel
                min_ttc = min(min_ttc, ttc)

        if min_ttc <= t_emerg or min_dist < self.get_parameter('min_safe_distance').value:
            return ('EMERGENCY', min_ttc, min_dist)
        elif min_ttc <= t_slow:
            return ('SLOWDOWN', min_ttc, min_dist)
        elif min_ttc <= t_warn:
            return ('WARNING', min_ttc, min_dist)
        return ('SAFE', min_ttc, min_dist)

    def _extract_tracks_data(self) -> List[Tuple[float, np.ndarray, float]]:
        if self._tracks is None:
            return []
        robot_pos = np.array([0.0, 0.0])
        robot_vel = np.array([self._actual_v, 0.0])
        result = []
        for obj in self._tracks.objects:
            target_pos = np.array([obj.position.x, obj.position.y])
            dist = float(np.linalg.norm(target_pos - robot_pos))
            if dist < 1e-6:
                result.append((0.0, target_pos, dist))
                continue
            rel_vel = np.array([obj.vx, obj.vy]) - robot_vel
            radial = float(np.dot((target_pos - robot_pos) / dist, rel_vel))
            ttc = dist / radial if radial > 0.01 else float('inf')
            result.append((ttc, target_pos, dist))
        result.sort(key=lambda x: x[0])
        return result

    def _find_safe_direction(self, closest_pos: np.ndarray,
                             ttc_list: List[Tuple[float, np.ndarray, float]]
                             ) -> Optional[float]:
        w_max = self.get_parameter('max_angular_vel').value
        n_sectors = 12
        d_angle = math.pi / n_sectors

        occupied = set()
        for ttc, pos, dist in ttc_list:
            if dist < 5.0:
                angle = math.atan2(pos[1], pos[0])
                sector = int((angle + math.pi) / d_angle) % n_sectors
                occupied.add(sector)

        safe_sectors = [i for i in range(n_sectors) if i not in occupied]
        if not safe_sectors:
            return None

        best_sector = min(safe_sectors,
                          key=lambda s: abs(math.pi - s * d_angle))
        best_angle = best_sector * d_angle - math.pi
        return float(np.clip(best_angle * 0.3, -w_max, w_max))


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
