#!/usr/bin/env python3
"""
decision_node.py -- 基于 TTC 的分级避障决策节点 (手柄控制 + 紧急接管)
=====================================================================

功能:
  1. 正常模式: 手柄 (joystick) 控制小车全向移动
  2. 紧急避障: 当 TTC 或距离触发阈值时, 接管控制权, 执行避障逻辑
  3. 恢复: 紧急状态解除后等待 3 秒无触发, 归还手柄控制权

输入:
  - /joy (sensor_msgs/Joy)                         # 手柄输入
  - /tracked_objects (TrackedObjectArray)           # 融合跟踪目标

输出:
  - /cmd_vel (geometry_msgs/Twist)                  # 速度指令

控制逻辑:
  ┌─────────────────────────────────────────────────────────┐
  │  IF 紧急条件触发 (TTC ≤ ttc_emergency OR dist < 0.3m):   │
  │      mode = EMERGENCY                                    │
  │      手柄输入被忽略, 输出避障 cmd_vel                       │
  │      记录触发时间 t_trigger                               │
  │  ELIF mode == EMERGENCY:                                 │
  │      IF now - t_trigger > cooldown (3s) AND 无紧急条件:   │
  │          mode = JOYSTICK  (归还控制权)                    │
  │      ELSE:                                               │
  │          继续输出安全指令 (缓慢减速或停车)                  │
  │  ELSE:                                                   │
  │      mode = JOYSTICK  (手柄直通)                         │
  │      输出手柄 cmd_vel                                     │
  └─────────────────────────────────────────────────────────┘

手柄映射 (与 turtlebot4 / 标准 ROS teleop 兼容):
  轴 1 (左摇杆上下) → linear.x  (前进/后退)
  轴 3 (右摇杆左右) → angular.z (左转/右转)
  或 轴 0 (左摇杆左右) → angular.z (备选)

=============================================================================
算法概述 (中文伪代码)
=============================================================================

┌─────────────────────────────────────────────────────────────────────────┐
│ TTC 计算 (Algorithm.md)                                                  │
│                                                                          │
│    d = ||target_pos - robot_pos||                                       │
│    v_rel = (v_target - v_robot) · (p_target - p_robot) / d              │
│    IF v_rel > 0:  TTC = d / v_rel                                       │
│    ELSE:          TTC = INF                                             │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│ 分级决策                                                                 │
│                                                                          │
│  TTC ≤ 1.0s  or  dist < 0.3m:  EMERGENCY → v=0, 接管控制权              │
│  TTC ≤ 3.0s:                   SLOWDOWN → v=v_desired*0.5, 尝试绕行     │
│  TTC ≤ 5.0s:                   WARNING  → v=v_desired*0.7               │
│  TTC > 5.0s:                   SAFE     → 手柄直通                      │
└─────────────────────────────────────────────────────────────────────────┘

=============================================================================
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

from adas_fusion_msgs.msg import TrackedObjectArray


class ControlMode(Enum):
    JOYSTICK = 0       # 手柄控制
    AUTONOMOUS = 1     # 自动避障接管


class DecisionNode(Node):
    """手柄控制 + TTC 紧急避障接管节点。"""

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
        # 手柄轴映射
        self.declare_parameter('joy_axis_linear', 1)     # 左摇杆上下
        self.declare_parameter('joy_axis_angular', 3)    # 右摇杆左右
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

        # ---- 状态 ----
        self._tracks: Optional[TrackedObjectArray] = None
        self._joy_cmd = Twist()               # 最新手柄指令
        self._joy_active = False              # 手柄是否在线
        self._last_joy_time = 0.0

        self._mode = ControlMode.JOYSTICK
        self._emergency_trigger_time = 0.0    # 紧急触发时刻
        self._cooldown = self.get_parameter('cooldown_seconds').value

        # ---- 定时器 ----
        self._timer = self.create_timer(0.05, self._decision_cycle)   # 20Hz

        self.get_logger().info(
            'DecisionNode v2.0 initialized. '
            f'Joystick enabled, TTC thresholds: '
            f'WARN={self.get_parameter("ttc_warning").value}s '
            f'SLOW={self.get_parameter("ttc_slowdown").value}s '
            f'EMERG={self.get_parameter("ttc_emergency").value}s '
            f'cooldown={self._cooldown}s')

    # ======================================================================
    # 回调
    # ======================================================================

    def _joy_cb(self, msg: Joy):
        """处理手柄输入，转换为 cmd_vel。"""
        ax_lin = self.get_parameter('joy_axis_linear').value
        ax_ang = self.get_parameter('joy_axis_angular').value
        deadzone = self.get_parameter('joy_deadzone').value
        v_max = self.get_parameter('max_linear_vel').value
        w_max = self.get_parameter('max_angular_vel').value

        # 读取轴值
        raw_linear = msg.axes[ax_lin] if ax_lin < len(msg.axes) else 0.0
        raw_angular = msg.axes[ax_ang] if ax_ang < len(msg.axes) else 0.0

        # 死区过滤
        if abs(raw_linear) < deadzone:
            raw_linear = 0.0
        if abs(raw_angular) < deadzone:
            raw_angular = 0.0

        self._joy_cmd.linear.x = raw_linear * v_max
        self._joy_cmd.angular.z = raw_angular * w_max
        self._joy_active = True
        self._last_joy_time = time.time()

    def _tracked_cb(self, msg: TrackedObjectArray):
        self._tracks = msg

    # ======================================================================
    # 决策主循环 (20Hz)
    # ======================================================================

    def _decision_cycle(self):
        """周期性评估风险并输出 cmd_vel。"""
        now = time.time()

        # 手柄超时检测 (1秒无数据视为离线)
        enable_joy = self.get_parameter('enable_joystick').value
        if enable_joy and now - self._last_joy_time > 1.0:
            if self._joy_active:
                self.get_logger().warn('Joystick timeout, stopping.')
                self._joy_active = False
                self._joy_cmd = Twist()

        # 风险等级判定
        risk_level, min_ttc, closest_dist = self._assess_risk()

        v_max = self.get_parameter('max_linear_vel').value
        w_max = self.get_parameter('max_angular_vel').value
        slowdown = self.get_parameter('slowdown_factor').value
        t_emerg = self.get_parameter('ttc_emergency').value

        # ---- 紧急条件判定 ----
        is_emergency = (risk_level == 'EMERGENCY' or
                        closest_dist < self.get_parameter('min_safe_distance').value)

        if is_emergency:
            # 紧急接管
            if self._mode != ControlMode.AUTONOMOUS:
                self.get_logger().warn(
                    f'EMERGENCY OVERRIDE: min_TTC={min_ttc:.2f}s, dist={closest_dist:.2f}m')
            self._mode = ControlMode.AUTONOMOUS
            self._emergency_trigger_time = now

            # 紧急避障: 停车
            self._publish_cmd(0.0, 0.0)

        elif self._mode == ControlMode.AUTONOMOUS:
            # 检查是否可以恢复手柄
            if now - self._emergency_trigger_time >= self._cooldown:
                self.get_logger().info('Cooldown expired, returning to joystick control.')
                self._mode = ControlMode.JOYSTICK
                self._publish_joy()
            else:
                # 冷却中, 保持安全指令
                safe_v = self._joy_cmd.linear.x * slowdown if self._joy_active else 0.0
                safe_v = min(safe_v, 0.1)  # 冷却期限制低速
                self._publish_cmd(safe_v, 0.0)

        else:
            # 手柄模式: 正常直通
            joy_v = self._joy_cmd.linear.x if self._joy_active else 0.0
            joy_w = self._joy_cmd.angular.z if self._joy_active else 0.0

            if risk_level == 'SAFE':
                # 完全手柄控制
                self._publish_joy()

            elif risk_level == 'WARNING':
                # 轻度限速, 手柄仍然生效
                v = np.clip(joy_v * 0.7, -v_max * 0.7, v_max * 0.7)
                w = np.clip(joy_w, -w_max, w_max)
                self._publish_cmd(v, w)
                self.get_logger().info(
                    f'WARNING: min_TTC={min_ttc:.2f}s, limiting speed')

            elif risk_level == 'SLOWDOWN':
                # 大幅限速, 尝试绕行
                v = np.clip(joy_v * slowdown, -v_max * slowdown, v_max * slowdown)
                # 综合手柄方向和避障方向
                tracks_data = self._extract_tracks_data()
                closest_pos = tracks_data[0][1] if tracks_data else np.array([1.0, 0.0])
                safe_w = self._find_safe_direction(closest_pos, tracks_data)
                w = safe_w if safe_w is not None else np.clip(joy_w, -w_max * 0.3, w_max * 0.3)
                w = np.clip(w, -w_max, w_max)
                self._publish_cmd(v, w)
                self.get_logger().warn(
                    f'SLOWDOWN: min_TTC={min_ttc:.2f}s, v={v:.3f}, w={w:.3f}')

    # ======================================================================
    # 风险评估
    # ======================================================================

    def _assess_risk(self) -> Tuple[str, float, float]:
        """
        评估当前风险等级。

        Returns:
            (risk_level, min_ttc, min_dist)
        """
        if self._tracks is None or not self._tracks.objects:
            return ('SAFE', float('inf'), float('inf'))

        robot_pos = np.array([0.0, 0.0])
        # 使用手柄期望速度或实际速度
        robot_vel = np.array([
            self._joy_cmd.linear.x if self._joy_active else 0.0,
            0.0,
        ])

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

    # ======================================================================
    # 辅助
    # ======================================================================

    def _extract_tracks_data(self) -> List[Tuple[float, np.ndarray, float]]:
        """提取跟踪目标数据: [(TTC, pos, dist), ...]."""
        if self._tracks is None:
            return []
        robot_pos = np.array([0.0, 0.0])
        robot_vel = np.array([
            self._joy_cmd.linear.x if self._joy_active else 0.0, 0.0])
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
        """绕行方向搜索 (扇区法)。"""
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

    # ======================================================================
    # 发布
    # ======================================================================

    def _publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self._cmd_pub.publish(msg)

    def _publish_joy(self):
        """直通手柄指令。"""
        if self._joy_active:
            self._cmd_pub.publish(self._joy_cmd)
        else:
            self._publish_cmd(0.0, 0.0)


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
