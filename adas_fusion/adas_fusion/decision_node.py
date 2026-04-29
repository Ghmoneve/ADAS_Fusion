#!/usr/bin/env python3
"""
decision_node.py -- 基于 TTC 的分级避障决策节点
================================================

=============================================================================
算法概述 (中文伪代码)
=============================================================================

本节点实现基于 TTC (Time-To-Collision) 的分级避障决策：

┌─────────────────────────────────────────────────────────────────────────┐
│ 1. TTC 计算                                                              │
│                                                                          │
│    对每个跟踪目标:                                                        │
│                                                                          │
│    d = sqrt((px_target - px_robot)^2 + (py_target - py_robot)^2)         │
│                                                                          │
│    v_rel = (v_target - v_robot) · (p_target - p_robot) / d               │
│           = 相对速度在径向上的投影                                        │
│                                                                          │
│    IF v_rel > 0:  (目标正在相对接近)                                      │
│        TTC = d / v_rel                                                   │
│    ELSE:                                                                 │
│        TTC = INF  (目标远离或静止, 无碰撞风险)                             │
│    END IF                                                                │
│                                                                          │
│    物理意义: TTC 表示在当前位置和速度下, 还有多少秒发生碰撞。               │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│ 2. 风险评估                                                              │
│                                                                          │
│    min_TTC = min(TTC_i),  i = 1..N                                      │
│                                                                          │
│    风险分级:                                                              │
│                                                                          │
│      IF   min_TTC > T_warning (5s):                                      │
│          risk = SAFE        "正常行驶"                                   │
│      ELIF min_TTC > T_slowdown (3s):                                     │
│          risk = WARNING      "预警, 轻度减速"                             │
│      ELIF min_TTC > T_emergency (1s):                                    │
│          risk = SLOWDOWN     "大幅减速"                                   │
│      ELSE:                                                                │
│          risk = EMERGENCY    "紧急停车"                                   │
│      END IF                                                              │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│ 3. 分级避障策略 (决策输出)                                                │
│                                                                          │
│    参数:                                                                 │
│        v_desired:  期望线速度 (用户设定)                                  │
│        v_max:      最大线速度 0.3 m/s                                    │
│        ω_max:     最大角速度 0.5 rad/s                                   │
│        slowdown:   减速因子 0.5                                          │
│                                                                          │
│    ┌──────────────────┬──────────────────────────────────────────┐       │
│    │ TTC 范围          │ 行为                                      │       │
│    ├──────────────────┼──────────────────────────────────────────┤       │
│    │ TTC > 5s          │ v = v_desired,  ω = ω_desired (正常)     │       │
│    │ 3s < TTC ≤ 5s    │ v = v_desired * 0.7,  保持原方向          │       │
│    │ 1s < TTC ≤ 3s    │ v = v_desired * 0.5,  尝试绕行            │       │
│    │ TTC ≤ 1s          │ v = 0.0,             ω = 0.0 (急停)      │       │
│    │ 特殊情况          │ 评估绕行方向后 ω = ω_safe                 │       │
│    └──────────────────┴──────────────────────────────────────────┘       │
│                                                                          │
│    绕行逻辑 (SLOWDOWN 阶段):                                              │
│      1. 将前方 180° 按 15° 分辨率分成 12 个扇区                           │
│      2. 对每个扇区, 检查是否有障碍物在该方向                               │
│      3. 选择无障碍且最接近目标方向的扇区                                   │
│      4. 若预计绕行路径足够安全 (TTC > 3s), 执行绕行角速度                  │
│      5. 若无法绕行, 保持直行减速                                          │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│ 4. 安全边界                                                              │
│                                                                          │
│    - 最小安全距离: 0.3m (低于此距离任何时候都急停)                         │
│    - 最大角速度限幅: ±ω_max                                              │
│    - 平滑过渡: 速度变化率 ≤ 0.1 m/s² (可选)                               │
└─────────────────────────────────────────────────────────────────────────┘

=============================================================================
实现开始
=============================================================================
"""

import math
import numpy as np
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from adas_fusion_msgs.msg import TrackedObjectArray


class DecisionNode(Node):
    """基于 TTC 的分级避障决策节点。"""

    def __init__(self):
        super().__init__('decision_node')

        # ---- 参数 ----
        self.declare_parameter('ttc_warning', 5.0)
        self.declare_parameter('ttc_slowdown', 3.0)
        self.declare_parameter('ttc_emergency', 1.0)
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('slowdown_factor', 0.5)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('desired_linear_vel', 0.2)
        self.declare_parameter('desired_angular_vel', 0.0)
        self.declare_parameter('min_safe_distance', 0.3)
        self.declare_parameter('tracked_objects_topic', '/tracked_objects')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # ---- 订阅 ----
        tracked_topic = self.get_parameter('tracked_objects_topic').value
        self._sub = self.create_subscription(
            TrackedObjectArray, tracked_topic, self._tracked_cb, 10)

        # ---- 发布 ----
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # ---- 当前跟踪目标缓存 ----
        self._tracks: Optional[TrackedObjectArray] = None

        # ---- 定时器: 周期性决策 ----
        self._timer = self.create_timer(0.1, self._decision_cycle)

        self.get_logger().info(
            'DecisionNode initialized. '
            f'TTC thresholds — WARN:{self.get_parameter("ttc_warning").value}s '
            f'SLOW:{self.get_parameter("ttc_slowdown").value}s '
            f'EMERG:{self.get_parameter("ttc_emergency").value}s')

    def _tracked_cb(self, msg: TrackedObjectArray):
        self._tracks = msg

    # ======================================================================
    # 决策主循环
    # ======================================================================

    def _decision_cycle(self):
        """周期性评估风险并发布 cmd_vel。"""
        v_desired = self.get_parameter('desired_linear_vel').value
        w_desired = self.get_parameter('desired_angular_vel').value
        v_max = self.get_parameter('max_linear_vel').value
        w_max = self.get_parameter('max_angular_vel').value
        slowdown = self.get_parameter('slowdown_factor').value
        min_dist = self.get_parameter('min_safe_distance').value

        # 无目标 → 正常行驶
        if self._tracks is None or not self._tracks.objects:
            self._publish_cmd(v_desired, w_desired)
            return

        # ---- 计算机器人参考位置 (假设在原点, base_link) ----
        robot_pos = np.array([0.0, 0.0])
        robot_vel = np.array([v_desired, 0.0])  # 假设前进方向

        # ---- 计算每个目标的 TTC ----
        ttc_list: List[Tuple[float, np.ndarray, float]] = []
        # (TTC, 目标位置, 目标距离)

        for obj in self._tracks.objects:
            # 相对位置
            target_pos = np.array([obj.position.x, obj.position.y])
            rel_pos = target_pos - robot_pos
            dist = float(np.linalg.norm(rel_pos))

            if dist < 1e-6:
                ttc_list.append((0.0, target_pos, dist))
                continue

            # 相对速度 (径向分量)
            target_vel = np.array([obj.vx, obj.vy])
            rel_vel = target_vel - robot_vel
            # 径向相对速度 = dot(相对位置方向, 相对速度)
            radial_vel = float(np.dot(rel_pos / dist, rel_vel))
            # 正值表示正在接近
            if radial_vel > 0.01:
                ttc = dist / radial_vel
            else:
                ttc = float('inf')  # 远离, 无风险

            ttc_list.append((ttc, target_pos, dist))

        if not ttc_list:
            self._publish_cmd(v_desired, w_desired)
            return

        # ---- 风险判定 (取最危险目标) ----
        ttc_list.sort(key=lambda x: x[0])
        min_ttc, closest_pos, closest_dist = ttc_list[0]

        t_warn = self.get_parameter('ttc_warning').value
        t_slow = self.get_parameter('ttc_slowdown').value
        t_emerg = self.get_parameter('ttc_emergency').value

        v = v_desired
        w = w_desired

        # ---- 绝对安全距离检查 ----
        if closest_dist < min_dist:
            self.get_logger().warn(
                f'EMERGENCY: Object within {min_dist}m! Stopping.')
            self._publish_cmd(0.0, 0.0)
            return

        # ---- 分级决策 ----
        if min_ttc <= t_emerg:
            # 紧急停车
            v = 0.0
            w = 0.0
            self.get_logger().warn(
                f'EMERGENCY STOP: min_TTC={min_ttc:.2f}s, dist={closest_dist:.2f}m')

        elif min_ttc <= t_slow:
            # 减速 + 尝试绕行
            v = v_desired * slowdown
            # 寻找安全绕行方向
            safe_w = self._find_safe_direction(closest_pos, ttc_list)
            w = safe_w if safe_w is not None else 0.0
            self.get_logger().warn(
                f'SLOWDOWN: min_TTC={min_ttc:.2f}s, '
                f'v={v:.3f}, w={w:.3f}')

        elif min_ttc <= t_warn:
            # 轻度减速
            v = v_desired * 0.7
            self.get_logger().info(
                f'WARNING: min_TTC={min_ttc:.2f}s, v={v:.3f}')

        else:
            # 正常行驶
            self.get_logger().debug(f'SAFE: min_TTC={min_ttc:.2f}s')

        # ---- 速度限幅 ----
        v = np.clip(v, 0.0, v_max)
        w = np.clip(w, -w_max, w_max)

        self._publish_cmd(v, w)

    # ======================================================================
    # 绕行方向搜索
    # ======================================================================

    def _find_safe_direction(self, closest_pos: np.ndarray,
                             ttc_list: List[Tuple[float, np.ndarray, float]]
                             ) -> Optional[float]:
        """
        在 SLOWDOWN 阶段搜索安全绕行方向。

        算法:
          1. 将前方 180° 分成 12 个扇区 (每 15°)
          2. 对每个扇区, 检查在该方向是否有障碍物
          3. 排除有障碍物的扇区
          4. 选择剩余扇区中偏离最近的障碍物最远的方向

        Returns:
           安全角速度 (rad/s), 若无法绕行返回 None
        """
        w_max = self.get_parameter('max_angular_vel').value
        n_sectors = 12
        d_angle = math.pi / n_sectors  # 每扇区 15°

        # 障碍物方向集合 (哪些角度扇区被占据)
        occupied = set()
        for ttc, pos, dist in ttc_list:
            if dist < 5.0:  # 只关注 5m 内的障碍物
                angle = math.atan2(pos[1], pos[0])
                sector = int((angle + math.pi) / d_angle) % n_sectors
                occupied.add(sector)

        # 寻找安全扇区
        safe_sectors = [i for i in range(n_sectors) if i not in occupied]
        if not safe_sectors:
            return None  # 所有方向被占据

        # 选择最接近正前方 (0°) 的安全扇区
        best_sector = min(safe_sectors,
                          key=lambda s: abs(math.pi - s * d_angle))
        # 该扇区中心角度
        best_angle = best_sector * d_angle - math.pi
        # 转换为角速度
        w = np.clip(best_angle * 0.3, -w_max, w_max)

        return w

    # ======================================================================
    # 发布
    # ======================================================================

    def _publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self._cmd_pub.publish(msg)


# ==============================================================================
# 入口
# ==============================================================================

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
