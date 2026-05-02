#!/usr/bin/env python3
"""
fusion_node.py -- 多传感器最优融合跟踪节点
==============================================

=============================================================================
算法概述 (基于 Algorithm.md v2.0 — 最优融合理论)
=============================================================================

本节点实现以下核心算法：

1. 时间同步 — 滑动时间窗 (100ms)
2. 空间对齐 — TF 坐标变换
3. LiDAR 点云 — 欧氏距离聚类 (§5.3)
4. 数据关联 — 马氏距离检验 + 最近邻 (§5.4)
5. 多传感器最优融合 — 精度加权 BLUE (§5.5, Theorem 5.1)
6. Kalman Filter — DWNA 模型 (§5.6, Theorem 5.2)
7. 目标生命周期管理

关键改进 (v2.0):
  - Q 矩阵: 采用 dt⁴/4, dt³/2 的离散白噪声加速度 (DWNA) 公式
  - 传感器噪声模型: 方差建模 (精度 = 1/方差)
    · 视觉: σ²_cam = (σ²_c0 / conf) * exp(d² / (2*σ_c²))
    · LiDAR: σ²_lidar = σ²_l0 * (N_ref / |C|)
    · 雷达: σ²_radar = σ²_r0 / (conf * (1 + α|v|/v₀))
  - 融合权重: w_i = (1/σ²_i) / Σ(1/σ²_j) — 最优线性无偏估计
  - R_fused: 由融合精度实时计算, 输入 Kalman 更新
  - 移除先验-后验平滑 (已被 Kalman 更新覆盖)
=============================================================================
"""

import math
import time
import numpy as np
from collections import deque
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from adas_fusion_msgs.msg import Detection2DArray, RadarObjectArray
from adas_fusion_msgs.msg import TrackedObject, TrackedObjectArray

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


# ==============================================================================
# 数据结构
# ==============================================================================

class Observation:
    """单个传感器观测 (空间对齐后, base_link 坐标系下)。"""
    __slots__ = ('position', 'velocity', 'confidence', 'source',
                 'cluster_size', 'stamp', 'dist')

    def __init__(self, x=0.0, y=0.0, vx=0.0, vy=0.0,
                 confidence=0.0, source=0, cluster_size=1):
        self.position = np.array([x, y], dtype=np.float64)
        self.velocity = np.array([vx, vy], dtype=np.float64)
        self.confidence = confidence
        self.source = source          # 1=camera, 2=lidar, 4=radar
        self.cluster_size = cluster_size
        self.stamp = 0.0
        self.dist = 0.0               # 径向距离 (TF变换后计算)


class Track:
    """单个跟踪目标，内部维护 Kalman Filter (DWNA 模型)。"""
    __slots__ = ('id', 'x', 'P', 'F', 'H', 'Q', 'R_fused', 'class_id',
                 'confidence', 'source_flag', 'miss_count', 'hit_count',
                 'state', 'last_update')

    _next_id = 0

    def __init__(self, state_init: np.ndarray, dt: float, q: float,
                 class_id: int = -1, confidence: float = 0.0,
                 source_flag: int = 0):
        Track._next_id += 1
        self.id = Track._next_id
        self.x = state_init.copy()            # [px, py, vx, vy]
        self.P = np.eye(4) * 10.0

        # 状态转移矩阵 (CV 模型)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float64)

        # 观测矩阵
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float64)

        # 过程噪声 Q (DWNA: Discrete White Noise Acceleration)
        # Algorithm.md §5.6.1:
        #   Q = q * [dt^4/4, 0, dt^3/2, 0; 0, dt^4/4, 0, dt^3/2;
        #            dt^3/2, 0, dt^2,   0; 0, dt^3/2, 0, dt^2]
        dt2 = dt * dt
        dt3 = dt2 * dt / 2.0        # dt³/2
        dt4 = dt2 * dt2 / 4.0       # dt⁴/4
        self.Q = q * np.array([
            [dt4, 0.0, dt3, 0.0],
            [0.0, dt4, 0.0, dt3],
            [dt3, 0.0, dt2, 0.0],
            [0.0, dt3, 0.0, dt2],
        ], dtype=np.float64)

        # 融合后的观测噪声 (每帧由融合算法更新)
        self.R_fused = np.array([[0.1, 0.0], [0.0, 0.1]], dtype=np.float64)

        self.class_id = class_id
        self.confidence = confidence
        self.source_flag = source_flag
        self.miss_count = 0
        self.hit_count = 1
        self.state = 'CANDIDATE'
        self.last_update = time.time()

    def predict(self):
        """Kalman 预测步骤。"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z: np.ndarray, R_obs: np.ndarray):
        """
        Kalman 更新步骤 (使用融合后的观测协方差 R_obs)。

        Algorithm.md §5.5.4 / §5.6.2:
          y = z - H*x_pred
          S = H*P_pred*H^T + R_obs
          K = P_pred*H^T*S^{-1}
          x_new = x_pred + K*y
          P_new = (I - K*H)*P_pred
        """
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R_obs
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = self.P @ self.H.T @ S_inv
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.R_fused = R_obs
        self.last_update = time.time()

    def mahalanobis_distance_sq(self, z: np.ndarray,
                                sigma2: float) -> float:
        """
        计算马氏距离平方 D_M² (Algorithm.md §5.4.4)。

        Args:
            z:      观测 [x, y]
            sigma2: 该观测的测量噪声方差 σ²

        Returns:
            D_M² = (z - H*x)^T * S^{-1} * (z - H*x)
            其中 S = H*P*H^T + sigma2*I
        """
        y = z - self.H @ self.x
        R = np.array([[sigma2, 0.0], [0.0, sigma2]], dtype=np.float64)
        S = self.H @ self.P @ self.H.T + R
        try:
            return float(y.T @ np.linalg.inv(S) @ y)
        except np.linalg.LinAlgError:
            return float('inf')

    @property
    def position(self) -> np.ndarray:
        return self.x[:2]

    @property
    def velocity(self) -> np.ndarray:
        return self.x[2:]

    def to_msg(self) -> TrackedObject:
        obj = TrackedObject()
        obj.id = self.id
        obj.class_id = self.class_id
        obj.position = Point(x=self.x[0], y=self.x[1], z=0.0)
        obj.vx = self.x[2]
        obj.vy = self.x[3]
        obj.confidence = self.confidence
        obj.source_flag = self.source_flag
        return obj


# ==============================================================================
# 融合节点
# ==============================================================================

class FusionNode(Node):
    """多传感器最优融合跟踪节点。"""

    SOURCE_CAM = 1
    SOURCE_LIDAR = 2
    SOURCE_RADAR = 4

    # ---- 传感器噪声模型参数 (Algorithm.md §5.5.1) ----
    # N_ref: 聚类点参考值 (经验值)
    N_REF = 30

    def __init__(self):
        super().__init__('fusion_node')
        self._declare_params()
        self._cache_params()

        # ---- TF ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- 观测缓存 (时间同步) ----
        self._cam_cache: deque = deque(maxlen=100)
        self._lidar_cache: deque = deque(maxlen=100)
        self._radar_cache: deque = deque(maxlen=100)

        # ---- 订阅 ----
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        det_topic = self.get_parameter('detection_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        radar_topic = self.get_parameter('radar_topic').value

        self._det_sub = self.create_subscription(
            Detection2DArray, det_topic, self._det_cb, qos)
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_cb, qos)
        self._radar_sub = self.create_subscription(
            RadarObjectArray, radar_topic, self._radar_cb, qos)

        # ---- 发布 ----
        tracked_topic = self.get_parameter('tracked_objects_topic').value
        self._track_pub = self.create_publisher(
            TrackedObjectArray, tracked_topic, 10)

        # ---- 跟踪目标列表 ----
        self._tracks: List[Track] = []

        # ---- 融合定时器 ----
        self._timer = self.create_timer(self._dt, self._fusion_cycle)

        self.get_logger().info(
            f'FusionNode v2.0 initialized. '
            f'time_window={self._time_window}s, gate={self._gate}m, '
            f'chi2={self._chi2_thresh}, N_ref={self.N_REF}')

    def _declare_params(self):
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('time_window', 0.1)
        self.declare_parameter('association_gate', 3.0)
        self.declare_parameter('mahalanobis_threshold', 5.991)
        self.declare_parameter('confirm_threshold', 3)
        self.declare_parameter('delete_threshold', 5)
        self.declare_parameter('max_tracks', 20)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('process_noise_q', 0.5)

        # 传感器噪声模型基准参数 (§5.5.1)
        # σ_c0: 相机基准标准差 (近距离, conf=1) [m]
        self.declare_parameter('sigma_cam_0', 0.05)
        # σ_c:  相机距离衰减尺度 [m]
        self.declare_parameter('sigma_cam_scale', 5.0)
        # σ_l0: LiDAR 基准标准差 (N_ref 点) [m]
        self.declare_parameter('sigma_lidar_0', 0.03)
        # σ_r0: 雷达基准标准差 (conf=1, v=0) [m]
        self.declare_parameter('sigma_radar_0', 0.2)
        # α:    雷达速度增益因子
        self.declare_parameter('radar_vel_alpha', 0.5)
        # v_0:  雷达参考速度 [m/s]
        self.declare_parameter('radar_vel_ref', 10.0)

        # Topic 参数
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('radar_topic', '/radar_objects')
        self.declare_parameter('tracked_objects_topic', '/tracked_objects')
        self.declare_parameter('camera_frame', 'oak_rgb_camera_optical_frame')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('radar_frame', 'radar_link')

    def _cache_params(self):
        self._dt = self.get_parameter('dt').value
        self._q = self.get_parameter('process_noise_q').value
        self._gate = self.get_parameter('association_gate').value
        self._chi2_thresh = self.get_parameter('mahalanobis_threshold').value
        self._time_window = self.get_parameter('time_window').value
        self._confirm_thresh = self.get_parameter('confirm_threshold').value
        self._delete_thresh = self.get_parameter('delete_threshold').value
        self._target_frame = self.get_parameter('target_frame').value

    # ======================================================================
    # 传感器噪声模型 (§5.5.1)
    # ======================================================================

    def _obs_variance(self, obs: Observation) -> float:
        """
        计算观测的测量噪声方差 σ²_s (Algorithm.md §5.5.1)。

        Camera:
          σ²_cam = (σ²_c0 / conf) * exp(d² / (2*σ_c²))
        LiDAR:
          σ²_lidar = σ²_l0 * (N_ref / |C|)
        Radar:
          σ²_radar = σ²_r0 / (conf * (1 + α*|v_radial|/v₀))

        Returns:
            sigma2: 该观测的测量方差
        """
        if obs.source == self.SOURCE_CAM:
            s0 = self.get_parameter('sigma_cam_0').value
            sc = self.get_parameter('sigma_cam_scale').value
            conf = max(obs.confidence, 0.01)
            d = obs.dist if obs.dist > 0 else np.linalg.norm(obs.position)
            return (s0 * s0 / conf) * math.exp(d * d / (2.0 * sc * sc))

        elif obs.source == self.SOURCE_LIDAR:
            s0 = self.get_parameter('sigma_lidar_0').value
            sz = max(obs.cluster_size, 1)
            return s0 * s0 * (self.N_REF / sz)

        elif obs.source == self.SOURCE_RADAR:
            s0 = self.get_parameter('sigma_radar_0').value
            alpha = self.get_parameter('radar_vel_alpha').value
            v0 = self.get_parameter('radar_vel_ref').value
            conf = max(obs.confidence, 0.01)
            v_radial = float(np.linalg.norm(obs.velocity))
            return s0 * s0 / (conf * (1.0 + alpha * v_radial / v0))

        return 1.0  # fallback

    # ======================================================================
    # 话题回调
    # ======================================================================

    def _to_sec(self, stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _det_cb(self, msg: Detection2DArray):
        t = self._to_sec(msg.header.stamp)
        for d in msg.detections:
            obs = Observation(x=d.position.x, y=d.position.y,
                              confidence=d.confidence,
                              source=self.SOURCE_CAM, cluster_size=1)
            obs.dist = d.depth if d.depth > 0 else float(np.linalg.norm(obs.position))
            obs.stamp = t
            self._cam_cache.append((msg.header, obs))

    def _scan_cb(self, msg: LaserScan):
        t = self._to_sec(msg.header.stamp)
        points = self._scan_to_points(msg)
        if not points:
            return
        clusters = self._euclidean_clustering(points, threshold=0.3)
        for cluster in clusters:
            if len(cluster) < 3:
                continue
            cx = np.mean([p[0] for p in cluster])
            cy = np.mean([p[1] for p in cluster])
            obs = Observation(x=cx, y=cy,
                              confidence=min(1.0, len(cluster) / self.N_REF),
                              source=self.SOURCE_LIDAR,
                              cluster_size=len(cluster))
            obs.dist = float(np.linalg.norm(obs.position))
            obs.stamp = t
            self._lidar_cache.append((msg.header, obs))

    def _radar_cb(self, msg: RadarObjectArray):
        t = self._to_sec(msg.header.stamp)
        for obj in msg.objects:
            obs = Observation(x=obj.position.x, y=obj.position.y,
                              vx=obj.vx, vy=obj.vy,
                              confidence=obj.confidence,
                              source=self.SOURCE_RADAR, cluster_size=1)
            obs.dist = float(np.linalg.norm(obs.position))
            obs.stamp = t
            self._radar_cache.append((msg.header, obs))

    # ======================================================================
    # 主融合周期
    # ======================================================================

    def _fusion_cycle(self):
        now = time.time()
        window_start = now - self._time_window

        cam_frame = self.get_parameter('camera_frame').value
        las_frame = self.get_parameter('laser_frame').value
        rad_frame = self.get_parameter('radar_frame').value

        observations = []
        observations += self._collect(window_start, self._cam_cache, cam_frame)
        observations += self._collect(window_start, self._lidar_cache, las_frame)
        observations += self._collect(window_start, self._radar_cache, rad_frame)

        if not observations and not self._tracks:
            return

        # ---- KF 预测 ----
        for track in self._tracks:
            track.predict()

        # ---- 数据关联 (§5.4) ----
        matched, unmatched_obs = self._associate(observations)

        # ---- 最优融合 + KF 更新 (§5.5, §5.6) ----
        for track, obs_list in matched.items():
            if not obs_list:
                track.miss_count += 1
                continue
            fused_z, R_fused = self._optimal_fusion(obs_list)
            track.update(fused_z, R_fused)
            track.hit_count += 1
            track.miss_count = 0
            track.confidence = np.mean([o.confidence for o in obs_list])
            track.source_flag = 0
            for o in obs_list:
                track.source_flag |= o.source
            if track.state == 'CANDIDATE' and track.hit_count >= self._confirm_thresh:
                track.state = 'CONFIRMED'

        # ---- 新目标 ----
        for obs in unmatched_obs:
            state_init = np.array([obs.position[0], obs.position[1],
                                    obs.velocity[0], obs.velocity[1]],
                                  dtype=np.float64)
            new_track = Track(state_init, self._dt, self._q,
                              confidence=obs.confidence,
                              source_flag=obs.source)
            new_track.state = 'CANDIDATE'
            self._tracks.append(new_track)

        # ---- 清理 ----
        self._tracks = [t for t in self._tracks
                        if t.miss_count <= self._delete_thresh]
        max_t = self.get_parameter('max_tracks').value
        if len(self._tracks) > max_t:
            self._tracks.sort(key=lambda t: t.confidence, reverse=True)
            self._tracks = self._tracks[:max_t]

        self._publish_tracks(now)

    # ======================================================================
    # LiDAR 处理
    # ======================================================================

    def _scan_to_points(self, msg: LaserScan) -> List[Tuple[float, float]]:
        points = []
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            points.append((r * math.cos(angle), r * math.sin(angle)))
        return points

    def _euclidean_clustering(self, points: List[Tuple[float, float]],
                              threshold: float) -> List[List[Tuple[float, float]]]:
        """欧氏距离聚类 (§5.3.2)。"""
        if len(points) < 2:
            return [points] if points else []
        clusters = []
        current = [points[0]]
        for i in range(1, len(points)):
            dx = points[i][0] - points[i-1][0]
            dy = points[i][1] - points[i-1][1]
            if dx*dx + dy*dy < threshold*threshold:
                current.append(points[i])
            else:
                clusters.append(current)
                current = [points[i]]
        clusters.append(current)
        return clusters

    # ======================================================================
    # 时间同步 + 空间对齐
    # ======================================================================

    def _collect(self, window_start: float, cache: deque,
                 source_frame: str) -> List[Observation]:
        result = []
        while cache and cache[0][1].stamp < window_start - 1.0:
            cache.popleft()
        for header, obs in cache:
            if obs.stamp < window_start:
                continue
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._target_frame, source_frame,
                    rclpy.time.Time(seconds=obs.stamp),
                    rclpy.duration.Duration(seconds=0.1),
                )
            except Exception:
                result.append(obs)
                continue
            ps = PointStamped()
            ps.header.frame_id = source_frame
            ps.point.x = obs.position[0]
            ps.point.y = obs.position[1]
            tp = do_transform_point(ps, tf)
            obs.position[0] = tp.point.x
            obs.position[1] = tp.point.y
            obs.dist = float(np.linalg.norm(obs.position))
            result.append(obs)
        return result

    # ======================================================================
    # 数据关联 (§5.4)
    # ======================================================================

    def _associate(self, observations: List[Observation]
                   ) -> Tuple[Dict[int, List[Observation]], List[Observation]]:
        if not observations:
            return {}, []

        matched: Dict[int, List[Observation]] = {}
        used_obs: set = set()
        active = [t for t in self._tracks
                  if t.state in ('CANDIDATE', 'CONFIRMED')]

        for track in active:
            candidates = []
            for i, obs in enumerate(observations):
                if i in used_obs:
                    continue
                dist = np.linalg.norm(track.position - obs.position)
                if dist < self._gate:
                    sigma2 = self._obs_variance(obs)
                    dm2 = track.mahalanobis_distance_sq(obs.position, sigma2)
                    if dm2 < self._chi2_thresh:
                        candidates.append((i, obs, dm2))
            if not candidates:
                continue
            candidates.sort(key=lambda x: x[2])
            best_idx, best_obs, _ = candidates[0]
            matched.setdefault(track.id, []).append(best_obs)
            used_obs.add(best_idx)

        unmatched = [obs for i, obs in enumerate(observations)
                     if i not in used_obs]
        return matched, unmatched

    # ======================================================================
    # 最优融合 (§5.5, Theorem 5.1)
    # ======================================================================

    def _optimal_fusion(self, observations: List[Observation]
                        ) -> Tuple[np.ndarray, np.ndarray]:
        """
        精度加权最优线性融合 (BLUE / MLE)。

        Theorem 5.1:
          λ_i = 1/σ²_i (精度)
          w_i = λ_i / Σ λ_j
          z_fused = Σ w_i * z_i
          σ²_fused = 1 / Σ λ_i
          R_fused = σ²_fused * I₂

        Returns:
            (z_fused [2,], R_fused [2×2])
        """
        n = len(observations)
        if n == 1:
            obs = observations[0]
            sigma2 = self._obs_variance(obs)
            return obs.position.copy(), np.array(
                [[sigma2, 0.0], [0.0, sigma2]], dtype=np.float64)

        precisions = np.zeros(n)
        positions = np.zeros((n, 2))

        for i, obs in enumerate(observations):
            sigma2 = self._obs_variance(obs)
            precisions[i] = 1.0 / max(sigma2, 1e-9)
            positions[i] = obs.position

        total_prec = np.sum(precisions)
        if total_prec < 1e-9:
            z_fused = np.mean(positions, axis=0)
            sigma2_fused = 1.0
        else:
            weights = precisions / total_prec
            z_fused = weights @ positions
            sigma2_fused = 1.0 / total_prec

        R_fused = np.array([[sigma2_fused, 0.0],
                            [0.0, sigma2_fused]], dtype=np.float64)
        return z_fused, R_fused

    # ======================================================================
    # 发布
    # ======================================================================

    def _publish_tracks(self, now: float):
        msg = TrackedObjectArray()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self._target_frame,
        )
        for t in self._tracks:
            if t.state in ('CANDIDATE', 'CONFIRMED'):
                msg.objects.append(t.to_msg())
        self._track_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
