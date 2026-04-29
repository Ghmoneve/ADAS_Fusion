#!/usr/bin/env python3
"""
fusion_node.py -- 多传感器自适应融合跟踪节点
==============================================

=============================================================================
算法概述 (中文伪代码)
=============================================================================

本节点实现以下核心算法：

┌─────────────────────────────────────────────────────────────────────────┐
│ 1. 时间同步                                                              │
│    维护一个大小为 time_window (默认0.1s) 的滑动缓存。                      │
│    当有新的传感器数据到达时，取出该时间窗口内所有传感器的数据。              │
│                                                                          │
│    输入: 传感器时间戳 t_sensor                                           │
│    处理:                                                                 │
│      cache.add(msg, t_sensor)                                            │
│      sync_data = cache.get_window(t_latest - time_window, t_latest)      │
│    输出: 时间对齐后的多传感器观测集合                                      │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│ 2. 空间对齐                                                              │
│    将所有传感器观测统一变换到 base_link 坐标系。                           │
│                                                                          │
│    FOR EACH 观测 Z_i IN 各传感器数据:                                     │
│        Z_i^base = TF.transform(Z_i, target_frame="base_link")            │
│    END FOR                                                               │
│                                                                          │
│    说明:                                                                  │
│    - 相机内参标定: 2D像素 → 3D相机坐标 (由OAK-D SDK/SpatialDetection)      │
│    - 外参 TF树:   相机/激光/雷达 → base_link                              │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│ 3. 目标生成                                                              │
│                                                                          │
│    视觉 (Detection2D):                                                   │
│      已有3D位置 position + depth，直接使用。                              │
│                                                                          │
│    激光雷达 (LaserScan):                                                 │
│      FOR i IN range(len(ranges)):                                        │
│          IF range_valid(ranges[i]):                                      │
│              angle = angle_min + i * angle_increment                     │
│              x = ranges[i] * cos(angle)                                  │
│              y = ranges[i] * sin(angle)                                  │
│              points.append((x, y))                                       │
│          END IF                                                          │
│      END FOR                                                             │
│      clusters = euclidean_clustering(points, threshold=0.3m)             │
│      FOR EACH cluster:                                                   │
│          obs = cluster_centroid(cluster)                                 │
│          obs.confidence ∝ cluster_size                                  │
│      END FOR                                                             │
│                                                                          │
│    毫米波雷达 (RadarObject):                                             │
│      已包含 position + velocity，直接使用。                               │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│ 4. 数据关联                                                              │
│                                                                          │
│    输入:                                                                 │
│      - 现有跟踪目标列表 T = {t_1, t_2, ..., t_N}                         │
│      - 新观测列表 O = {o_1, o_2, ..., o_M}                               │
│                                                                          │
│    步骤 4.1 — 欧氏距离门限:                                              │
│      FOR EACH t_k IN T:                                                  │
│          FOR EACH o_j IN O:                                              │
│              d_euc = sqrt((t_k.x - o_j.x)^2 + (t_k.y - o_j.y)^2)        │
│              IF d_euc < gate_threshold:                                  │
│                  candidates[k].append(o_j)                               │
│              END IF                                                      │
│          END FOR                                                         │
│      END FOR                                                             │
│                                                                          │
│    步骤 4.2 — 马氏距离检验 (最近邻匹配):                                  │
│      FOR EACH t_k WITH candidates:                                       │
│          FOR EACH o_j IN candidates:                                     │
│              z = [o_j.x, o_j.y]^T                                        │
│              H = [[1,0,0,0],[0,1,0,0]]                                   │
│              y = z - H * x_pred                                          │
│              S = H * P_pred * H^T + R                                    │
│              D_M^2 = y^T * S^{-1} * y                                    │
│              IF D_M^2 < chi2_95:                                         │
│                  接受 H0: 该观测属于当前目标                              │
│                  匹配池 += (o_j, D_M^2)                                  │
│              ELSE:                                                       │
│                  拒绝 H0: 该观测为离群值/噪声                             │
│              END IF                                                      │
│          END FOR                                                         │
│          最佳匹配 = argmin(D_M^2)                                        │
│          t_k 关联 最佳匹配                                                │
│      END FOR                                                             │
│                                                                          │
│      未关联的观测 → 初始化为候选目标                                      │
│      未关联的跟踪 → 标记为 miss_count++                                   │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│ 5. 自适应贝叶斯融合 (核心创新)                                            │
│                                                                          │
│    背景:                                                                  │
│      不同传感器在不同环境下可靠性不同:                                     │
│      - 雨雾: 视觉置信度下降                                               │
│      - 远距离: LiDAR 点稀疏                                               │
│      - 动态目标: 雷达有速度信息, 优势明显                                  │
│                                                                          │
│    贝叶斯模型:                                                            │
│      P(X | Z) ∝ P(Z | X) * P(X)                                         │
│      其中:                                                                │
│        X = 目标真实状态                                                   │
│        Z = {Z_cam, Z_lidar, Z_radar} 传感器观测集合                       │
│        P(Z | X) = 观测模型 (似然, 体现传感器可信度)                        │
│        P(X) = 先验分布 (来自 Kalman Filter 预测)                          │
│                                                                          │
│    步骤 5.1 — 传感器权重计算:                                             │
│      w_cam_i = conf_i * exp(-d_i^2 / (2 * σ_cam^2))                      │
│        说明: 距离越远, 双目深度精度 ↓ → 权重 ↓                            │
│                                                                          │
│      w_lidar_i = cluster_quality * exp(-d_i^2 / (2 * σ_lidar^2))         │
│        说明: 点云数越大, 聚类质量 ↑ → 权重 ↑                              │
│                                                                          │
│      w_radar_i = conf_i * (1 + α * |v_radial|)                           │
│        说明: 径向速度越大, 动态目标识别优势越明显 → 权重 ↑                 │
│                                                                          │
│      归一化: w_sum = w_cam + w_lidar + w_radar                           │
│               w_cam /= w_sum, w_lidar /= w_sum, w_radar /= w_sum         │
│                                                                          │
│    步骤 5.2 — 假设检验 (离群值剔除):                                      │
│      FOR EACH 观测 Z_k:                                                  │
│          D_M^2 = (Z_k - H*x_pred)^T * S^{-1} * (Z_k - H*x_pred)         │
│          IF D_M^2 > chi2_{0.95}(df=2):                                   │
│              Z_k 标记为离群值, 不参与当前融合                             │
│          END IF                                                          │
│      END FOR                                                             │
│                                                                          │
│    步骤 5.3 — 加权融合:                                                   │
│      X_fused = Σ w_i * Z_i  (局部加权平均)                               │
│                                                                          │
│    步骤 5.4 — 先验-后验融合:                                              │
│      α = mean(confidences)  (自适应信任度)                                │
│      X_posterior = α * X_fused + (1-α) * X_prior                        │
│                                                                          │
│    输出: X_fused_posterior (融合后的位置估计)                              │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│ 6. Kalman Filter 状态估计                                                │
│                                                                          │
│    状态向量: x = [px, py, vx, vy]^T                                      │
│       px, py: 目标在 base_link 下的位置                                   │
│       vx, vy: 目标在 base_link 下的速度                                   │
│                                                                          │
│    运动模型 (恒定速度模型 CV):                                            │
│      x_{k+1} = F * x_k + w_k,  w_k ~ N(0, Q)                            │
│                                                                          │
│      状态转移矩阵 F (dt = 采样间隔):                                      │
│        F = [[1, 0, dt, 0 ],                                              │
│             [0, 1, 0,  dt],                                              │
│             [0, 0, 1,  0 ],                                              │
│             [0, 0, 0,  1 ]]                                              │
│                                                                          │
│      过程噪声协方差 Q (假设加速度为白噪声):                                │
│        Q = q * [[dt^3/3, 0,       dt^2/2, 0      ],                      │
│                 [0,       dt^3/3, 0,       dt^2/2],                      │
│                 [dt^2/2, 0,       dt,      0      ],                      │
│                 [0,       dt^2/2, 0,       dt     ]]                      │
│        其中 q = process_noise_q (加速度方差)                               │
│                                                                          │
│    观测模型:                                                              │
│      z = H * x + v_k,  v_k ~ N(0, R)                                    │
│                                                                          │
│      观测矩阵 H (仅观测位置):                                            │
│        H = [[1, 0, 0, 0],                                                │
│             [0, 1, 0, 0]]                                                │
│                                                                          │
│      观测噪声协方差 R:                                                    │
│        R = r * [[1, 0],                                                   │
│                 [0, 1]]                                                   │
│        其中 r = measurement_noise_r                                       │
│                                                                          │
│    步骤 6.1 — 预测:                                                      │
│      x_pred = F @ x_prev                                                 │
│      P_pred = F @ P_prev @ F^T + Q                                       │
│                                                                          │
│    步骤 6.2 — 更新 (使用融合后的观测 Z_fused):                             │
│      y = Z_fused - H @ x_pred           (创新/残差)                       │
│      S = H @ P_pred @ H^T + R           (创新协方差)                      │
│      K = P_pred @ H^T @ inv(S)          (卡尔曼增益)                      │
│                                                                          │
│      x_new = x_pred + K @ y             (状态更新)                        │
│      P_new = (I - K @ H) @ P_pred       (协方差更新)                      │
│                                                                          │
│    输出: 平滑后的位置 (px, py) 和速度 (vx, vy)                            │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│ 7. 目标生命周期管理                                                       │
│                                                                          │
│    状态机:                                                                │
│      CANDIDATE → CONFIRMED → LOST → DELETED                              │
│                                                                          │
│    新目标:                                                                │
│      IF 未关联观测连续 3 帧出现在相近位置:                                 │
│          状态 → CONFIRMED, 分配唯一 ID                                    │
│      END IF                                                              │
│                                                                          │
│    目标删除:                                                              │
│      IF miss_count > delete_threshold (默认 5 帧):                        │
│          状态 → DELETED, 从跟踪列表移除                                   │
│      END IF                                                              │
└─────────────────────────────────────────────────────────────────────────┘

=============================================================================
实现开始
=============================================================================
"""

import math
import time
import numpy as np
from collections import deque
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# ROS 消息
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Header

# 自定义消息
from adas_fusion_msgs.msg import Detection2DArray, RadarObjectArray
from adas_fusion_msgs.msg import TrackedObject, TrackedObjectArray

# TF2
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


# ==============================================================================
# 数据结构
# ==============================================================================

class Observation:
    """单个传感器观测 (经过空间对齐后)。"""
    __slots__ = ('position', 'velocity', 'confidence', 'source',
                 'cluster_size', 'stamp')

    def __init__(self, x=0.0, y=0.0, vx=0.0, vy=0.0,
                 confidence=0.0, source=0, cluster_size=1):
        self.position = np.array([x, y], dtype=np.float64)
        self.velocity = np.array([vx, vy], dtype=np.float64)
        self.confidence = confidence
        self.source = source          # 1=camera, 2=lidar, 4=radar
        self.cluster_size = cluster_size
        self.stamp = 0.0


class Track:
    """单个跟踪目标，内部维护一个 Kalman Filter。"""
    __slots__ = ('id', 'x', 'P', 'F', 'H', 'Q', 'R', 'class_id',
                 'confidence', 'source_flag', 'miss_count', 'hit_count',
                 'state', 'last_update')

    _next_id = 0

    def __init__(self, state_init: np.ndarray, dt: float,
                 q: float, r: float, class_id: int = -1,
                 confidence: float = 0.0, source_flag: int = 0):
        Track._next_id += 1
        self.id = Track._next_id
        self.x = state_init.copy()            # [px, py, vx, vy]
        self.P = np.eye(4) * 10.0             # 初始协方差 (大不确定度)

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

        # 过程噪声 Q = q * G * G^T
        # G = [dt^2/2, dt^2/2, dt, dt] 作为加速度输入
        dt2 = dt * dt / 2.0
        G = np.array([
            [dt2, 0],
            [0, dt2],
            [dt, 0],
            [0, dt],
        ], dtype=np.float64)
        self.Q = q * (G @ G.T)

        # 观测噪声
        self.R = np.array([
            [r, 0],
            [0, r],
        ], dtype=np.float64)

        self.class_id = class_id
        self.confidence = confidence
        self.source_flag = source_flag
        self.miss_count = 0
        self.hit_count = 1          # 初始化算一次观测
        self.state = 'CANDIDATE'    # CANDIDATE → CONFIRMED → LOST
        self.last_update = time.time()

    def predict(self):
        """Kalman 预测步骤。"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z: np.ndarray):
        """
        Kalman 更新步骤。

        Args:
            z: 观测 [x, y] (融合后的位置)
        """
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = self.P @ self.H.T @ S_inv
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.last_update = time.time()

    def mahalanobis_distance_sq(self, z: np.ndarray) -> float:
        """
        计算马氏距离的平方 D_M^2。

        Args:
            z: 观测 [x, y]

        Returns:
            D_M^2 = (z - H*x)^T * S^{-1} * (z - H*x)
        """
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
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
    """多传感器自适应融合跟踪节点。"""

    SOURCE_CAM = 1
    SOURCE_LIDAR = 2
    SOURCE_RADAR = 4

    def __init__(self):
        super().__init__('fusion_node')

        # ---- 参数 ----
        self._declare_params()

        # 参数缓存
        self._dt = self.get_parameter('dt').value
        self._q = self.get_parameter('process_noise_q').value
        self._r = self.get_parameter('measurement_noise_r').value
        self._gate = self.get_parameter('association_gate').value
        self._chi2_thresh = self.get_parameter('mahalanobis_threshold').value
        self._time_window = self.get_parameter('time_window').value
        self._confirm_thresh = self.get_parameter('confirm_threshold').value
        self._delete_thresh = self.get_parameter('delete_threshold').value

        self._target_frame = self.get_parameter('target_frame').value

        # 传感器先验权重
        self._w_cam_base = self.get_parameter('camera_weight').value
        self._w_lidar_base = self.get_parameter('lidar_weight').value
        self._w_radar_base = self.get_parameter('radar_weight').value

        # ---- TF ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- 观测缓存 (时间同步) ----
        self._cam_cache: deque = deque(maxlen=100)
        self._lidar_cache: deque = deque(maxlen=100)
        self._radar_cache: deque = deque(maxlen=100)

        # ---- 话题订阅 ----
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

        # ---- 定时器: 周期性触发融合 ----
        self._timer = self.create_timer(self._dt, self._fusion_cycle)

        self.get_logger().info(
            'FusionNode initialized. '
            f'Time window: {self._time_window}s, gate: {self._gate}m, '
            f'chi2_thresh: {self._chi2_thresh}')

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
        self.declare_parameter('measurement_noise_r', 0.1)
        self.declare_parameter('camera_weight', 0.35)
        self.declare_parameter('lidar_weight', 0.35)
        self.declare_parameter('radar_weight', 0.30)
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('radar_topic', '/radar_objects')
        self.declare_parameter('tracked_objects_topic', '/tracked_objects')

        # 传感器 frame
        self.declare_parameter('camera_frame', 'oak_rgb_camera_optical_frame')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('radar_frame', 'radar_link')

    # ======================================================================
    # 话题回调 (存入缓存)
    # ======================================================================

    def _to_sec(self, stamp) -> float:
        """将 ROS Header.stamp 转为秒。"""
        return stamp.sec + stamp.nanosec * 1e-9

    def _det_cb(self, msg: Detection2DArray):
        t = self._to_sec(msg.header.stamp)
        for d in msg.detections:
            obs = Observation(
                x=d.position.x, y=d.position.y,
                confidence=d.confidence,
                source=self.SOURCE_CAM,
                cluster_size=1,
            )
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
            obs = Observation(
                x=cx, y=cy,
                confidence=min(1.0, len(cluster) / 30.0),
                source=self.SOURCE_LIDAR,
                cluster_size=len(cluster),
            )
            obs.stamp = t
            self._lidar_cache.append((msg.header, obs))

    def _radar_cb(self, msg: RadarObjectArray):
        t = self._to_sec(msg.header.stamp)
        for obj in msg.objects:
            obs = Observation(
                x=obj.position.x, y=obj.position.y,
                vx=obj.vx, vy=obj.vy,
                confidence=obj.confidence,
                source=self.SOURCE_RADAR,
                cluster_size=1,
            )
            obs.stamp = t
            self._radar_cache.append((msg.header, obs))

    # ======================================================================
    # 主融合周期
    # ======================================================================

    def _fusion_cycle(self):
        """
        每个时间步触发一次融合:
          时间同步 → 空间对齐 → 数据关联 → 自适应融合 → KF 更新 → 发布
        """
        now = time.time()
        window_start = now - self._time_window

        # ---- Step 1+2: 时间同步 + 空间对齐 ----
        cam_frame = self.get_parameter('camera_frame').value
        las_frame = self.get_parameter('laser_frame').value
        rad_frame = self.get_parameter('radar_frame').value

        observations = []
        observations += self._collect(window_start, self._cam_cache, cam_frame)
        observations += self._collect(window_start, self._lidar_cache, las_frame)
        observations += self._collect(window_start, self._radar_cache, rad_frame)

        if not observations and not self._tracks:
            return

        # ---- 对每个目标执行 KF 预测 ----
        for track in self._tracks:
            track.predict()

        # ---- Step 4: 数据关联 ----
        matched, unmatched_obs = self._associate(observations)

        # ---- Step 5+6: 自适应融合 + KF 更新 ----
        for track, obs_list in matched.items():
            if not obs_list:
                track.miss_count += 1
                continue
            # 融合多个观测
            fused_z = self._adaptive_fusion(track, obs_list)
            # KF 更新
            track.update(fused_z)
            track.hit_count += 1
            track.miss_count = 0
            track.confidence = np.mean([o.confidence for o in obs_list])
            # 更新来源标志
            track.source_flag = 0
            for o in obs_list:
                track.source_flag |= o.source
            # 确认逻辑
            if track.state == 'CANDIDATE' and track.hit_count >= self._confirm_thresh:
                track.state = 'CONFIRMED'

        # ---- Step 7: 未关联观测 → 候选新目标 ----
        for obs in unmatched_obs:
            # 简单初始化 (无速度信息时设为0)
            state_init = np.array([obs.position[0], obs.position[1],
                                    obs.velocity[0], obs.velocity[1]],
                                  dtype=np.float64)
            new_track = Track(state_init, self._dt, self._q, self._r,
                              confidence=obs.confidence,
                              source_flag=obs.source)
            new_track.state = 'CANDIDATE'
            self._tracks.append(new_track)

        # ---- 目标清理 ----
        self._tracks = [t for t in self._tracks
                        if t.miss_count <= self._delete_thresh]
        # 限制最大跟踪数
        max_t = self.get_parameter('max_tracks').value
        if len(self._tracks) > max_t:
            # 按置信度排序, 保留最好的
            self._tracks.sort(key=lambda t: t.confidence, reverse=True)
            self._tracks = self._tracks[:max_t]

        # ---- 发布 ----
        self._publish_tracks(now)

    # ======================================================================
    # 激光雷达处理
    # ======================================================================

    def _scan_to_points(self, msg: LaserScan) -> List[Tuple[float, float]]:
        """从 LaserScan 消息提取 2D 点。"""
        points = []
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y))
        return points

    def _euclidean_clustering(self, points: List[Tuple[float, float]],
                              threshold: float) -> List[List[Tuple[float, float]]]:
        """
        简单欧氏距离聚类。

        算法:
          对排序后的点逐一遍历, 若相邻点距离 < threshold → 归入同一聚类。
        """
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
        """
        取出时间窗口内的观测, 并变换到 target_frame。

        步骤:
          1. 遍历缓存, 丢弃过期数据
          2. 取窗口内的观测
          3. 查找 TF, 将观测位置变换到 target_frame
        """
        result = []
        # 丢弃过期
        while cache and cache[0][1].stamp < window_start - 1.0:
            cache.popleft()
        # 收集窗口内的观测
        for header, obs in cache:
            if obs.stamp < window_start:
                continue
            # 查找 TF
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    source_frame,
                    rclpy.time.Time(seconds=obs.stamp),
                    rclpy.duration.Duration(seconds=0.1),
                )
            except Exception:
                # TF 不可用, 使用原始坐标 (假设已对齐)
                result.append(obs)
                continue
            # 变换位置
            ps = PointStamped()
            ps.header.frame_id = source_frame
            ps.point.x = obs.position[0]
            ps.point.y = obs.position[1]
            ps.point.z = 0.0
            tp = do_transform_point(ps, tf)
            obs.position[0] = tp.point.x
            obs.position[1] = tp.point.y
            result.append(obs)
        return result

    # ======================================================================
    # 数据关联
    # ======================================================================

    def _associate(self, observations: List[Observation]
                   ) -> Tuple[Dict[int, List[Observation]], List[Observation]]:
        """
        将观测关联到现有跟踪目标。

        步骤:
          1. 对每个 track, 计算与所有观测的欧氏距离
          2. 距离小于 gate_threshold 的观测为候选
          3. 对候选计算马氏距离, 满足 chi2 检验后取最近邻
          4. 未关联的观测返回用作新目标初始化

        Returns:
          (matched, unmatched_obs)
            matched:     {track.id: [obs, ...]}
            unmatched:   未关联的观测列表
        """
        if not observations:
            return {}, []

        matched: Dict[int, List[Observation]] = {}
        used_obs: set = set()

        # 仅处理已确认的目标
        confirmed_tracks = [t for t in self._tracks
                            if t.state in ('CANDIDATE', 'CONFIRMED')]

        for track in confirmed_tracks:
            candidates = []
            for i, obs in enumerate(observations):
                if i in used_obs:
                    continue
                # 欧氏距离门限
                dist = np.linalg.norm(track.position - obs.position)
                if dist < self._gate:
                    # 马氏距离检验
                    dm2 = track.mahalanobis_distance_sq(obs.position)
                    if dm2 < self._chi2_thresh:
                        candidates.append((i, obs, dm2))

            if not candidates:
                continue

            # 最近邻 (最小马氏距离)
            candidates.sort(key=lambda x: x[2])
            best_idx, best_obs, _ = candidates[0]

            if track.id not in matched:
                matched[track.id] = []
            matched[track.id].append(best_obs)
            used_obs.add(best_idx)

        # 收集未关联观测
        unmatched = [obs for i, obs in enumerate(observations)
                     if i not in used_obs]

        return matched, unmatched

    # ======================================================================
    # 自适应贝叶斯融合
    # ======================================================================

    def _adaptive_fusion(self, track: Track,
                         observations: List[Observation]) -> np.ndarray:
        """
        自适应融合多个传感器的观测。

        算法:
          1. 计算每个观测的自适应权重
          2. 假设检验剔除离群值 (已在 _associate 中完成马氏检验)
          3. 加权平均得到 X_fused
          4. 先验后验融合

        Args:
            track: 目标跟踪器 (包含预测状态 x_pred)
            observations: 该目标关联的观测列表

        Returns:
            fused_z: 融合后的观测 [x, y]
        """
        if len(observations) == 1:
            return observations[0].position.copy()

        n = len(observations)
        weights = np.zeros(n)
        positions = np.zeros((n, 2))
        confidences = np.zeros(n)

        for i, obs in enumerate(observations):
            positions[i] = obs.position
            confidences[i] = obs.confidence

            # 距离因子: 越远越不可靠 (对视觉和激光影响大)
            dist = np.linalg.norm(obs.position)
            dist_factor = math.exp(-dist * dist / 50.0)  # σ=5m 高斯衰减

            # 计算各传感器权重
            if obs.source == self.SOURCE_CAM:
                weights[i] = self._w_cam_base * confidences[i] * dist_factor
            elif obs.source == self.SOURCE_LIDAR:
                # 激光: 点云数量反映聚类质量
                cluster_factor = min(1.0, obs.cluster_size / 20.0)
                weights[i] = (self._w_lidar_base * cluster_factor *
                              dist_factor)
            elif obs.source == self.SOURCE_RADAR:
                # 雷达: 速度信息是优势
                vel_mag = float(np.linalg.norm(obs.velocity))
                vel_factor = 1.0 + min(0.5, vel_mag / 10.0)
                weights[i] = (self._w_radar_base * confidences[i] *
                              vel_factor)
            else:
                weights[i] = 0.01

        # 归一化权重
        w_sum = np.sum(weights)
        if w_sum < 1e-9:
            # 所有传感器均不可靠, 回退到均值
            return np.mean(positions, axis=0)
        weights /= w_sum

        # 加权平均 (局部融合)
        x_fused = np.average(positions, axis=0, weights=weights)

        # 先验-后验融合
        # α = mean(confidences): 观测越可靠, 越相信观测; 否则更相信预测
        alpha = np.clip(np.mean(confidences), 0.1, 0.9)
        x_prior = track.position
        x_posterior = alpha * x_fused + (1.0 - alpha) * x_prior

        return x_posterior

    # ======================================================================
    # 发布
    # ======================================================================

    def _publish_tracks(self, now: float):
        """发布已确认的跟踪目标。"""
        msg = TrackedObjectArray()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self._target_frame,
        )
        for t in self._tracks:
            if t.state in ('CANDIDATE', 'CONFIRMED'):
                msg.objects.append(t.to_msg())
        self._track_pub.publish(msg)


# ==============================================================================
# 入口
# ==============================================================================

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
