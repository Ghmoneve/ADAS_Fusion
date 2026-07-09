"""
types.py — 核心数据结构 (传感器无关, 零 ROS 依赖)

所有融合算法模块共享的基础类型。
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional, Tuple
import numpy as np


# ==============================================================================
# Observation
# ==============================================================================

@dataclass
class Observation:
    """单个传感器观测 (已完成空间对齐到统一参考系)。

    Attributes:
        position:   [x, y] 目标位置 (统一参考系, 单位: m)
        velocity:   [vx, vy] 目标速度 (统一参考系, 单位: m/s), 可为 [0,0]
        confidence: 置信度 [0.0, 1.0], 由传感器提供
        sensor_id:  传感器标识符 (如 "camera", "lidar", "radar")
        metadata:   传感器特定元数据 (如聚类点数、深度距离、径向速度等),
                    供噪声模型使用
        timestamp:  Unix 时间戳 (秒), 用于时间同步
    """
    position:   np.ndarray
    velocity:   np.ndarray
    confidence: float
    sensor_id:  str
    metadata:   Dict[str, Any] = field(default_factory=dict)
    timestamp:  float = 0.0

    def __post_init__(self):
        if self.position.shape != (2,):
            raise ValueError(f"position must be shape (2,), got {self.position.shape}")
        if self.velocity.shape != (2,):
            raise ValueError(f"velocity must be shape (2,), got {self.velocity.shape}")
        self.confidence = float(np.clip(self.confidence, 0.0, 1.0))


# ==============================================================================
# SensorConfig
# ==============================================================================

# 噪声模型回调签名
#   输入: Observation → 输出: sigma^2 (float, 测量方差)
NoiseModel = Callable[[Observation], float]


@dataclass
class SensorConfig:
    """传感器注册配置。

    Attributes:
        sensor_id:   全局唯一传感器标识符
        noise_model: 噪声模型回调 f(obs) → sigma²
                     - 预置: SensorPresets.camera() / lidar() / radar()
                     - 自定义: lambda obs: 0.01 / obs.confidence
        frame_id:    TF 源坐标系 (如 "camera_optical_frame", "laser")
        bit_mask:    内部位掩码, 由 FusionEngine 自动分配
    """
    sensor_id:   str
    noise_model: NoiseModel
    frame_id:    str = ""
    bit_mask:    int = 0


# ==============================================================================
# Track
# ==============================================================================

@dataclass
class Track:
    """融合跟踪目标。

    Attributes:
        id:          全局唯一跟踪 ID
        state:       [px, py, vx, vy] Kalman 状态估计
        covariance:  [4,4] 状态协方差矩阵
        confidence:  综合置信度 [0, 1]
        source_mask: 来源传感器位掩码
        hit_count:   累计命中帧数
        miss_count:  连续丢失帧数
        status:      "CANDIDATE" | "CONFIRMED"
        metadata:    用户自定义元数据
    """
    id:          int
    state:       np.ndarray          # [px, py, vx, vy]
    covariance:  np.ndarray          # [4, 4]
    confidence:  float = 0.0
    source_mask: int = 0
    hit_count:   int = 0
    miss_count:  int = 0
    status:      str = "CANDIDATE"
    metadata:    Dict[str, Any] = field(default_factory=dict)

    @property
    def position(self) -> np.ndarray:
        return self.state[:2]

    @property
    def velocity(self) -> np.ndarray:
        return self.state[2:]

    @property
    def position_covariance(self) -> np.ndarray:
        return self.covariance[:2, :2]


# ==============================================================================
# FusionConfig
# ==============================================================================

@dataclass
class FusionConfig:
    """融合引擎全局参数。

    Attributes:
        dt:                  Kalman Filter 离散时间步长 (s)
        process_noise_q:     DWNA 过程噪声系数 (加速度方差)
        association_gate:    欧氏距离关联门限 (m)
        chi2_threshold:      马氏距离卡方阈值 (df=2, 95% → 5.991)
        time_window:         时间同步窗口 (s)
        confirm_threshold:   连续命中 N 帧后确认目标
        delete_threshold:    连续丢失 N 帧后删除目标
        max_tracks:          最大跟踪目标数
        transform_callback:  坐标变换回调
                             f(source_frame, target_frame, timestamp, [x,y]) → [x',y']
                             若为 None, 跳过空间对齐
    """
    dt:                  float = 0.1
    process_noise_q:     float = 0.5
    association_gate:    float = 3.0
    chi2_threshold:      float = 5.991
    time_window:         float = 0.1
    confirm_threshold:   int   = 3
    delete_threshold:    int   = 5
    max_tracks:          int   = 20
    target_frame:        str   = "base_link"
    transform_callback:  Optional[Callable] = None
