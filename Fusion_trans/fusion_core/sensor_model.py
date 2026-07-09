"""
sensor_model.py — 传感器噪声模型注册表 (Algorithm.md §5.5.1)

提供三种预置模型和一个自定义接口:
  - camera(d, conf)   → σ² = (σ₀²/conf) * exp(d²/(2σ_c²))
  - lidar(|C|)         → σ² = σ₀² * (N_ref / |C|)
  - radar(conf, |v|)  → σ² = σ₀² / (conf * (1 + α|v|/v₀))
  - custom(callback)   → σ² = callback(obs)

用法:
  from fusion_core.sensor_model import SensorPresets

  noise_fn = SensorPresets.camera(sigma_0=0.05, sigma_c=5.0)
  sigma2 = noise_fn(obs)   # obs 为 Observation 实例
"""

from __future__ import annotations
from typing import Callable
import math
import numpy as np
from .types import Observation, NoiseModel


class SensorPresets:
    """预置传感器噪声模型工厂。"""

    @staticmethod
    def camera(sigma_0: float = 0.05,
               sigma_c: float = 5.0) -> NoiseModel:
        """
        视觉 (双目深度) 噪声模型。

        σ²_cam = (σ₀² / conf) * exp(d² / (2 * σ_c²))

        Args:
            sigma_0: 基准标准差 (近距离, conf=1) [m]
            sigma_c: 距离衰减尺度 [m]
        """
        s0_sq = sigma_0 * sigma_0
        sc_sq_2 = 2.0 * sigma_c * sigma_c

        def _noise(obs: Observation) -> float:
            conf = max(obs.confidence, 0.01)
            dist = obs.metadata.get('dist', 0.0)
            if dist <= 0.0 and obs.position is not None:
                dist = float(np.linalg.norm(obs.position))
            return (s0_sq / conf) * math.exp(dist * dist / sc_sq_2)

        return _noise

    @staticmethod
    def lidar(sigma_0: float = 0.03,
              n_ref: float = 30.0) -> NoiseModel:
        """
        LiDAR 聚类噪声模型。

        σ²_lidar = σ₀² * (N_ref / |C|)

        Args:
            sigma_0: 基准标准差 (N_ref 个点时) [m]
            n_ref:   参考聚类点数
        """
        s0_sq = sigma_0 * sigma_0

        def _noise(obs: Observation) -> float:
            cluster_size = max(obs.metadata.get('cluster_size', 1), 1)
            return s0_sq * (n_ref / cluster_size)

        return _noise

    @staticmethod
    def radar(sigma_0: float = 0.2,
              alpha: float = 0.5,
              v_ref: float = 10.0) -> NoiseModel:
        """
        毫米波雷达噪声模型。

        σ²_radar = σ₀² / (conf * (1 + α * |v_radial| / v₀))

        Args:
            sigma_0: 基准标准差 (conf=1, v=0) [m]
            alpha:   速度增益因子
            v_ref:   参考速度 [m/s]
        """
        s0_sq = sigma_0 * sigma_0

        def _noise(obs: Observation) -> float:
            conf = max(obs.confidence, 0.01)
            v_radial = obs.metadata.get('v_radial', 0.0)
            if v_radial <= 0.0 and obs.velocity is not None:
                v_radial = float(np.linalg.norm(obs.velocity))
            return s0_sq / (conf * (1.0 + alpha * v_radial / v_ref))

        return _noise

    @staticmethod
    def custom(callback: Callable[[Observation], float]) -> NoiseModel:
        """自定义噪声模型。

        Args:
            callback: f(obs) → sigma²
        """
        return callback

    @staticmethod
    def constant(sigma2: float) -> NoiseModel:
        """常量噪声模型。σ² = sigma2。"""
        return lambda _obs: sigma2
