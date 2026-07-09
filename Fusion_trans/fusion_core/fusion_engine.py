"""
fusion_engine.py — 多传感器最优融合引擎 (传感器无关)

主编排器: 传感器注册 → 时间同步 → 空间对齐 → 数据关联 → 最优融合 → KF 跟踪

用法:
    engine = FusionEngine(config)
    engine.register_sensor("lidar", noise_fn, frame_id="laser")
    engine.add_observation(obs)
    tracks = engine.step()   # 每 dt 秒调用一次
"""

from __future__ import annotations
from collections import deque
from typing import Dict, List, Optional
import time
import numpy as np

from .types import (Observation, SensorConfig, Track,
                    FusionConfig, NoiseModel)
from .kalman_filter import KalmanFilter
from .sensor_model import SensorPresets
from .association import associate
from .track_manager import TrackManager


class FusionEngine:
    """多传感器最优融合引擎。

    核心流水线 (每次 step()):
      predict → time_sync → spatial_align → associate → fuse → update → publish

    传感器无关: 通过 register_sensor() 注册任意传感器 + 噪声模型。
    """

    def __init__(self, config: Optional[FusionConfig] = None):
        self.config = config or FusionConfig()
        self._sensors: Dict[str, SensorConfig] = {}
        self._bit_counter: int = 0
        self._manager = TrackManager(
            confirm_threshold=self.config.confirm_threshold,
            delete_threshold=self.config.delete_threshold,
            max_tracks=self.config.max_tracks,
        )
        self._obs_cache: Dict[str, deque] = {}

    # ======================================================================
    # 传感器注册
    # ======================================================================

    def register_sensor(self, sensor_id: str,
                        noise_model: Optional[NoiseModel] = None,
                        frame_id: str = "",
                        **preset_kwargs) -> SensorConfig:
        """
        注册传感器。

        Args:
            sensor_id:  全局唯一标识符 (如 "camera_front", "lidar_main")
            noise_model: 噪声模型 f(obs) → sigma².
                        若为 None, 使用 preset_kwargs 从 SensorPresets 构造.
            frame_id:   TF 源坐标系 (用于空间对齐), 空字符串表示无需变换
            **preset_kwargs: 传递给 SensorPresets 的参数
                            (仅当 noise_model 为 None 时生效)
                            type="camera" → SensorPresets.camera(sigma_0=..., sigma_c=...)
                            type="lidar"  → SensorPresets.lidar(sigma_0=..., n_ref=...)
                            type="radar"  → SensorPresets.radar(sigma_0=..., alpha=..., v_ref=...)
                            type="custom" → SensorPresets.custom(callback=...)
                            type="constant" → SensorPresets.constant(sigma2=...)

        Returns:
            SensorConfig 对象

        Example:
            # 预置模型
            engine.register_sensor("lidar", type="lidar", sigma_0=0.03, n_ref=30)

            # 自定义噪声
            engine.register_sensor("camera", lambda obs: 0.01 / obs.confidence)

            # 常量噪声
            engine.register_sensor("radar", type="constant", sigma2=0.1)
        """
        if self._bit_counter >= 32:
            raise RuntimeError("Maximum 32 sensor types supported")

        if noise_model is None:
            stype = preset_kwargs.pop('type', 'custom')
            if stype == 'camera':
                noise_model = SensorPresets.camera(
                    sigma_0=preset_kwargs.pop('sigma_0', 0.05),
                    sigma_c=preset_kwargs.pop('sigma_c', 5.0))
            elif stype == 'lidar':
                noise_model = SensorPresets.lidar(
                    sigma_0=preset_kwargs.pop('sigma_0', 0.03),
                    n_ref=preset_kwargs.pop('n_ref', 30))
            elif stype == 'radar':
                noise_model = SensorPresets.radar(
                    sigma_0=preset_kwargs.pop('sigma_0', 0.2),
                    alpha=preset_kwargs.pop('alpha', 0.5),
                    v_ref=preset_kwargs.pop('v_ref', 10.0))
            elif stype == 'constant':
                noise_model = SensorPresets.constant(
                    sigma2=preset_kwargs.pop('sigma2', 0.01))

        bit_mask = 1 << self._bit_counter
        self._bit_counter += 1

        cfg = SensorConfig(
            sensor_id=sensor_id,
            noise_model=noise_model,
            frame_id=frame_id,
            bit_mask=bit_mask,
        )
        self._sensors[sensor_id] = cfg
        self._obs_cache[sensor_id] = deque(maxlen=200)
        return cfg

    def unregister_sensor(self, sensor_id: str):
        """注销传感器。"""
        self._sensors.pop(sensor_id, None)
        self._obs_cache.pop(sensor_id, None)

    @property
    def sensors(self) -> Dict[str, SensorConfig]:
        return dict(self._sensors)

    # ======================================================================
    # 观测输入
    # ======================================================================

    def add_observation(self, sensor_id: str,
                        position: "array-like",
                        velocity: "array-like" = (0.0, 0.0),
                        confidence: float = 0.5,
                        timestamp: Optional[float] = None,
                        **metadata) -> Optional[Observation]:
        """
        喂入一个传感器观测。

        Args:
            sensor_id:  传感器标识符 (需已注册)
            position:   [x, y] 目标位置 (m)
            velocity:   [vx, vy] 目标速度 (m/s)
            confidence: 置信度 [0, 1]
            timestamp:  Unix 时间戳 (秒), None=当前时间
            **metadata: 传感器特定元数据
                        (如 cluster_size=15, dist=3.2, v_radial=1.5)

        Returns:
            Observation 对象, 若传感器未注册则返回 None
        """
        if sensor_id not in self._sensors:
            return None

        pos = np.array(position, dtype=np.float64)
        vel = np.array(velocity, dtype=np.float64)
        ts = timestamp if timestamp is not None else time.time()

        obs = Observation(
            position=pos, velocity=vel,
            confidence=confidence, sensor_id=sensor_id,
            metadata=metadata, timestamp=ts,
        )
        self._obs_cache[sensor_id].append(obs)
        return obs

    # ======================================================================
    # 融合步进
    # ======================================================================

    def step(self) -> List[Track]:
        """
        执行一次融合周期。

        流水线: predict → time_sync → spatial_align → associate → fuse → update → purge

        Returns:
            当前所有活跃跟踪目标列表
        """
        now = time.time()
        window_start = now - self.config.time_window

        # 1. 收集时间窗口内的观测 + 空间对齐
        observations: List[Observation] = []
        for sid, cfg in self._sensors.items():
            cache = self._obs_cache[sid]
            # 丢弃过期
            while cache and cache[0].timestamp < window_start - 1.0:
                cache.popleft()
            for obs in cache:
                if obs.timestamp >= window_start:
                    self._apply_transform(obs, cfg.frame_id)
                    observations.append(obs)

        # 2. KF 预测
        self._manager.predict_all()

        # 3. 数据关联
        noise_map = {sid: cfg.noise_model for sid, cfg in self._sensors.items()}
        matched, unmatched = associate(
            self._manager.active_tracks,
            observations,
            self.config.association_gate,
            self.config.chi2_threshold,
            noise_map,
        )

        # 4. 最优融合 + KF 更新
        for track_id, obs_list in matched.items():
            track = self._find_track(track_id)
            if track is None:
                continue
            if obs_list:
                fused_z, R_fused = self._optimal_fusion(obs_list)
                self._manager.update_track(
                    track, fused_z, R_fused,
                    confidence=float(np.mean([o.confidence for o in obs_list])),
                    source_mask=self._compute_source_mask(obs_list),
                )
            else:
                self._manager.mark_missed(track)

        # 5. 未关联观测 → 新目标
        for obs in unmatched:
            self._manager.create_track(
                obs.position, obs.velocity,
                self.config.dt, self.config.process_noise_q,
                obs.confidence,
                self._sensors[obs.sensor_id].bit_mask,
            )

        # 6. 清理
        self._manager.purge()

        return self._manager.tracks

    # ======================================================================
    # 内部方法
    # ======================================================================

    def _apply_transform(self, obs: Observation, source_frame: str):
        """空间对齐: 将观测变换到 target_frame。"""
        if not source_frame or self.config.transform_callback is None:
            return
        try:
            new_pos = self.config.transform_callback(
                source_frame,
                self.config.target_frame,
                obs.timestamp,
                obs.position,
            )
            obs.position = np.array(new_pos, dtype=np.float64)
        except Exception:
            pass

    def _find_track(self, track_id: int) -> Optional[Track]:
        for t in self._manager.tracks:
            if t.id == track_id:
                return t
        return None

    def _compute_source_mask(self, obs_list: List[Observation]) -> int:
        mask = 0
        for o in obs_list:
            if o.sensor_id in self._sensors:
                mask |= self._sensors[o.sensor_id].bit_mask
        return mask

    def _optimal_fusion(self, obs_list: List[Observation]
                        ) -> "Tuple[np.ndarray, np.ndarray]":
        """
        精度加权最优融合 (Theorem 5.1)。

        Returns:
            (z_fused [2,], R_fused [2,2])
        """
        n = len(obs_list)
        if n == 1:
            obs = obs_list[0]
            fn = self._sensors[obs.sensor_id].noise_model
            s2 = fn(obs)
            R = np.array([[s2, 0.0], [0.0, s2]], dtype=np.float64)
            return obs.position.copy(), R

        precisions = np.zeros(n)
        positions = np.zeros((n, 2))
        for i, obs in enumerate(obs_list):
            fn = self._sensors[obs.sensor_id].noise_model
            precisions[i] = 1.0 / max(fn(obs), 1e-9)
            positions[i] = obs.position

        total_prec = float(np.sum(precisions))
        if total_prec < 1e-9:
            z = np.mean(positions, axis=0)
            R = np.eye(2, dtype=np.float64)
        else:
            w = precisions / total_prec
            z = w @ positions
            s2 = 1.0 / total_prec
            R = np.array([[s2, 0.0], [0.0, s2]], dtype=np.float64)
        return z, R
