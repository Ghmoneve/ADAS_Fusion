"""
fusion_core — 传感器无关的多传感器最优融合引擎

核心 API:
    from fusion_core import FusionEngine, FusionConfig, SensorPresets

    engine = FusionEngine()
    engine.register_sensor("lidar", type="lidar", sigma_0=0.03)
    engine.add_observation("lidar", position=[1, 2], confidence=0.9)
    tracks = engine.step()

参考:
    README.md — 完整集成指南与 API 文档
"""

from .fusion_engine import FusionEngine
from .types import (FusionConfig, Observation, SensorConfig,
                    Track, NoiseModel)
from .kalman_filter import KalmanFilter
from .sensor_model import SensorPresets
from .association import associate
from .track_manager import TrackManager

__version__ = "1.0.0"
__all__ = [
    "FusionEngine",
    "FusionConfig",
    "Observation",
    "SensorConfig",
    "Track",
    "NoiseModel",
    "KalmanFilter",
    "SensorPresets",
    "associate",
    "TrackManager",
]
