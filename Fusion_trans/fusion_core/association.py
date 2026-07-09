"""
association.py — 数据关联 (Algorithm.md §5.4)

算法: 欧氏距离门限 + 马氏距离检验 + 最近邻匹配

输入: 跟踪目标列表 + 观测列表
输出: (track→obs 匹配字典, 未匹配观测列表)
"""

from __future__ import annotations
from typing import Dict, List, Set, Tuple
import numpy as np
from .types import Observation, Track


def associate(
    tracks: List[Track],
    observations: List[Observation],
    association_gate: float,
    chi2_threshold: float,
    noise_model_map: Dict[str, "NoiseModel"],
) -> Tuple[Dict[int, List[Observation]], List[Observation]]:
    """
    将观测关联到现有跟踪目标。

    步骤:
      1. 对每个 track, 计算与所有 obs 的欧氏距离
      2. 距离 < association_gate 的为候选
      3. 候选 obs 计算马氏距离 → 卡方检验 → 最近邻选择
      4. 返回 (matched, unmatched)

    Args:
        tracks:           当前跟踪目标 (需已 predict)
        observations:     当前帧观测
        association_gate: 欧氏距离门限 (m)
        chi2_threshold:   马氏距离卡方阈值
        noise_model_map:  {sensor_id: noise_model} 映射

    Returns:
        (matched, unmatched)
          matched:   {track.id: [obs, ...]}
          unmatched: 未关联的观测列表
    """
    if not observations:
        return {}, []

    matched: Dict[int, List[Observation]] = {}
    used_obs: Set[int] = set()

    for track in tracks:
        candidates: List[Tuple[int, Observation, float]] = []

        for i, obs in enumerate(observations):
            if i in used_obs:
                continue
            dist = float(np.linalg.norm(track.position - obs.position))
            if dist >= association_gate:
                continue

            # 获取传感器噪声模型 → 计算 sigma² → 马氏检验
            noise_fn = noise_model_map.get(obs.sensor_id)
            if noise_fn is None:
                continue
            sigma2 = noise_fn(obs)

            # 使用 Track 的 KalmanFilter 计算马氏距离
            kf = getattr(track, '_kf', None)
            if kf is None:
                continue
            dm2 = kf.mahalanobis_sq(obs.position, sigma2)
            if dm2 < chi2_threshold:
                candidates.append((i, obs, dm2))

        if not candidates:
            continue

        candidates.sort(key=lambda x: x[2])
        best_idx, best_obs, _ = candidates[0]

        matched.setdefault(track.id, []).append(best_obs)
        used_obs.add(best_idx)

    unmatched = [obs for i, obs in enumerate(observations) if i not in used_obs]
    return matched, unmatched
