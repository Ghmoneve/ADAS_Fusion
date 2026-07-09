"""
track_manager.py — 目标生命周期管理 (§5.1.7)

状态机: CANDIDATE → CONFIRMED → DELETED
"""

from __future__ import annotations
from typing import List
from .types import Track
from .kalman_filter import KalmanFilter


class TrackManager:
    """管理跟踪目标的创建、确认与删除。"""

    def __init__(self, confirm_threshold: int = 3,
                 delete_threshold: int = 5,
                 max_tracks: int = 20):
        self.confirm_threshold = confirm_threshold
        self.delete_threshold = delete_threshold
        self.max_tracks = max_tracks
        self._tracks: List[Track] = []
        self._next_id: int = 0

    def __len__(self):
        return len(self._tracks)

    def __iter__(self):
        return iter(self._tracks)

    @property
    def tracks(self) -> List[Track]:
        return self._tracks

    @property
    def confirmed_tracks(self) -> List[Track]:
        return [t for t in self._tracks if t.status == 'CONFIRMED']

    @property
    def active_tracks(self) -> List[Track]:
        return [t for t in self._tracks
                if t.status in ('CANDIDATE', 'CONFIRMED')]

    def _alloc_id(self) -> int:
        self._next_id += 1
        return self._next_id

    def create_track(self, pos: "np.ndarray", vel: "np.ndarray",
                     dt: float, q: float, confidence: float,
                     source_mask: int) -> Track:
        """创建候选跟踪目标。"""
        kf = KalmanFilter(dt=dt, process_noise_q=q)
        kf.init_state(pos[0], pos[1], vel[0], vel[1])
        self._next_id += 1
        track = Track(
            id=self._next_id,
            state=kf.x.copy(),
            covariance=kf.P.copy(),
            confidence=confidence,
            source_mask=source_mask,
            hit_count=1,
            status='CANDIDATE',
        )
        track._kf = kf
        self._tracks.append(track)
        return track

    def predict_all(self):
        """对所有活跃目标执行 KF 预测。"""
        for t in self._tracks:
            kf = getattr(t, '_kf', None)
            if kf is not None:
                kf.predict()
                t.state = kf.x.copy()
                t.covariance = kf.P.copy()

    def update_track(self, track: Track, fused_z: "np.ndarray",
                     R_fused: "np.ndarray", confidence: float,
                     source_mask: int):
        """更新已关联目标。"""
        kf = getattr(track, '_kf', None)
        if kf is not None:
            kf.update(fused_z, R_fused)
            track.state = kf.x.copy()
            track.covariance = kf.P.copy()
        track.hit_count += 1
        track.miss_count = 0
        track.confidence = max(track.confidence, confidence)
        track.source_mask |= source_mask
        if track.status == 'CANDIDATE' and track.hit_count >= self.confirm_threshold:
            track.status = 'CONFIRMED'

    def mark_missed(self, track: Track):
        """标记未关联目标。"""
        track.miss_count += 1

    def purge(self):
        """删除超时目标, 限制最大数量。"""
        self._tracks = [t for t in self._tracks
                        if t.miss_count <= self.delete_threshold]
        if len(self._tracks) > self.max_tracks:
            self._tracks.sort(key=lambda t: t.confidence, reverse=True)
            self._tracks = self._tracks[:self.max_tracks]
