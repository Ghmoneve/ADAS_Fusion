"""
kalman_filter.py — 恒定速度 (CV) 模型 Kalman Filter (DWNA 过程噪声)

Algorithm.md §5.6:
  状态: x = [px, py, vx, vy]^T
  Q = q * [[dt⁴/4, 0,  dt³/2, 0  ],
           [0,      dt⁴/4, 0,  dt³/2],
           [dt³/2, 0,      dt², 0  ],
           [0,      dt³/2, 0,  dt² ]]
  H = [[1,0,0,0],[0,1,0,0]]
"""

import numpy as np


class KalmanFilter:
    """恒定速度模型 Kalman Filter (DWNA)。

    用法:
        kf = KalmanFilter(dt=0.1, process_noise_q=0.5)
        kf.init_state(px=1.0, py=2.0, vx=0.0, vy=0.0)
        kf.predict()
        kf.update(z=np.array([1.1, 2.0]), R=np.eye(2)*0.01)
    """

    __slots__ = ('dt', 'q', 'F', 'H', 'Q', 'x', 'P')

    def __init__(self, dt: float = 0.1, process_noise_q: float = 0.5):
        self.dt = dt
        self.q = process_noise_q

        # 状态转移矩阵
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float64)

        # 观测矩阵 (仅位置)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float64)

        # 过程噪声 (DWNA)
        dt2 = dt * dt
        dt3 = dt2 * dt / 2.0
        dt4 = dt2 * dt2 / 4.0
        self.Q = self.q * np.array([
            [dt4, 0.0, dt3, 0.0],
            [0.0, dt4, 0.0, dt3],
            [dt3, 0.0, dt2, 0.0],
            [0.0, dt3, 0.0, dt2],
        ], dtype=np.float64)

        self.x = np.zeros(4, dtype=np.float64)
        self.P = np.eye(4, dtype=np.float64) * 10.0

    def init_state(self, px: float, py: float,
                   vx: float = 0.0, vy: float = 0.0,
                   P_diag: float = 10.0):
        """初始化状态向量和协方差。"""
        self.x = np.array([px, py, vx, vy], dtype=np.float64)
        self.P = np.eye(4, dtype=np.float64) * P_diag

    def predict(self):
        """预测步: x = F*x, P = F*P*F^T + Q。"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z: np.ndarray, R: np.ndarray):
        """
        更新步。

        Args:
            z: 观测 [x, y]
            R: 观测噪声协方差 [2, 2]
        """
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        K = self.P @ self.H.T @ S_inv
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    def mahalanobis_sq(self, z: np.ndarray, sigma2: float) -> float:
        """
        马氏距离平方 (§5.4.4)。

        Args:
            z:      观测 [x, y]
            sigma2: 观测噪声方差 σ²

        Returns:
            D_M² = (z - Hx)^T (H P H^T + σ²I)^{-1} (z - Hx)
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

    @property
    def position_covariance(self) -> np.ndarray:
        return self.P[:2, :2]
