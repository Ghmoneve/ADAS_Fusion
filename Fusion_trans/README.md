# Fusion_trans — 传感器无关的多传感器最优融合引擎

> **版本**: v1.0.0 | **依赖**: Python 3.8+, NumPy | **ROS 依赖**: 无 (可选, 仅示例需要)

---

## 目录

1. [概述](#1-概述)
2. [架构](#2-架构)
3. [快速开始](#3-快速开始)
4. [核心 API 参考](#4-核心-api-参考)
5. [传感器注册指南](#5-传感器注册指南)
6. [噪声模型推导](#6-噪声模型推导)
7. [集成示例](#7-集成示例)
8. [参数调优](#8-参数调优)
9. [文件清单](#9-文件清单)

---

## 1. 概述

**Fusion_trans** 是一个传感器无关的多传感器最优融合引擎。它将不同传感器 (相机/LiDAR/雷达/超声波/...) 的异步观测融合为统一的跟踪目标列表，输出平滑的位置、速度和协方差估计。

### 核心特性

- **传感器无关**: 通过 `register_sensor()` 注册任意类型、任意数量的传感器
- **最优融合**: 精度加权 BLUE (Best Linear Unbiased Estimator)，达到 Cramér-Rao 下界
- **自适应噪声**: 每个传感器有独立的噪声模型 `f(obs) → σ²`，支持预置模型和自定义回调
- **零 ROS 依赖**: `fusion_core/` 仅依赖 NumPy，可在任意 Python 环境运行
- **工业级 API**: 完整类型标注、dataclass 数据结构、健壮的错误处理

### 融合流水线

```
register_sensor()  →  [LiDAR]  [Camera]  [Radar]  [Custom]
                           ↓        ↓        ↓        ↓
add_observation()    →  观测缓存 (100ms 时间窗口)
                           ↓
step()               →  [predict] → [associate] → [fuse] → [update] → [publish]
                           ↓
                       List[Track]
```

---

## 2. 架构

```
Fusion_trans/
├── fusion_core/                 # 核心引擎 (零 ROS 依赖)
│   ├── __init__.py              # 公开 API 导出
│   ├── types.py                 # Observation, Track, SensorConfig, FusionConfig
│   ├── kalman_filter.py         # DWNA 恒定速度 Kalman Filter
│   ├── sensor_model.py          # 传感器噪声模型工厂
│   ├── association.py           # 马氏距离关联 + 最近邻匹配
│   ├── track_manager.py         # 目标生命周期管理
│   └── fusion_engine.py         # 主编排器 (传感器注册/观测输入/融合步进)
├── msg/
│   ├── TrackedObject.msg        # 融合跟踪目标消息
│   └── TrackedObjectArray.msg
├── example/
│   └── example_fusion.py        # 最小可运行示例
└── README.md                    # 本文档
```

### 模块依赖关系

```
types.py  ←  (所有模块的基础)
    ↑
kalman_filter.py  ←  sensor_model.py  ←  association.py
    ↑                    ↑                  ↑
    └────────────────────┼──────────────────┘
                         ↑
                  fusion_engine.py  ←  track_manager.py
```

### 无 ROS 依赖验证

```bash
# fusion_core 可在任何 Python 3.8+ 环境直接导入
python3 -c "from fusion_core import FusionEngine, FusionConfig; print('OK')"
```

---

## 3. 快速开始

### 3.1 纯 Python (无 ROS)

```python
from fusion_core import FusionEngine, FusionConfig

# 1. 创建引擎
config = FusionConfig(dt=0.1, process_noise_q=0.5)
engine = FusionEngine(config)

# 2. 注册传感器
engine.register_sensor("lidar", type="lidar", sigma_0=0.03, n_ref=30)

# 3. 喂入观测
engine.add_observation(
    "lidar",
    position=[1.0, 2.0],
    velocity=[0.0, 0.0],
    confidence=0.9,
    cluster_size=20,
    timestamp=None,  # None = 使用 time.time()
)

# 4. 步进 (每 100ms 调用一次)
tracks = engine.step()
for t in tracks:
    print(f"Track #{t.id}: pos={t.position}, vel={t.velocity}, status={t.status}")
```

### 3.2 ROS 2 集成

```python
from fusion_core import FusionEngine, FusionConfig

class MyFusionNode(Node):
    def __init__(self):
        super().__init__('my_fusion_node')
        self.engine = FusionEngine()
        self.engine.register_sensor("camera", type="camera", sigma_0=0.05)
        self.engine.register_sensor("lidar",  type="lidar",  sigma_0=0.03)
        self.engine.register_sensor("radar",  type="radar",  sigma_0=0.2)

        self.cam_sub  = self.create_subscription(..., self.cam_cb, 10)
        self.lidar_sub = self.create_subscription(..., self.lidar_cb, 10)
        self.radar_sub = self.create_subscription(..., self.radar_cb, 10)
        self.timer = self.create_timer(0.1, self.fusion_cycle)

    def cam_cb(self, msg):
        self.engine.add_observation("camera", position=[msg.x, msg.y],
                                     confidence=msg.confidence, dist=msg.depth)

    def fusion_cycle(self):
        tracks = self.engine.step()
        # 发布 tracks...
```

---

## 4. 核心 API 参考

### 4.1 FusionEngine

```python
class FusionEngine:
    def __init__(self, config: Optional[FusionConfig] = None)

    # 传感器管理
    def register_sensor(self, sensor_id: str,
                        noise_model: Optional[NoiseModel] = None,
                        frame_id: str = "",
                        **preset_kwargs) -> SensorConfig

    def unregister_sensor(self, sensor_id: str)

    @property
    def sensors(self) -> Dict[str, SensorConfig]

    # 观测输入
    def add_observation(self, sensor_id: str,
                        position: array-like,
                        velocity: array-like = (0.0, 0.0),
                        confidence: float = 0.5,
                        timestamp: Optional[float] = None,
                        **metadata) -> Optional[Observation]

    # 融合步进 (每 dt 秒调用一次)
    def step(self) -> List[Track]
```

### 4.2 数据结构

```python
@dataclass
class Observation:
    position:   np.ndarray    # [x, y] 目标位置 (m)
    velocity:   np.ndarray    # [vx, vy] 目标速度 (m/s)
    confidence: float         # [0, 1]
    sensor_id:  str           # 传感器标识符
    metadata:   Dict          # 传感器特定元数据
    timestamp:  float         # Unix 秒

@dataclass
class Track:
    id:          int          # 全局唯一跟踪 ID
    state:       np.ndarray   # [px, py, vx, vy]
    covariance:  np.ndarray   # [4, 4]
    confidence:  float        # [0, 1]
    source_mask: int          # 来源传感器位掩码
    hit_count:   int          # 累计命中帧数
    miss_count:  int          # 连续丢失帧数
    status:      str          # "CANDIDATE" | "CONFIRMED"

@dataclass
class FusionConfig:
    dt:                  float = 0.1     # KF 时间步长 (s)
    process_noise_q:     float = 0.5     # 过程噪声系数
    association_gate:    float = 3.0     # 欧氏距离门限 (m)
    chi2_threshold:      float = 5.991   # 马氏距离 χ²(df=2, 95%)
    time_window:         float = 0.1     # 时间同步窗口 (s)
    confirm_threshold:   int   = 3       # 确认帧数
    delete_threshold:    int   = 5       # 删除帧数
    max_tracks:          int   = 20      # 最大目标数
    target_frame:        str   = "base_link"
    transform_callback:  Optional[Callable] = None
```

### 4.3 噪声模型接口

```python
# NoiseModel = Callable[[Observation], float]
# 输入: Observation → 输出: sigma² (测量方差)

# 预置模型
SensorPresets.camera(sigma_0=0.05, sigma_c=5.0)    # → NoiseModel
SensorPresets.lidar(sigma_0=0.03, n_ref=30.0)       # → NoiseModel
SensorPresets.radar(sigma_0=0.2, alpha=0.5, v_ref=10.0)  # → NoiseModel
SensorPresets.constant(sigma2=0.01)                  # → NoiseModel
SensorPresets.custom(lambda obs: 0.01 / obs.confidence)  # → NoiseModel
```

---

## 5. 传感器注册指南

### 5.1 使用预置模型 (推荐)

| 传感器类型 | 预置模型 | 所需 metadata |
|----------|---------|-------------|
| 相机 (双目深度) | `type="camera"` | `dist` (目标距离 m) |
| LiDAR (点云聚类) | `type="lidar"` | `cluster_size` (聚类点数) |
| 毫米波雷达 | `type="radar"` | `v_radial` (径向速度 m/s) |
| 通用 (常量噪声) | `type="constant"` | 无 |

```python
# LiDAR
engine.register_sensor("lidar_main", type="lidar",
                       sigma_0=0.03, n_ref=30, frame_id="laser")

# Camera
engine.register_sensor("camera_front", type="camera",
                       sigma_0=0.05, sigma_c=5.0,
                       frame_id="camera_optical_frame")

# Radar
engine.register_sensor("radar_front", type="radar",
                       sigma_0=0.2, alpha=0.5, v_ref=10.0,
                       frame_id="radar_link")
```

### 5.2 自定义噪声模型

```python
# 基于置信度的简单模型
def my_noise(obs):
    return 0.01 / max(obs.confidence, 0.01)

engine.register_sensor("my_sensor", noise_model=my_noise)

# Lambda 形式
engine.register_sensor("ultrasonic",
    noise_model=lambda obs: 0.05 / obs.confidence)
```

### 5.3 可插拔性: 接入新传感器

```python
# 步骤 1: 注册新传感器
engine.register_sensor("thermal_camera",
    noise_model=lambda obs: 0.02 / max(obs.confidence, 0.01),
    frame_id="thermal_frame")

# 步骤 2: 在传感器回调中喂入观测
def on_thermal_detection(msg):
    for det in msg.detections:
        engine.add_observation(
            "thermal_camera",
            position=[det.x, det.y],
            velocity=[0, 0],
            confidence=det.confidence,
            temperature=det.temp,    # 自定义 metadata
        )

# 步骤 3: 周期性调用 step()
tracks = engine.step()
```

---

## 6. 噪声模型推导

### 6.1 理论背景

多传感器融合问题中，每个传感器 $s$ 的观测 $\mathbf{z}_s$ 满足:

$\mathbf{z}_s = \mathbf{x}_{\text{true}} + \mathbf{v}_s, \quad \mathbf{v}_s \sim \mathcal{N}(\mathbf{0}, \sigma_s^2 \mathbf{I}_2)$

**Theorem 5.1 (最小方差线性无偏融合)**: 对于 $m$ 个独立观测，最优融合权重为:

$w_i = \frac{\lambda_i}{\sum_j \lambda_j}, \quad \lambda_i = \frac{1}{\sigma_i^2}$

融合后方差: $\sigma_{\text{fused}}^2 = \frac{1}{\sum_i \lambda_i}$

### 6.2 相机噪声模型

双目深度误差随距离平方增长:

$\sigma_{\text{cam}}^2 = \frac{\sigma_{c,0}^2}{\text{conf}} \cdot \exp\left(\frac{d^2}{2\sigma_c^2}\right)$

| 参数 | 默认值 | 物理含义 |
|------|-------|---------|
| σ_c,0 | 0.05 m | 1m 距离、置信度 1 时的基准标准差 |
| σ_c | 5.0 m | 距离衰减尺度 |

### 6.3 LiDAR 噪声模型

聚类中心精度与点数平方根成反比 (中心极限定理):

$\sigma_{\text{lidar}}^2 = \sigma_{l,0}^2 \cdot \frac{N_{\text{ref}}}{|C|}$

| 参数 | 默认值 | 物理含义 |
|------|-------|---------|
| σ_l,0 | 0.03 m | N_ref 点时的基准标准差 |
| N_ref | 30 | 参考聚类点数 |

### 6.4 雷达噪声模型

多普勒效应使动态目标定位更准:

$\sigma_{\text{radar}}^2 = \frac{\sigma_{r,0}^2}{\text{conf} \cdot \left(1 + \alpha \cdot \frac{|v_{\text{radial}}|}{v_0}\right)}$

| 参数 | 默认值 | 物理含义 |
|------|-------|---------|
| σ_r,0 | 0.2 m | 静态目标基准标准差 |
| α | 0.5 | 速度增益因子 |
| v₀ | 10 m/s | 参考速度 |

### 6.5 参数标定建议

1. 将传感器对准已知位置的静止目标
2. 采集 N=100+ 帧测量数据
3. 计算测量标准差 σ_measured = std(z - z_truth)
4. 令模型中 conf=1, dist 取实际值, 反解 σ_0

---

## 7. 集成示例

### 7.1 验证融合引擎是否正确工作

```python
from fusion_core import FusionEngine, FusionConfig

config = FusionConfig(dt=0.1)
engine = FusionEngine(config)
engine.register_sensor("test_sensor", type="constant", sigma2=0.01)

# 模拟目标沿 x 轴匀速运动: x(t) = 0.5 * t
for i in range(100):
    t = i * 0.1
    true_x = 0.5 * t
    # 添加噪声观测
    noisy_x = true_x + np.random.normal(0, 0.1)

    engine.add_observation("test_sensor",
        position=[noisy_x, 0.0], velocity=[0, 0],
        confidence=0.9, timestamp=t)

    tracks = engine.step()
    if tracks:
        t0 = tracks[0]
        # 预期: position ≈ true_x, velocity ≈ 0.5
        print(f"t={t:.1f} true_x={true_x:.2f} "
              f"est_x={t0.position[0]:.2f} est_vx={t0.velocity[0]:.2f}")
```

### 7.2 ROS 2 全集成示例

参见 `example/example_fusion.py`。

---

## 8. 参数调优

| 场景 | 参数调整建议 |
|------|------------|
| 高速目标 (v > 2 m/s) | 增大 `process_noise_q` (0.5 → 1.0) |
| 密集目标场景 | 减小 `association_gate` (3.0 → 1.5) |
| 传感器噪声大 | 增大各传感器 `sigma_0` |
| 误检率高 | 增大 `chi2_threshold` (5.991 → 7.378, df=2 99%) |
| 短暂遮挡多 | 增大 `delete_threshold` (5 → 10) |
| CPU 资源紧张 | 增大 `dt` (0.1 → 0.2), 减小 `max_tracks` (20 → 10) |

---

## 9. 文件清单

| 文件 | 行数 | 职责 |
|------|-----|------|
| `fusion_core/types.py` | 110 | 数据结构: Observation, Track, SensorConfig, FusionConfig |
| `fusion_core/kalman_filter.py` | 105 | DWNA Kalman Filter: predict / update / mahalanobis |
| `fusion_core/sensor_model.py` | 95 | 噪声模型工厂: Camera / LiDAR / Radar / Custom |
| `fusion_core/association.py` | 75 | 数据关联: 欧氏门限 + 马氏检验 + 最近邻 |
| `fusion_core/track_manager.py` | 75 | 目标管理: CANDIDATE→CONFIRMED→DELETED |
| `fusion_core/fusion_engine.py` | 235 | 主编排器: 注册 / 观测输入 / 融合步进 |
| `fusion_core/__init__.py` | 25 | 公开 API 导出 |
| `example/example_fusion.py` | 85 | 最小可运行示例 |
| `msg/TrackedObject.msg` | 10 | 跟踪目标 ROS 消息 |

**总代码量**: ~800 行 (不含注释), 零 ROS 依赖核心 ~700 行。

### 依赖

```
# 核心 (必须)
pip install numpy

# ROS 2 集成 (可选)
sudo apt install ros-${ROS_DISTRO}-tf2-ros
```

---

> **集成方须知**: 本模块仅提供融合引擎 API。真实传感器数据由集成方通过 `add_observation()` 喂入，通过 `register_sensor()` 注册噪声模型。空间对齐 (TF) 通过 `FusionConfig.transform_callback` 注入。
