# ADAS Fusion — 多传感器融合自动紧急避障系统 设计说明书

> **平台**: NVIDIA Jetson ORIN NX (16GB) + Ubuntu 22.04 + ROS 2 Humble + JetPack 6.x  
> **硬件**: OAK-D PRO 深度相机 / RPLIDAR A1 激光雷达 / MS60-3015S80M4 毫米波雷达 / STM32F407 底盘控制器

---

## 目录

1. [系统总体设计](#1-系统总体设计)
2. [软件架构](#2-软件架构)
3. [数据流](#3-数据流)
4. [Package 结构与文件说明](#4-package-结构与文件说明)
5. [关键算法详解](#5-关键算法详解)
6. [硬件接口与连接](#6-硬件接口与连接)
7. [构建与运行](#7-构建与运行)
8. [配置参数说明](#8-配置参数说明)
9. [安全设计](#9-安全设计)

---

## 1. 系统总体设计

### 1.1 核心处理流程

```
感知 → 融合 → 状态估计 → 风险评估 (TTC) → 分级避障决策 → 底盘控制
```

### 1.2 硬件拓扑

```
┌─────────────────────────────────────────────────────────┐
│                   NVIDIA Jetson ORIN NX (Ubuntu 22.04)   │
│                                                          │
│  USB 3.0 ←── OAK-D PRO (RGB + Stereo Depth + YOLO)      │
│  USB 2.0 ←── RPLIDAR A1 (LaserScan)                     │
│  USB 2.0 ←── CH340 ←── MS60-3015S80M4 (mmWave Radar)    │
│  UART ←───── STM32F407VGT6 (Chassis Controller)          │
│                  ├── Motor MG513P30_12V (PWM)            │
│                  ├── Servo (PWM)                         │
│                  ├── Encoder (500线, Odometry)           │
│                  ├── IMU BMI088 (6-axis)                 │
│                  └── Battery / Ultrasonic                │
└─────────────────────────────────────────────────────────┘
```

### 1.3 设计原则

1. **不修改现有传感器驱动包** (`depthai-ros`, `rplidar_ros`, `mmwRadar_ros`)，所有新增功能放在独立节点中
2. **模块化**: 感知适配器 → 融合 → 决策 → 执行，各节点独立可替换
3. **自适应融合**: 传感器权重根据置信度、距离、环境动态调整
4. **安全优先**: 看门狗超时自动停车、最小安全距离强制制动

---

## 2. 软件架构

### 2.1 节点拓扑

```
┌──────────────────────────────────────────────────────────────┐
│                     SENSORS (不修改)                          │
│                                                               │
│  depthai-ros ──→ SpatialDetectionArray                         │
│  rplidar_ros ──→ LaserScan                                    │
│  mmw_radar    ──→ RadarTargetArray (polar)                    │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│                     ADAPTERS (新建)                           │
│                                                               │
│  detection_adapter ──→ /detections (Detection2DArray)         │
│  radar_adapter      ──→ /radar_objects (RadarObjectArray)     │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│                   FUSION NODE (新建)                          │
│                                                               │
│  时间同步 → 空间对齐(TF) → 目标生成 → 数据关联               │
│  → 自适应贝叶斯融合 → 假设检验 → Kalman Filter 跟踪          │
│                                                               │
│  输出: /tracked_objects (TrackedObjectArray)                   │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│                  DECISION NODE (新建)                         │
│                                                               │
│  TTC 计算 → 风险评估 → 分级决策 (SAFE/WARNING/SLOWDOWN/STOP) │
│                                                               │
│  输出: /cmd_vel (Twist)                                       │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│                 SERIAL BRIDGE (新建)                          │
│                                                               │
│  Twist → STM32 协议帧 (Header+ID+Len+Data+Checksum)          │
│  UART (115200 8N1) → STM32F407                               │
│                                                               │
│  接口: /dev/ttyTHS2 (Jetson USART3)                          │
└──────────────────────────────────────────────────────────────┘
```

### 2.2 ROS 2 Node 一览

| 节点名 | 包 | 功能 |
|--------|-----|------|
| `detection_adapter` | adas_fusion | depthai SpatialDetectionArray → Detection2DArray |
| `radar_adapter` | adas_fusion | mmWave polar → Cartesian (+ velocity) |
| `fusion_node` | adas_fusion | 多传感器自适应融合 + Kalman 跟踪 |
| `decision_node` | adas_fusion | TTC 分级避障决策 |
| `serial_bridge` | adas_fusion | /cmd_vel → STM32 UART 串口协议 |

---

## 3. 数据流

### 3.1 Topic 一览

| Topic | 消息类型 | 方向 | 说明 |
|-------|---------|------|------|
| `/oak/color/yolov4_Spatial_detections` | `SpatialDetectionArray` | 传感器→适配器 | OAK-D 内置 YOLO 输出 |
| `/detections` | `Detection2DArray` | 适配器→融合 | 统一视觉检测格式 |
| `/scan` | `sensor_msgs/LaserScan` | 传感器→融合 | LiDAR 扫描 |
| `/radar/targets` | `RadarTargetArray` | 传感器→适配器 | 雷达极坐标目标 |
| `/radar_objects` | `RadarObjectArray` | 适配器→融合 | 雷达笛卡尔目标 |
| `/tracked_objects` | `TrackedObjectArray` | 融合→决策 | 融合跟踪目标 |
| `/cmd_vel` | `geometry_msgs/Twist` | 决策→串口 | 速度指令 |
| TF | `tf2_msgs/TFMessage` | 系统→融合 | 坐标变换 |

### 3.2 自定义消息

**`Detection2D.msg`** — 视觉检测结果

```
uint16 x, y, width, height    # 2D bbox (像素坐标)
int16  class_id                # 类别ID
float32 confidence             # 置信度 [0,1]
float32 depth                  # 深度 (m)
geometry_msgs/Point position   # 3D位置 (相机系)
```

**`RadarObject.msg`** — 雷达目标 (笛卡尔)

```
geometry_msgs/Point position   # 位置 (m)
float32 vx, vy                 # 速度分量 (m/s)
float32 confidence             # 置信度 [0,1]
```

**`TrackedObject.msg`** — 融合跟踪目标

```
int32  id                      # 稳定跟踪ID
int16  class_id                # 类别ID
geometry_msgs/Point position   # 位置 (base_link)
float32 vx, vy                 # 速度分量 (m/s)
float32 confidence             # 综合置信度
uint8  source_flag             # 来源标志位 (bit0:视觉 bit1:LiDAR bit2:雷达)
```

### 3.3 坐标变换 (TF)

```
oak_parent_frame → oak → oak_rgb_camera_optical_frame  (depthai 自动发布)
base_link ← laser                                       (rplidar 配置: frame_id=laser)
base_link ← radar_link                                  (mmWave 配置: frame_id=radar_link)
```

**关键**: 需要手动配置 `oak_parent_frame` → `base_link` 和 `laser` → `base_link` 的静态变换，或通过 URDF 自动生成。

---

## 4. Package 结构与文件说明

### 4.1 目录树

```
ADAS_Fusion/
├── adas_fusion_msgs/                          # 自定义消息包 (CMake)
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── msg/
│       ├── Detection2D.msg                    # 视觉检测
│       ├── Detection2DArray.msg
│       ├── RadarObject.msg                    # 雷达目标 (笛卡尔)
│       ├── RadarObjectArray.msg
│       ├── TrackedObject.msg                  # 融合跟踪
│       └── TrackedObjectArray.msg
│
├── adas_fusion/                               # 融合 + 决策主包 (Python)
│   ├── package.xml
│   ├── setup.py                               # 入口点注册
│   ├── setup.cfg
│   ├── resource/adas_fusion                    # ament 标记文件
│   ├── adas_fusion/
│   │   ├── __init__.py
│   │   ├── detection_adapter.py               # 视觉适配器
│   │   ├── radar_adapter.py                   # 雷达适配器
│   │   ├── fusion_node.py                     # ★ 融合节点 (核心算法)
│   │   ├── decision_node.py                   # ★ 决策节点 (TTC避障)
│   │   └── serial_bridge.py                  # Jetson ↔ STM32 串口桥
│   ├── config/
│   │   └── fusion_params.yaml                 # 全局参数
│   └── launch/
│       ├── sensors.launch.py                  # 传感器驱动
│       ├── fusion_decision.launch.py          # 适配器+融合+决策+桥接
│       └── system.launch.py                   # 一键全系统启动
│
├── ADAS_Fusion_设计说明书.md                   # 本文档
└── Overview.md                                # 原始需求大纲
```

### 4.2 关键文件说明

| 文件 | 行数 | 职责 |
|------|------|------|
| `fusion_node.py` | ~550 | 全部融合算法: 时间/空间对齐、数据关联、贝叶斯融合、KF跟踪 |
| `decision_node.py` | ~280 | TTC计算、分级决策、绕行搜索 |
| `serial_bridge.py` | ~300 | STM32协议编码、UART收发、看门狗 |
| `detection_adapter.py` | ~100 | 消息格式转换 |
| `radar_adapter.py` | ~110 | 极坐标→笛卡尔转换 |

---

## 5. 关键算法详解

### 5.1 融合节点 (fusion_node.py)

#### 5.1.1 时间同步

```
维护一个滑动时间窗口 (默认 Δt=100ms) 的观测缓存。
当定时器触发融合循环时，取出窗口内所有传感器的最近观测。

数据到达时:
  cache.append(msg, timestamp=msg.header.stamp)

融合循环:
  window_start = now - time_window
  valid_obs = [obs for obs in cache if obs.stamp >= window_start]
```

#### 5.1.2 空间对齐 — TF 坐标变换

```
FOR EACH observation Z_i:
    source_frame = Z_i 所属传感器 frame (oak_rgb_camera_optical_frame / laser / radar_link)
    target_frame = "base_link"

    tf = lookup_transform(target_frame, source_frame, time=Z_i.stamp)

    Z_i^base = do_transform_point(Z_i, tf)

说明:
  - 相机内参转换 (2D像素→3D相机坐标) 由 OAK-D SDK 完成
  - 外参 TF 树需要用户根据实际安装位置配置
```

#### 5.1.3 目标生成 — LiDAR 聚类

```
算法: 欧氏距离聚类 (Euclidean Clustering)

输入: LaserScan (ranges[], angle_min, angle_increment)
输出: 聚类列表 [(cx, cy, size), ...]

步骤:
  1. 提取有效点:
     FOR i in range(len(ranges)):
         IF range_min < ranges[i] < range_max:
             angle = angle_min + i * angle_increment
             x = ranges[i] * cos(angle)
             y = ranges[i] * sin(angle)
             points.append((x, y))

  2. 欧氏聚类:
     对排序后的点遍历:
     IF 相邻点间距 < threshold (0.3m):
         归入当前聚类
     ELSE:
         开始新聚类

  3. 计算聚类中心:
     cluster_center = (mean(x), mean(y))
     cluster_confidence ∝ cluster_size / 30.0
```

#### 5.1.4 数据关联 — 马氏距离检验

```
算法: 最近邻关联 (Nearest Neighbor) + 马氏距离门限 (Mahalanobis Gate)

输入:
  T = {track_1, ..., track_N}  现有跟踪目标 (KF预测后)
  O = {obs_1, ..., obs_M}      当前观测

步骤 4.1 — 欧氏距离门限:
  FOR EACH track_k IN T:
      FOR EACH obs_j IN O:
          d_euclidean = ||track_k.position - obs_j.position||
          IF d_euclidean < gate_threshold (3.0m):
              candidates[k].append(obs_j)

步骤 4.2 — 马氏距离检验 (假设检验):
  FOR EACH track_k WITH candidates:
      FOR EACH obs_j IN candidates:
          z = obs_j.position            # 观测 [x, y]
          y = z - H * x_pred            # 创新 (Innovation)
          S = H * P_pred * H^T + R     # 创新协方差

          D_M^2 = y^T * S^{-1} * y

          IF D_M^2 < χ²_{0.95}(df=2) = 5.991:
              接受 H0: obs_j 属于 track_k
          ELSE:
              拒绝 H0: obs_j 为离群值/误检 → 剔除

      选择 D_M^2 最小的观测作为最佳匹配

  未匹配观测 → 初始化为候选目标
  未匹配跟踪 → miss_count++
```

#### 5.1.5 自适应贝叶斯融合 (核心创新)

```
贝叶斯模型:
  P(X | Z) ∝ P(Z | X) * P(X)

  其中:
    X = [px, py, vx, vy]^T  目标真实状态
    Z = {Z_cam, Z_lidar, Z_radar}  传感器观测集合
    P(Z|X) = 观测模型 (似然, 体现传感器可信度)
    P(X) = 先验分布 (来自 KF 预测)

步骤 5.1 — 传感器权重计算:

  视觉权重:
    w_cam = w_cam_base * confidence * exp(-d² / (2 * σ_cam²))
    说明: 距离增加 → 双目深度精度下降 → 权重降低

  LiDAR 权重:
    w_lidar = w_lidar_base * cluster_quality * exp(-d² / (2 * σ_lidar²))
    说明: 点云数越多 → 聚类越可靠 → 权重越高

  雷达权重:
    w_radar = w_radar_base * confidence * (1 + α * |v_radial| / 10)
    说明: 径向速度越大 → 动态目标识别优势 → 权重越高

  归一化:
    w_sum = w_cam + w_lidar + w_radar
    w_cam /= w_sum, w_lidar /= w_sum, w_radar /= w_sum

步骤 5.2 — 加权平均:
  X_fused = w_cam * Z_cam + w_lidar * Z_lidar + w_radar * Z_radar

步骤 5.3 — 先验-后验融合:
  α = clip(mean(confidences), 0.1, 0.9)
  X_posterior = α * X_fused + (1-α) * X_prior

  其中 X_prior 来自 KF 预测的位置分量
```

#### 5.1.6 Kalman Filter 状态估计

```
状态向量:
  x = [px, py, vx, vy]^T

运动模型 (Constant Velocity):
  x_{k+1} = F * x_k + w_k,   w_k ~ N(0, Q)

  状态转移矩阵:
    F = [[1, 0, dt, 0 ],
         [0, 1, 0,  dt],
         [0, 0, 1,  0 ],
         [0, 0, 0,  1 ]]

  过程噪声 (加速度作为白噪声):
    Q = q * G * G^T
    G = [[dt²/2, 0    ],
         [0,     dt²/2],
         [dt,    0    ],
         [0,     dt   ]]
    q = process_noise_q (默认 0.5)

观测模型:
  z = H * x + v_k,   v_k ~ N(0, R)

  H = [[1, 0, 0, 0],
       [0, 1, 0, 0]]     (仅观测位置)

  R = r * [[1, 0],
           [0, 1]]
  r = measurement_noise_r (默认 0.1)

预测步骤:
  x_pred = F @ x_prev
  P_pred = F @ P_prev @ F^T + Q

更新步骤 (使用融合后观测 Z_fused):
  y = Z_fused - H @ x_pred          (创新残差)
  S = H @ P_pred @ H^T + R         (创新协方差)
  K = P_pred @ H^T @ S^{-1}        (Kalman 增益)

  x_new = x_pred + K @ y
  P_new = (I - K @ H) @ P_pred
```

#### 5.1.7 目标生命周期管理

```
状态机: CANDIDATE → CONFIRMED → LOST → DELETED

  新目标初始化:
    IF 未关联观测连续 3 帧存在于相近位置:
        状态 → CONFIRMED, 分配全局唯一 ID

  目标丢失:
    IF miss_count > delete_threshold (5帧):
        从跟踪列表移除

  目标数上限:
    按置信度排序, 保留前 max_tracks (20) 个
```

---

### 5.2 决策节点 (decision_node.py)

#### 5.2.1 TTC 计算

```
对每个跟踪目标:

  d = ||target_pos - robot_pos||           # 相对距离
  u = (target_pos - robot_pos) / d         # 径向单位向量
  v_rel = (target_vel - robot_vel) · u     # 径向相对速度

  IF v_rel > 0.01:                         # 正值 = 正在接近
      TTC = d / v_rel
  ELSE:
      TTC = ∞                              # 远离, 无碰撞风险
```

#### 5.2.2 分级决策

```
min_TTC = min(TTC_i),  i = 1..N

┌──────────────────┬───────────────────────────────────────┐
│ TTC 范围          │ 行为                                    │
├──────────────────┼───────────────────────────────────────┤
│ TTC > 5.0 s      │ SAFE:     v = v_desired, ω = ω_desired │
│ 3.0 < TTC ≤ 5.0  │ WARNING:  v = v_desired * 0.7         │
│ 1.0 < TTC ≤ 3.0  │ SLOWDOWN: v = v_desired * 0.5 + 尝试绕行│
│ TTC ≤ 1.0        │ STOP:     v = 0.0, ω = 0.0            │
│ dist < 0.3 m     │ STOP:     无条件紧急停车               │
└──────────────────┴───────────────────────────────────────┘
```

#### 5.2.3 绕行方向搜索

```
算法: 扇区扫描法 (SLOWDOWN 阶段)

1. 前方 180° 按 15° 分辨率分成 12 个扇区
2. 对每个 5m 内障碍物, 标记其所在扇区为 "被占据"
3. 选择未被占据且最接近正前方的扇区作为绕行方向
4. 若所有扇区被占据 → 无法绕行, 保持直行减速
```

### 5.3 STM32 串口协议 (serial_bridge.py)

#### 5.3.1 协议帧格式

```
┌──────────┬────────┬──────────┬──────────┬──────────┐
│ Header   │ Cmd ID │ Data Len │ Data     │ Checksum │
│ 2 bytes  │ 1 byte │ 1 byte   │ N bytes  │ 1 byte   │
├──────────┼────────┼──────────┼──────────┼──────────┤
│ 0x55 0xAA│  ...   │   N      │   ...    │  XOR     │
└──────────┴────────┴──────────┴──────────┴──────────┘

校验和 = Header[0] ^ Header[1] ^ CmdID ^ DataLen ^ Data[0] ^ ... ^ Data[N-1]
```

#### 5.3.2 速度控制指令 (Cmd ID = 0x01)

```
Data[0:2] = v_linear  (int16 big-endian, 单位: mm/s,  范围: ±5000)
Data[2:4] = v_angular  (int16 big-endian, 单位: 0.001 rad/s, 范围: ±2000)

示例: v=0.2m/s, ω=0.1rad/s
  v_linear  = 200 (mm/s)    → 0x00 0xC8
  v_angular = 100 (0.001rad/s) → 0x00 0x64
  完整帧: 55 AA 01 04 00 C8 00 64 XX (XX=checksum)
```

#### 5.3.3 看门狗机制

```
每 50ms 发送一次速度帧 (20Hz)

IF 超过 watchdog_timeout (0.5s) 未收到 /cmd_vel:
    → 自动发送 v=0, ω=0 停车指令
END IF
```

---

## 6. 硬件接口与连接

### 6.1 传感器连接

| 传感器 | 接口 | Jetson 设备路径 | 默认参数 |
|--------|------|----------------|---------|
| OAK-D PRO | USB 3.0 | USB (depthai 自动识别) | camera_name=oak |
| RPLIDAR A1 | USB 2.0 | `/dev/ttyUSB0` | baud=115200 |
| MS60-3015S80M4 Radar | CH340 USB-UART | `/dev/ttyUSB1` | baud=921600 |

### 6.2 STM32 串口连接

| 项目 | 值 |
|------|-----|
| Jetson 引脚 | UART TX (Pin 8), UART RX (Pin 10) |
| 设备路径 | `/dev/ttyTHS2` (USART3) |
| 波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | None |
| 电平标准 | 3.3V TTL |

### 6.3 设备权限设置 (重要)

```bash
# 将用户 nvidia 加入 dialout 组以访问串口
sudo usermod -a -G dialout nvidia

# 设置 UART 权限 (持久化)
sudo chmod 666 /dev/ttyTHS2

# 或者通过 udev 规则自动设置
echo 'KERNEL=="ttyTHS*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-jetson-uart.rules
sudo udevadm control --reload-rules
```

---

## 7. 构建与运行

### 7.1 依赖安装

```bash
# 系统依赖 (Jetson ORIN NX, Ubuntu 22.04)
sudo apt update
sudo apt install -y python3-pip python3-numpy python3-serial
pip3 install pyserial numpy

# ROS 2 依赖 (如果还没安装)
# depthai-ros, rplidar_ros, mmw_radar_driver 需要先构建
```

### 7.2 构建

```bash
cd ~/ADAS_Fusion

# Step 1: 构建自定义消息包
colcon build --symlink-install --packages-select adas_fusion_msgs
source install/setup.bash

# Step 2: 构建融合决策包
colcon build --symlink-install --packages-select adas_fusion
source install/setup.bash

# 可选: 一次性构建
colcon build --symlink-install
```

### 7.3 运行

```bash
source install/setup.bash

# === 方式 1: 一键启动全系统 ===
ros2 launch adas_fusion system.launch.py

# === 方式 2: 仅启动传感器 ===
ros2 launch adas_fusion sensors.launch.py

# === 方式 3: 仅启动融合+决策 (传感器已运行) ===
ros2 launch adas_fusion fusion_decision.launch.py

# === 带参数启动 ===
ros2 launch adas_fusion system.launch.py \
    max_linear_vel:=0.5 \
    ttc_emergency:=0.8 \
    serial_port:=/dev/ttyTHS2 \
    enable_radar:=false \
    camera_name:=oak \
    nn_type:=spatial
```

### 7.4 验证

```bash
# 查看活跃话题
ros2 topic list

# 查看融合跟踪输出
ros2 topic echo /tracked_objects

# 查看决策输出
ros2 topic echo /cmd_vel

# 查看 TF 树
ros2 run tf2_tools view_frames
```

---

## 8. 配置参数说明

### 8.1 融合参数 (`fusion_params.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `time_window` | 0.1 s | 时间同步窗口 |
| `association_gate` | 3.0 m | 欧氏距离关联门限 |
| `mahalanobis_threshold` | 5.991 | 马氏距离 χ² 阈值 (95%置信) |
| `confirm_threshold` | 3 帧 | 确认新目标所需连续关联帧数 |
| `delete_threshold` | 5 帧 | 删除丢失目标所需连续丢失帧数 |
| `max_tracks` | 20 | 最大跟踪目标数 |
| `dt` | 0.1 s | Kalman Filter 时间步长 |
| `process_noise_q` | 0.5 | 过程噪声系数 (加速度方差) |
| `measurement_noise_r` | 0.1 | 观测噪声协方差 |
| `camera_weight` | 0.35 | 视觉初始权重 |
| `lidar_weight` | 0.35 | LiDAR 初始权重 |
| `radar_weight` | 0.30 | 雷达初始权重 |

### 8.2 决策参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ttc_warning` | 5.0 s | 预警 TTC 阈值 |
| `ttc_slowdown` | 3.0 s | 减速 TTC 阈值 |
| `ttc_emergency` | 1.0 s | 紧急停车 TTC 阈值 |
| `max_linear_vel` | 0.3 m/s | 最大线速度 |
| `slowdown_factor` | 0.5 | 减速因子 |
| `max_angular_vel` | 0.5 rad/s | 最大角速度 |
| `min_safe_distance` | 0.3 m | 最小安全距离 |

### 8.3 串口桥接参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyTHS2` | Jetson UART 设备 |
| `baud_rate` | 115200 | 波特率 |
| `watchdog_timeout` | 0.5 s | cmd_vel 超时自动停车 |
| `wheel_base` | 0.2 m | 轮距 (用于里程计解算) |
| `ticks_per_meter` | 2000 | 编码器每米脉冲数 |

---

## 9. 安全设计

### 9.1 多层安全防护

```
Layer 1: min_safe_distance (0.3m) → 无条件急停
Layer 2: TTC ≤ 1.0s → 紧急停车
Layer 3: TTC ≤ 3.0s → 大幅减速 + 尝试绕行
Layer 4: TTC ≤ 5.0s → 轻度减速预警
Layer 5: 看门狗超时 (0.5s 无 cmd_vel) → 串口桥自动发停车指令
Layer 6: 串口断连 → 自动重连 + 停车保护
```

### 9.2 已知局限与待改进

1. **TF 外参**: 传感器间的静态 TF 需根据实际安装位置手动配置
2. **LiDAR 聚类**: 使用简单欧氏聚类, 复杂场景可用 DBSCAN
3. **绕行策略**: 当前仅做方向选择, 完整路径规划应集成 Nav2
4. **传感器故障检测**: 当前通过置信度降低权重, 可加入显式故障检测
5. **速度平滑**: 可加入加速度限制 (ramp) 避免急加速

---

> **文档版本**: v1.0  
> **最后更新**: 2026-04-28  
> **作者**: Moneve  
> **目标平台**: NVIDIA Jetson ORIN NX + Ubuntu 22.04 + ROS 2 Humble

---

### 附录 A: 代码中各算法的伪代码入口

| 算法 | 文件 | 方法/位置 |
|------|------|----------|
| 时间同步 | `fusion_node.py` | `_fusion_cycle()` → `_collect()` |
| 空间对齐 | `fusion_node.py` | `_collect()` — TF lookup |
| LiDAR 聚类 | `fusion_node.py` | `_scan_to_points()` + `_euclidean_clustering()` |
| 数据关联 | `fusion_node.py` | `_associate()` |
| 马氏检验 | `fusion_node.py` | `Track.mahalanobis_distance_sq()` |
| 自适应融合 | `fusion_node.py` | `_adaptive_fusion()` |
| Kalman Filter | `fusion_node.py` | `Track.predict()` + `Track.update()` |
| TTC 计算 | `decision_node.py` | `_decision_cycle()` |
| 绕行搜索 | `decision_node.py` | `_find_safe_direction()` |
| STM32 协议 | `serial_bridge.py` | `Stm32Protocol.encode_velocity()` |

### 附录 B: 依赖关系图

```
adas_fusion_msgs ──→ adas_fusion
depthai_ros_msgs ──→ adas_fusion
mmw_radar_msgs ────→ adas_fusion
tf2_ros ───────────→ adas_fusion
pyserial ──────────→ adas_fusion (serial_bridge)
numpy ─────────────→ adas_fusion (fusion_node)
```
