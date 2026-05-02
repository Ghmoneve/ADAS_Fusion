# ADAS Fusion — 多传感器融合自动紧急避障系统 设计说明书

> **平台**: NVIDIA Jetson ORIN NX (16GB) + Ubuntu 22.04 + ROS 2 Humble + JetPack 6.x
> **硬件**: OAK-D PRO 深度相机 / RPLIDAR A1 激光雷达 / MS60-3015S80M4 毫米波雷达 / STM32F407 底盘控制器
> **版本**: v2.0 | **作者**: Moneve | **日期**: 2026-05-02
> **变更**: 采用最优融合理论 (BLUE), 传感器方差噪声模型, 手柄控制 + 紧急接管架构

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
│  USB 2.0 ←── Gamepad/Joystick (手柄控制)                  │
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
│  joy_node     ──→ /joy (sensor_msgs/Joy)                     │
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
│                   FUSION NODE (核心算法)                      │
│                                                               │
│  时间同步 → 空间对齐(TF) → 目标生成 → 数据关联               │
│  → 自适应贝叶斯融合 → 假设检验 → Kalman Filter 跟踪          │
│                                                               │
│  输出: /tracked_objects (TrackedObjectArray)                   │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│  joy_node ──→ /joy ─────────────────────────────────┐         │
│                  DECISION NODE                       │         │
│                                               │               │
│  手柄直通 ←─── SAFE/WARNING ─── TTC 计算 ←── 跟踪目标 │         │
│      │                                          │               │
│  紧急接管 ←─── SLOWDOWN/EMERGENCY ─── 避障 cmd_vel    │         │
│      │                                          │               │
│  (3s 冷却后归还手柄)                                    │         │
│                                                               │
│  输出: /cmd_vel (Twist)                                       │
└──────────────────────┬───────────────────────────────────────┘
                       │
┌──────────────────────▼───────────────────────────────────────┐
│                 SERIAL BRIDGE                                 │
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
| `radar_adapter` | adas_fusion | mmWave 极坐标 → 笛卡尔坐标 + 速度分解 |
| `fusion_node` | adas_fusion | 多传感器自适应融合 + Kalman 跟踪 |
| `decision_node` | adas_fusion | 手柄控制 + TTC 分级避障决策 + 紧急接管 + 绕行搜索 |
| `serial_bridge` | adas_fusion | /cmd_vel → STM32 UART 串口协议 |

---

## 3. 数据流

### 3.1 Topic 一览

| Topic | 消息类型 | 方向 | 说明 |
|-------|---------|------|------|
| `/oak/color/yolov4_Spatial_detections` | `SpatialDetectionArray` | 传感器→适配器 | OAK-D 内置 YOLO |
| `/detections` | `Detection2DArray` | 适配器→融合 | 统一视觉检测格式 |
| `/scan` | `sensor_msgs/LaserScan` | 传感器→融合 | LiDAR 扫描 |
| `/radar/targets` | `RadarTargetArray` | 传感器→适配器 | 雷达极坐标 |
| `/radar_objects` | `RadarObjectArray` | 适配器→融合 | 雷达笛卡尔 |
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
geometry_msgs/Point position   # 3D位置 (相机坐标系)
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
├── mmwRadar_ros/                               # 毫米波雷达驱动 (不修改)
├── depthai-ros/                                # OAK-D 相机驱动 (不修改)
├── rplidar_ros/                                # LiDAR 驱动 (不修改)
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
维护滑动时间窗口 (100ms) 的观测缓存:
  cache.append(msg, timestamp=msg.header.stamp)
  valid_obs = [obs for obs in cache if obs.stamp >= now - time_window]
```

#### 5.1.2 空间对齐 — TF 坐标变换

```
FOR EACH observation Z_i:
    source_frame = Z_i 所属传感器 frame (camera/laser/radar_link)
    target_frame = "base_link"
    tf = lookup_transform(target_frame, source_frame, time=Z_i.stamp)
    Z_i^base = do_transform_point(Z_i, tf)
```

#### 5.1.3 目标生成 — LiDAR 聚类

```
FOR i in range(len(ranges)):
    IF range_min < ranges[i] < range_max:
        angle = angle_min + i * angle_increment
        x = ranges[i] * cos(angle); y = ranges[i] * sin(angle)
        points.append((x, y))
clusters = euclidean_clustering(points, threshold=0.3m)
FOR EACH cluster:
    center = mean(cluster); confidence ∝ len(cluster) / 30.0
```

#### 5.1.4 数据关联 — 马氏距离检验

```
步骤 1: 欧氏距离门限
    FOR EACH track_k, obs_j:
        IF ||track_k.pos - obs_j.pos|| < gate (3.0m):
            candidates[k].append(obs_j)

步骤 2: 马氏距离 (χ² 检验, df=2, 95%置信 → 5.991)
    y = z - H * x_pred               # 创新
    S = H * P_pred * H^T + R         # 创新协方差
    D_M^2 = y^T * S^{-1} * y
    IF D_M^2 < 5.991: 接受 H0 (属于目标)
    ELSE:              拒绝 H0 → 剔除离群值
    → 最近邻匹配: argmin(D_M^2)
```

#### 5.1.5 最优线性融合 (BLUE, Theorem 5.1)

```
传感器噪声模型 (Algorithm.md §5.5.1):
  视觉: σ²_cam = (σ²_c0 / conf) * exp(d² / (2*σ_c²))
  LiDAR: σ²_lidar = σ²_l0 * (N_ref / |C|)
  雷达: σ²_radar = σ²_r0 / (conf * (1 + α*|v_radial|/v₀))

最优融合权重 (精度加权, Theorem 5.1):
  λ_i = 1/σ²_i  (精度/information)
  w_i = λ_i / Σ λ_j
  z_fused = Σ w_i * z_i

融合后方差:
  σ²_fused = 1 / Σ λ_i
  R_fused = σ²_fused * I₂

该权重在线性无偏估计类中达到最小方差 (MVUE)，也是高斯假设下的 MLE。
先验-后验平滑步骤已移除 — 卡尔曼更新自动覆盖该功能 (§5.5.4)。
```

#### 5.1.6 Kalman Filter (DWNA 模型, §5.6)

```
状态: x = [px, py, vx, vy]^T
F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
H = [[1,0,0,0],[0,1,0,0]]
Q = q * [[dt⁴/4,0,dt³/2,0],
         [0,dt⁴/4,0,dt³/2],
         [dt³/2,0,dt²,0],
         [0,dt³/2,0,dt²]]       (DWNA 离散白噪声加速度)
R_fused = σ²_fused * I₂          (由融合精度实时计算)

预测: x_pred = F*x, P_pred = F*P*F^T + Q
更新: K = P_pred*H^T*(H*P_pred*H^T + R_fused)^{-1}
      x_new = x_pred + K*(z_fused - H*x_pred)
      P_new = (I-K*H)*P_pred
```

#### 5.1.7 目标生命周期

```
状态机: CANDIDATE → CONFIRMED → LOST → DELETED
  关联 3 帧 → CONFIRMED (分配全局唯一ID)
  丢失 5 帧 → DELETED (移除)
  上限 max_tracks (20) → 按置信度保留
```

---

### 5.2 决策节点 (decision_node.py)

#### 5.2.1 TTC 计算

```
d = ||target_pos - robot_pos||
v_rel = (target_vel - robot_vel) · (target_pos - robot_pos) / d   # 径向投影
IF v_rel > 0.01:  TTC = d / v_rel
ELSE:             TTC = ∞ (远离, 无风险)
```

#### 5.2.2 分级决策

| TTC 范围 | 风险等级 | 行为 | 控制权 |
|----------|---------|------|--------|
| TTC > 5.0 s | SAFE | joystick 直通 | 手柄 |
| 3.0 < TTC ≤ 5.0 | WARNING | joystick 限速 ×0.7 | 手柄 (限速) |
| 1.0 < TTC ≤ 3.0 | SLOWDOWN | joystick 限速 ×0.5 + 绕行 | 半接管 |
| TTC ≤ 1.0 | EMERGENCY | v=0, ω=0, 接管控制权 | 紧急接管 |
| dist < 0.3 m | EMERGENCY | 无条件强制停车, 接管 | 紧急接管 |
| 紧急解除后 3s | RECOVERY | 缓慢限速, 冷却后归还手柄 | 冷却过渡 |

#### 5.2.3 绕行方向搜索

```
前方 180° 按 15° 分辨率分成 12 个扇区
标记 5m 内障碍物占据的扇区
选择未占据且最接近正前方的扇区作为绕行方向
全被占据 → 直行减速
```

### 5.3 STM32 串口协议 (serial_bridge.py)

#### 5.3.1 协议帧

```
┌──────────┬────────┬──────────┬──────────┬──────────┐
│ Header   │ Cmd ID │ Data Len │ Data     │ Checksum │
│ 2 bytes  │ 1 byte │ 1 byte   │ N bytes  │ 1 byte   │
├──────────┼────────┼──────────┼──────────┼──────────┤
│ 0x55 0xAA│  ...   │   N      │   ...    │  XOR     │
└──────────┴────────┴──────────┴──────────┴──────────┘

Checksum = Header[0] ^ Header[1] ^ CmdID ^ DataLen ^ Data[0] ^ ... ^ Data[N-1]
```

#### 5.3.2 速度指令 (Cmd ID = 0x01)

```
Data[0:2] = v_linear  (int16 BE, mm/s,   ±5000)
Data[2:4] = v_angular (int16 BE, 0.001rad/s, ±2000)

示例: v=0.2m/s, ω=0.1rad/s → 55 AA 01 04 00 C8 00 64 CS
```

#### 5.3.3 看门狗

```
每 50ms 发送速度帧 (20Hz)
超过 0.5s 未收到 /cmd_vel → 自动发送停车指令
```

---

## 6. 硬件接口与连接

### 6.1 传感器连接

| 传感器 | 接口 | Jetson 设备路径 | 参数 |
|--------|------|----------------|------|
| OAK-D PRO | USB 3.0 | USB (自动识别) | camera_name=oak |
| RPLIDAR A1 | USB 2.0 | `/dev/ttyUSB0` | baud=115200 |
| MS60-3015S80M4 | CH340 USB-UART | `/dev/ttyUSB1` | baud=921600 |

### 6.2 STM32 串口

| 参数 | 值 |
|------|-----|
| 设备路径 | `/dev/ttyTHS2` (USART3) |
| 波特率 | 115200 / 8N1 |
| 电平 | 3.3V TTL |

### 6.3 权限设置

```bash
sudo usermod -a -G dialout nvidia
sudo chmod 666 /dev/ttyTHS2
# 持久化:
echo 'KERNEL=="ttyTHS*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-jetson-uart.rules
```

---

## 7. 构建与运行

```bash
# 依赖
sudo apt install -y python3-numpy python3-serial
pip3 install pyserial numpy

# 构建自定义消息
cd ~/ADAS_Fusion
colcon build --symlink-install --packages-select adas_fusion_msgs
source install/setup.bash

# 构建融合包
colcon build --symlink-install --packages-select adas_fusion
source install/setup.bash

# 一键启动
ros2 launch adas_fusion system.launch.py

# 带参数
ros2 launch adas_fusion system.launch.py \
    max_linear_vel:=0.5 ttc_emergency:=0.8 serial_port:=/dev/ttyTHS2

# 仅传感器
ros2 launch adas_fusion sensors.launch.py
# 仅融合决策
ros2 launch adas_fusion fusion_decision.launch.py
```

---

## 8. 配置参数说明

### 8.1 融合参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `time_window` | 0.1 s | 时间同步窗口 |
| `association_gate` | 3.0 m | 欧氏距离关联门限 |
| `mahalanobis_threshold` | 5.991 | χ²(df=2,95%) 阈值 |
| `confirm_threshold` | 3 | 确认新目标所需帧数 |
| `delete_threshold` | 5 | 删除丢失目标所需帧数 |
| `max_tracks` | 20 | 最大跟踪数 |
| `dt` | 0.1 s | KF 时间步长 |
| `process_noise_q` | 0.5 | 过程噪声系数 (DWNA) |
| `sigma_cam_0` | 0.05 m | 相机基准标准差 |
| `sigma_cam_scale` | 5.0 m | 相机距离衰减尺度 |
| `sigma_lidar_0` | 0.03 m | LiDAR 基准标准差 |
| `sigma_radar_0` | 0.2 m | 雷达基准标准差 |
| `radar_vel_alpha` | 0.5 | 雷达速度增益 |
| `radar_vel_ref` | 10.0 m/s | 雷达参考速度 |

### 8.2 手柄控制参数 (v2.0 新增)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_joystick` | true | 启用手柄控制 |
| `cooldown_seconds` | 3.0 s | 紧急恢复冷却时间 |
| `joy_topic` | `/joy` | 手柄输入话题 |
| `joy_axis_linear` | 1 | 线速度轴 (左摇杆上下) |
| `joy_axis_angular` | 3 | 角速度轴 (右摇杆左右) |
| `joy_deadzone` | 0.1 | 摇杆死区 |

### 8.3 决策参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ttc_warning` | 5.0 s | 预警阈值 |
| `ttc_slowdown` | 3.0 s | 减速阈值 |
| `ttc_emergency` | 1.0 s | 急停阈值 |
| `max_linear_vel` | 0.3 m/s | 最大线速度 |
| `max_angular_vel` | 0.5 rad/s | 最大角速度 |
| `slowdown_factor` | 0.5 | 减速因子 |
| `min_safe_distance` | 0.3 m | 绝对安全距离 |

### 8.3 串口参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyTHS2` | Jetson UART |
| `baud_rate` | 115200 | 波特率 |
| `watchdog_timeout` | 0.5 s | 超时自动停车 |

---

## 9. 安全设计

### 9.1 多层防护

```
Layer 1: min_safe_distance (0.3m) → 无条件急停
Layer 2: TTC ≤ 1.0s → 紧急停车
Layer 3: TTC ≤ 3.0s → 大幅减速 + 绕行尝试
Layer 4: TTC ≤ 5.0s → 预警减速
Layer 5: 看门狗超时 (0.5s 无 cmd_vel) → 串口桥自动停车
Layer 6: 串口断连 → 自动重连 + 停车保护
```

### 9.2 已知局限

1. TF 外参需根据实际安装位置手动配置
2. LiDAR 聚类使用简单欧氏聚类, 复杂场景可用 DBSCAN
3. 绕行仅做方向选择, 完整路径规划应集成 Nav2
4. 可加入加速度 ramp 避免急加速
5. 手柄控制依赖标准 ROS 2 `joy` 包, 需确保 `/dev/input/js0` 可读
6. 传感器噪声模型基准参数 (σ_c0, σ_l0, σ_r0) 建议通过标定实验校准

---

### 附录 A: 算法入口索引

| 算法 | 文件 | 方法 |
|------|------|------|
| 时间同步 | `fusion_node.py` | `_collect()` |
| 空间对齐 | `fusion_node.py` | `_collect()` — TF lookup |
| LiDAR 聚类 | `fusion_node.py` | `_euclidean_clustering()` |
| 数据关联 | `fusion_node.py` | `_associate()` |
| 马氏检验 | `fusion_node.py` | `Track.mahalanobis_distance_sq()` |
| 自适应融合 | `fusion_node.py` | `_adaptive_fusion()` |
| Kalman Filter | `fusion_node.py` | `Track.predict()` / `Track.update()` |
| TTC 计算 | `decision_node.py` | `_decision_cycle()` |
| 绕行搜索 | `decision_node.py` | `_find_safe_direction()` |
| STM32 协议 | `serial_bridge.py` | `Stm32Protocol.encode_velocity()` |
