# CHANGELOG

---

## [v2.1.0] — 2026-05-04

### Fixed
- `sensors.launch.py`: 用 `try/except PackageNotFoundError` 包裹传感器包检测, 包不存在时打印警告而非崩溃
- `system.launch.py`: `enable_camera` 默认值改为 `false` (相机需 OAK-D 硬件 + libdepthai)
- `detection_adapter.py`: `depthai_ros_msgs` 改为可选 import, 构建失败时有明确提示
- `RUN.sh`: 移除 Step 3 中的 `depthai_ros_driver` 构建尝试 (VM 上必然失败, 需独立 Jetson SDK)
- `RUN.sh`: 新增 `./RUN.sh camera` 命令 — 仅在 Jetson 硬件上独立构建相机驱动

### Added
- `RUN.sh`: 一站式构建启动脚本 (build / start / advanced / camera / deps / clean)
- `CHANGELOG.md`: 版本更新日志

---

## [v2.0.0] — 2026-05-02

### Changed
- `fusion_node.py`: 重构融合算法
  - 传感器噪声模型: 方差建模替代启发式权重 (§5.5.1)
    - 视觉: σ²_cam = (σ²_c0 / conf) * exp(d² / (2σ_c²))
    - LiDAR: σ²_lidar = σ²_l0 * (N_ref / |C|)
    - 雷达: σ²_radar = σ²_r0 / (conf * (1 + α|v|/v₀))
  - 融合权重: 精度加权 BLUE w_i = λ_i / Σλ_j (Theorem 5.1)
  - Q 矩阵: DWNA 模型 (dt⁴/4, dt³/2, dt²) 替代 G·Gᵀ
  - R_fused: 每帧由融合精度实时计算
  - 移除先验-后验平滑 (KF 更新自动覆盖)
- `decision_node.py`: 手柄控制 + 紧急接管架构
  - 订阅 `/joy` (手柄), 正常模式下手柄直通
  - 紧急条件 (TTC≤1s 或 dist<0.3m): 接管控制权
  - 3s 冷却后自动归还手柄控制权
- `fusion_params.yaml`: 更新参数表
  - 新增: sigma_cam_0, sigma_cam_scale, sigma_lidar_0, sigma_radar_0, radar_vel_alpha, radar_vel_ref
  - 新增: enable_joystick, cooldown_seconds, joy_topic, joy_axis_linear, joy_axis_angular
  - 移除: camera_weight, lidar_weight, radar_weight, measurement_noise_r
- `Algorithm.md`: 更新至 v2.0 (最优融合理论 + 定理证明)
- `ADAS_Fusion_设计说明书.md`: 同步 v2.0 更新

### Added
- `launch/fusion_decision.launch.py`: 新增 joy_node (手柄驱动)
- `launch/system.launch.py`: 新增 enable_joystick / cooldown_seconds 参数

---

## [v1.0.0] — 2026-04-28

### Added
- `adas_fusion_msgs/`: 自定义 ROS 2 消息包
  - Detection2D / Detection2DArray: 视觉检测结果
  - RadarObject / RadarObjectArray: 雷达目标 (笛卡尔坐标)
  - TrackedObject / TrackedObjectArray: 融合跟踪目标
- `adas_fusion/`: 主功能包
  - `detection_adapter.py`: depthai SpatialDetectionArray → Detection2DArray
  - `radar_adapter.py`: 毫米波雷达极坐标 → 笛卡尔坐标转换
  - `fusion_node.py`: 多传感器融合 + Kalman Filter 跟踪
  - `decision_node.py`: TTC 分级避障决策
  - `serial_bridge.py`: Jetson ↔ STM32F407 UART 串口桥接
  - `config/fusion_params.yaml`: 全局参数配置
  - `launch/sensors.launch.py`: 传感器驱动启动
  - `launch/fusion_decision.launch.py`: 融合 + 决策启动
  - `launch/system.launch.py`: 全系统一键启动
- `Algorithm.md`: 算法规范 (v1.0)
- `ADAS_Fusion_设计说明书.md`: 软件设计说明书
- `Overview.md`: 项目需求大纲
- `.gitignore`: userguide/, ds/*.pdf, build/, install/, log/
