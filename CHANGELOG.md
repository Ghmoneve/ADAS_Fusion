# CHANGELOG

所有对该项目的显著变更均记录于此。版本号遵循 [语义化版本](https://semver.org/lang/zh-CN/) 规范。

---

## [v2.1.2] — 2026-05-04

### Fixed
- `system.launch.py`: 移除传给 `sensors.launch.py` 的未声明参数 `camera_model` / `nn_type` (导致 launch RuntimeError)
- `sensors.launch.py`: `PackageNotFoundError` 导入兼容旧版 ament_index_python, 增加 `LookupError` 兜底

---

## [v2.1.1] — 2026-05-04

### Fixed
- `sensors.launch.py`: 用 `try/except PackageNotFoundError` 包裹所有传感器包检测, 包不存在时打印警告而非崩溃整个 launch
- `system.launch.py`: `enable_camera` 默认值改为 `false` (相机需 OAK-D 硬件 + libdepthai)
- `detection_adapter.py`: `depthai_ros_msgs` 改为 try/except 可选 import, 构建失败时有明确 runtime 提示而非 import crash
- `RUN.sh`: 移除 Step 3 中的 `depthai_ros_driver` 自动构建 (VM 上必然失败, libdepthai 仅 Jetson L4T / Intel OpenVINO 可用)
- `RUN.sh`: 新增 `./RUN.sh camera` 独立命令 → 仅在 Jetson 硬件上构建相机驱动

### Added
- `RUN.sh`: 一站式构建启动脚本 (命令: `build` / `camera` / `start` / `advanced` / `deps` / `clean`)
- `CHANGELOG.md`: 版本更新日志文件

---

## [v2.1.0] — 2026-05-04

### Fixed
- `detection_adapter.py`: bbox center 字段修正 — `Pose2D` 使用 `.x` / `.y` 直访, 非 `.position.x`
- `detection_adapter.py`: `ObjectHypothesis.class_id` 为 string → 增加 int 转换容错
- `fusion_params.yaml`: `depthai_detection_topic` 默认值更新为 `/oak/color/yolov4_Spatial_detections`

### Added
- `RUN.sh`: 初始版本 (build / start / advanced / deps / clean)

---

## [v2.0.0] — 2026-05-02

### Changed
- `fusion_node.py`: 重构融合算法 (算法依据: Algorithm.md v2.0)
  - **传感器噪声模型** — 方差建模替代启发式权重 (§5.5.1)
    - 视觉: σ²_cam = (σ²_c0 / conf) * exp(d² / (2σ_c²))
    - LiDAR: σ²_lidar = σ²_l0 * (N_ref / |C|)
    - 雷达: σ²_radar = σ²_r0 / (conf * (1 + α|v_radial|/v₀))
  - **融合权重** — 精度加权 BLUE: w_i = λ_i / Σλ_j, λ_i = 1/σ²_i (Theorem 5.1)
  - **Q 矩阵** — DWNA 离散白噪声加速度模型 (dt⁴/4, dt³/2, dt²)
  - **R_fused** — 每帧由融合精度实时计算 `σ²_fused = 1 / Σλ_i`, 替代固定 `measurement_noise_r`
  - **移除**先验-后验平滑步骤 (已被 KF 更新覆盖, §5.5.4)
  - KF 更新接口改为 `update(z, R_obs)`, 马氏检验接受 `sigma2` 参数
- `decision_node.py`: 手柄控制 + 紧急接管
  - 订阅 `/joy` (sensor_msgs/Joy), 正常模式下直通手柄 cmd_vel
  - 紧急条件 (TTC ≤ 1s 或 dist < 0.3m): 接管控制权, 输出避障指令
  - 3s 冷却期后自动归还手柄控制权
  - 手柄映射: 轴1→线速度, 轴3→角速度, 死区 0.1
  - 风险分级 WARNING/SLOWDOWN 时对手柄速度限幅
- `fusion_params.yaml`: 参数表重构
  - **新增**: sigma_cam_0, sigma_cam_scale, sigma_lidar_0, sigma_radar_0, radar_vel_alpha, radar_vel_ref
  - **新增**: enable_joystick, cooldown_seconds, joy_topic, joy_axis_linear, joy_axis_angular, joy_deadzone, min_safe_distance
  - **移除**: camera_weight, lidar_weight, radar_weight, measurement_noise_r, desired_linear_vel, desired_angular_vel
- `Algorithm.md`: 更新至 v2.0 — 最优融合理论 + 严格定理证明
- `ADAS_Fusion_设计说明书.md`: 同步 v2.0 更新 (版本标注、架构图、参数表、已知局限)

### Added
- `launch/fusion_decision.launch.py`: 新增 joy_node (标准 ROS 2 joy 包)
- `launch/system.launch.py`: 新增 enable_joystick / cooldown_seconds 参数

---

## [v1.0.0] — 2026-04-28

### Added
- `adas_fusion_msgs/`: 自定义 ROS 2 消息包 (CMake, 8 文件)
  - `Detection2D.msg` / `Detection2DArray.msg` — 视觉检测结果 (bbox + class + confidence + depth)
  - `RadarObject.msg` / `RadarObjectArray.msg` — 雷达目标笛卡尔坐标 (position + velocity)
  - `TrackedObject.msg` / `TrackedObjectArray.msg` — 融合跟踪目标 (id + class + position + velocity + source_flag)
- `adas_fusion/`: 主功能包 (Python, 14 文件)
  - `detection_adapter.py` (~100 行) — depthai-ros SpatialDetectionArray → Detection2DArray
  - `radar_adapter.py` (~110 行) — mmWave RadarTargetArray (极坐标) → RadarObjectArray (笛卡尔)
  - `fusion_node.py` (~550 行) — 多传感器融合 (时间同步 + TF 空间对齐 + LiDAR 欧氏聚类 + 马氏距离数据关联 + 自适应贝叶斯融合 + Kalman Filter 跟踪 + 目标生命周期管理)
  - `decision_node.py` (~280 行) — TTC 分级避障决策 (SAFE/WARNING/SLOWDOWN/STOP + 绕行扇区搜索)
  - `serial_bridge.py` (~300 行) — Jetson ↔ STM32F407 UART 串口桥接 (0x55 0xAA 协议帧, 看门狗)
  - `config/fusion_params.yaml` — 全局参数 (融合/决策/TF/串口/Topic 映射)
  - `launch/sensors.launch.py` — 传感器驱动启动 (OAK-D/RPLIDAR/mmWave Radar)
  - `launch/fusion_decision.launch.py` — 适配器 + 融合 + 决策 + 串口桥接
  - `launch/system.launch.py` — 全系统一键启动
- `mmwRadar_ros/mmw_radar_msgs/`: 毫米波雷达消息定义 (RadarTarget / RadarTargetArray)
- `mmwRadar_ros/mmw_radar_driver/`: 毫米波雷达 ROS 2 驱动 (MS60-3015S80M4, AT6010 SOC)
  - 串口通信 (serial_driver.py), 协议解析 (protocol.py), 动态配置 (config_manager.py)
- `turtlebot4/`: Turtlebot4 参考包 (导航/控制模式参考)
- **文档**: `Overview.md` (需求大纲), `Algorithm.md` (算法规范 v1.0), `ADAS_Fusion_设计说明书.md` (设计说明书), `cam_lidar_Pkg.md` (CLAUDE.md 重命名)
- **工程文件**: `.gitignore` (排除 userguide/, ds/*.pdf, build/, install/, log/)
