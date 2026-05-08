# ADAS Fusion 实验子项目 — 设计说明文档

> **文档版本**: v1.0 | **日期**: 2026-05-08  
> **项目**: ADAS Fusion — 多传感器融合自动紧急避障系统  
> **实验设计依据**: `Exp_design.md`  
> **算法依据**: `Description_Algorithm.md`

---

## 目录

1. [项目概述](#1-项目概述)
2. [设计理念与架构](#2-设计理念与架构)
3. [完整目录结构](#3-完整目录结构)
4. [数据采集模块 — Data_collection](#4-数据采集模块--data_collection)
   - [4.1 采集程序 data_collector.py](#41-采集程序-data_collectorpy)
   - [4.2 数据格式规范](#42-数据格式规范)
   - [4.3 关键帧采集逻辑](#43-关键帧采集逻辑)
5. [绘图模块 — plot_py](#5-绘图模块--plot_py)
   - [5.1 共用工具模块 plot_utils.py](#51-共用工具模块-plot_utilspy)
   - [5.2 LiDAR 距离图脚本](#52-lidar-距离图脚本)
   - [5.3 雷达方位与速度图脚本](#53-雷达方位与速度图脚本)
   - [5.4 TTC 变化图脚本](#54-ttc-变化图脚本)
   - [5.5 运动状态图脚本](#55-运动状态图脚本)
   - [5.6 融合方差对比图脚本](#56-融合方差对比图脚本)
   - [5.7 有光/暗环境对照图脚本](#57-有光暗环境对照图脚本)
6. [图像输出模块 — Image](#6-图像输出模块--image)
7. [评价报告模块 — evaluation](#7-评价报告模块--evaluation)
8. [启动脚本 collect.sh](#8-启动脚本-collectsh)
9. [完整使用流程](#9-完整使用流程)
   - [9.1 前置条件](#91-前置条件)
   - [9.2 数据采集](#92-数据采集)
   - [9.3 生成图表](#93-生成图表)
   - [9.4 填写评价报告](#94-填写评价报告)
10. [数据与图像流转全景](#10-数据与图像流转全景)

---

## 1. 项目概述

本实验子项目是 ADAS Fusion 多传感器融合系统的验证平台。其核心任务是在 1:10 缩微沙盘环境中，使用 TurtleBot4 平台运行四类紧急避障场景（外加暗环境对照场景），通过记录三传感器原始数据、融合算法输出和决策系统状态，验证以下两个关键问题：

1. **融合算法的正确性**：三种异构传感器（OAK-D PRO 深度相机、RPLIDAR A1 激光雷达、MS60-3015S80M4 毫米波雷达）的观测是否被正确同步、对齐、关联、融合并滤波追踪；
2. **TTC 分级决策的有效性**：基于融合输出的 TTC（碰撞时间）计算是否正确触发 SAFE → WARNING → SLOWDOWN → EMERGENCY 四级避障决策，以及紧急停车是否在碰撞前完成。

实验子项目提供了一整套从**数据采集 → 原始数据存储 → 图表生成 → 评价报告撰写**的完整流水线。

### 五个实验场景

| 场景 | 名称 | 道路特征 | 测试焦点 |
|------|------|---------|---------|
| 场景一 | 匀速跟驰 | 直线道路标线 | 纵向 TTC 分级避障基础逻辑 |
| 场景二 | 静态抛锚车避让 | 十字路口引道 | 静态障碍物检测与绕行搜索 |
| 场景三 | 弯道前车突然制动 | 弯道标线 | 曲率约束下的 TTC 计算与减速 |
| 场景四 | 十字路口侧向切入 | 十字路口交汇区 | 横向威胁感知与四向避障拓展 |
| 场景五 | 暗环境匀速跟驰 | 直线道路（无光源） | 视觉失效下 LiDAR+雷达主导融合的适应性 |

---

## 2. 设计理念与架构

### 2.1 核心设计原则

1. **数据完整可追溯**：每一张图表都有对应的原始 CSV 数据和生成它的 Python 脚本。三者通过命名规范建立一一对应关系，任何图表均可被独立重新生成；
2. **模块高内聚低耦合**：数据采集、绘图、评价三个环节各自独立。采集程序只负责写 CSV 和捕获关键帧；绘图脚本只负责从 CSV 读取并生成 JPG；评价文件由实验人员根据图表和数据手工撰写；
3. **精度严格可控**：所有数值输出按统一精度规范（时间 ms 级 `%.3f`，速度 `%.3f`，方位角 `%.2f`），保证数据在不同场景间可比；
4. **图表规范统一**：所有绘图脚本使用同一套 matplotlib 配置（中文字体 SimSun / 宋体，英文与数字 Times New Roman，dpi=300），图例、横纵坐标标签、图题齐全。

### 2.2 数据流转全景

```
┌─────────────────────┐
│  TurtleBot4 传感器   │
│  (Camera/LiDAR/Radar)│
│  + Odom 里程计       │
│  + Fusion 融合节点   │
│  + Decision 决策节点 │
└────────┬────────────┘
         │ ROS2 topics
         ▼
┌─────────────────────┐
│  data_collector.py  │  ← 10 Hz 采样，风险等级切换时捕获关键帧
│  (ROS2 Node)        │
└────────┬────────────┘
         │ 写入 CSV + JPG
         ▼
┌─────────────────────────┐
│  Data_collection/       │
│  └─ raw_data/           │
│      └─ scene_n/run_m/  │
│          ├─ motion.csv   │
│          ├─ lidar.csv    │
│          ├─ radar.csv    │
│          ├─ fusion.csv   │
│          ├─ ttc.csv      │
│          └─ camera_keyframes/
│              └─ scene_n_phase_X_LEVEL.jpg
└────────┬────────────────┘
         │ 脚本读取 CSV
         ▼
┌─────────────────────┐
│  plot_py/            │
│  各场景绘图脚本      │  ← 读取 CSV → matplotlib 绘图 → 保存 JPG
│  (调用 plot_utils)   │
└────────┬────────────┘
         │ 输出 JPG @ 300dpi
         ▼
┌─────────────────────┐
│  Image/scene_n/      │
│  ├─ scene_n_lidar.jpg│
│  ├─ scene_n_radar.jpg│
│  ├─ scene_n_TTC.jpg  │
│  └─ scene_n_motion.jpg│
└─────────────────────┘
         │
         ▼ 实验人员参照
┌─────────────────────┐
│  evaluation/         │
│  └─ scene_n_evaluation.md
└─────────────────────┘
```

---

## 3. 完整目录结构

以下列出实验子项目的全部文件和目录（标注 **已有** 者不纳入本次构建，标注 **[生成]** 者为实验运行后动态产出）：

```
ADAS_Fusion/                               ← 项目根目录
├── collect.sh                             ← [新增] 数据采集启动脚本，与 RUN.sh 并列
├── RUN.sh                                 ← [已有] 主启动脚本
├── Description_Algorithm.md               ← [已有] 融合算法综述文档
│
└── Expriment/                             ← [实验子项目根目录]
    ├── Exp_design.md                      ← [已有] 实验设计文档
    ├── Design_illustration.md             ← [本文档] 设计说明
    │
    ├── Data_collection/                   ← 数据采集模块
    │   ├── data_collector.py              ← ROS2 数据采集节点
    │   └── raw_data/                      ← 原始数据存储（运行后产出）
    │       ├── scene_1/
    │       │   ├── run_1/                 ← [生成] 第 1 次运行
    │       │   │   ├── motion.csv
    │       │   │   ├── lidar.csv
    │       │   │   ├── radar.csv
    │       │   │   ├── fusion.csv
    │       │   │   ├── ttc.csv
    │       │   │   └── camera_keyframes/
    │       │   │       ├── scene_1_phase_1_SAFE.jpg
    │       │   │       ├── scene_1_phase_2_WARNING.jpg
    │       │   │       ├── scene_1_phase_3_SLOWDOWN.jpg
    │       │   │       └── scene_1_phase_4_EMERGENCY.jpg
    │       │   └── run_2/                 ← [生成] 第 2 次运行（可多次采集）
    │       ├── scene_2/                   ← 同 scene_1 结构
    │       ├── scene_3/                   ← 同 scene_1 结构
    │       ├── scene_4/                   ← 同 scene_1 结构
    │       └── scene_5/                   ← 同 scene_1 结构（暗环境）
    │
    ├── plot_py/                           ← 绘图脚本模块
    │   ├── plot_utils.py                  ← [核心] 共用工具 + 所有绘图函数定义
    │   │
    │   ├── scene_1_lidarPlot.py           ← 场景一 LiDAR 距离图入口
    │   ├── scene_1_radarPlot.py           ← 场景一 雷达方位/速度图入口
    │   ├── scene_1_TTCPlot.py             ← 场景一 TTC 变化图入口
    │   ├── scene_1_motionPlot.py          ← 场景一 运动状态图入口
    │   │
    │   ├── scene_2_lidarPlot.py           ← 场景二 LiDAR 距离图入口
    │   ├── scene_2_radarPlot.py           ← 场景二 雷达方位/速度图入口
    │   ├── scene_2_TTCPlot.py             ← 场景二 TTC 变化图入口
    │   ├── scene_2_motionPlot.py          ← 场景二 运动状态图入口
    │   │
    │   ├── scene_3_lidarPlot.py           ← 场景三 LiDAR 距离图入口
    │   ├── scene_3_radarPlot.py           ← 场景三 雷达方位/速度图入口
    │   ├── scene_3_TTCPlot.py             ← 场景三 TTC 变化图入口
    │   ├── scene_3_motionPlot.py          ← 场景三 运动状态图入口
    │   │
    │   ├── scene_4_lidarPlot.py           ← 场景四 LiDAR 距离图入口
    │   ├── scene_4_radarPlot.py           ← 场景四 雷达方位/速度图入口
    │   ├── scene_4_TTCPlot.py             ← 场景四 TTC 变化图入口
    │   ├── scene_4_motionPlot.py          ← 场景四 运动状态图入口
    │   │
    │   ├── scene_5_lidarPlot.py           ← 场景五 LiDAR 距离图入口
    │   ├── scene_5_radarPlot.py           ← 场景五 雷达方位/速度图入口
    │   ├── scene_5_TTCPlot.py             ← 场景五 TTC 变化图入口
    │   ├── scene_5_motionPlot.py          ← 场景五 运动状态图入口
    │   │
    │   ├── fusion_variance_plot.py        ← 跨场景融合方差对比图入口
    │   └── scene_1_vs_scene_5_comparison.py ← 场景一 vs 场景五对照图入口
    │
    ├── Image/                             ← 图像输出模块
    │   ├── scene_1/                       ← [生成] 场景一图像输出目录
    │   │   ├── scene_1_lidar.jpg
    │   │   ├── scene_1_radar.jpg
    │   │   ├── scene_1_TTC.jpg
    │   │   └── scene_1_motion.jpg
    │   ├── scene_2/                       ← [生成] 场景二图像
    │   ├── scene_3/                       ← [生成] 场景三图像
    │   ├── scene_4/                       ← [生成] 场景四图像
    │   ├── scene_5/                       ← [生成] 场景五图像
    │   ├── fusion_variance_comparison.jpg  ← [生成] 跨场景融合方差对比图
    │   └── scene_1_vs_scene_5_comparison.jpg ← [生成] 有光/暗环境对照图
    │
    └── evaluation/                        ← 评价报告模块
        ├── scene_1_evaluation.md          ← 场景一评价报告模板
        ├── scene_2_evaluation.md          ← 场景二评价报告模板
        ├── scene_3_evaluation.md          ← 场景三评价报告模板
        ├── scene_4_evaluation.md          ← 场景四评价报告模板
        └── scene_5_evaluation.md          ← 场景五评价报告模板
```

---

## 4. 数据采集模块 — Data_collection

### 4.1 采集程序 `data_collector.py`

#### 4.1.1 概述

`data_collector.py` 是一个 **ROS2 节点**（继承自 `rclpy.node.Node`），负责在实验运行期间订阅所有相关 ROS2 topic，以 10 Hz 固定频率记录结构化数据到 CSV 文件，并在风险等级切换时刻自动捕获相机关键帧。

**设计依据**：融合周期为 10 Hz（100 ms），采集频率与之对齐，避免漏帧或重复采样。时间戳以 `time.time()` 纪元秒为单位，记录为自采集启动时刻起的相对秒数（`now - self._start_time`），精度为微秒（`.6f`）。

#### 4.1.2 订阅的 ROS2 Topic

| Topic | 消息类型 | QoS | 用途 |
|-------|---------|-----|------|
| `<odom_topic>` (默认 `/odom`) | `nav_msgs/Odometry` | BEST_EFFORT | 获取 TurtleBot4 实时位姿与速度，写入 `motion.csv` |
| `/scan` | `sensor_msgs/LaserScan` | BEST_EFFORT | 获取 LiDAR 扫描数据，提取正前方 ±15° 范围内的最近距离，写入 `lidar.csv` |
| `/radar/data` | `std_msgs/String` | RELIABLE | 获取毫米波雷达目标数据（逗号分隔: `bearing,dist,velocity`），写入 `radar.csv` |
| `/fusion/tracked_objects` | `std_msgs/String` | RELIABLE | 获取融合节点输出的追踪目标（JSON 格式，含位置、速度、融合方差），写入 `fusion.csv` |
| `/decision/ttc` | `std_msgs/String` | RELIABLE | 获取决策节点输出的 TTC 值和风险等级（逗号分隔: `ttc_value,risk_level`），写入 `ttc.csv` |
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | BEST_EFFORT | 获取 OAK-D PRO 的 RGB 图像，用于关键帧捕获 |

#### 4.1.3 关键设计细节

**里程计数据获取自车运动信息**：

小车速度来源于 OAK-D PRO 的 VIO 里程计或 Create 3 底盘的轮式里程计，通过 `<odom_topic>` 参数可配置。OAK-D PRO 支持视觉-惯性里程计（VIO），在纹理丰富的室内环境中可提供高精度位姿估计；Create 3 底盘的 `/odom` 提供轮式里程计，两者均可通过 `--odom_topic` 参数切换。

里程计回调记录三个轴的位置和线速度分量，在 `_record_tick()` 中以欧几里得范数合成标量位移和标量速度：
- `displacement = sqrt(pos_x² + pos_y² + pos_z²)`
- `velocity = sqrt(vel_x² + vel_y² + vel_z²)`
- `acceleration` 暂置为 0（后续可通过速度差分实时计算）

**LiDAR 前方距离提取**：

LiDAR 扫描为 360° 点云。采集程序仅提取正前方 ±15°（0.2618 弧度）范围内的有效测距值（`range_min < r < range_max`），取最小值作为距障碍物距离。这模拟了车辆前向碰撞预警的实际感知范围。

**内存缓冲区解耦回调与记录线程**：

每个传感器的回调函数仅将最新数据追加到线程安全的 `collections.deque(maxlen=10)` 中；10 Hz 定时器 `_record_tick()` 从每个 buffer 取最后一条记录写入 CSV。这种设计避免了回调阻塞和线程竞争。

**毫秒级时间精度**：

时间戳以 `%.6f` 格式写入（即微秒级），在后续绘图脚本中通过 matplotlib 的 `FuncFormatter` 仅显示到毫秒位（`%.3f`），兼顾存储精度和显示清晰性。

#### 4.1.4 命令行参数

| 参数 | 类型 | 必选 | 默认值 | 说明 |
|------|------|------|--------|------|
| `--scene` | int (1-5) | 是 | — | 场景编号，决定输出子目录 |
| `--run` | int | 是 | — | 运行编号，同一场景可多次运行 |
| `--duration` | int | 否 | 120 | 采集时长（秒），到时自动停止 |
| `--odom_topic` | str | 否 | `/odom` | 里程计话题名 |

#### 4.1.5 程序执行流程

```
main()
  │
  ├── 1. 解析命令行参数
  ├── 2. rclpy.init()
  ├── 3. 创建 DataCollector 节点
  │     ├── 创建输出目录 raw_data/scene_n/run_m/
  │     ├── 打开 5 个 CSV 文件并写入表头
  │     ├── 订阅 6 个 ROS2 topic
  │     └── 创建 10 Hz 记录定时器
  ├── 4. 创建 duration 秒后自动关闭的定时器
  └── 5. rclpy.spin() — 进入事件循环
        │
        ├── 传感器回调（每个 topic 独立异步触发）
        │    ├── _odom_callback    → 追加到 _odom_buffer
        │    ├── _lidar_callback   → 提取前方 min 距离 → _lidar_buffer
        │    ├── _radar_callback   → 解析 bearing,dist,vel → _radar_buffer
        │    ├── _fusion_callback  → 解析 JSON → _fusion_buffer
        │    ├── _ttc_callback     → 解析 ttc,risk → _ttc_buffer
        │    │                       └─ 风险等级变化 → _capture_keyframe()
        │    └── _camera_callback  → 缓存最新帧
        │
        └── 10 Hz _record_tick    → 从各 buffer 取最新 → 写入 CSV 行
```

---

### 4.2 数据格式规范

每个 CSV 文件第一行为列标题，后续行为时间序列数据。

#### motion.csv — 自车运动状态

| 列 | 类型 | 精度 | 单位 | 说明 |
|-----|------|------|------|------|
| `timestamp` | float | `.6f` | s | 自采集启动起的相对时间 |
| `displacement_m` | float | `.6f` | m | 自起点起的欧几里得位移 |
| `velocity_ms` | float | `.3f` | m/s | 合速度（取 3 轴速度分量欧几里得范数） |
| `acceleration_ms2` | float | `.3f` | m/s² | 合加速度（当前置 0，预留扩展） |

#### lidar.csv — LiDAR 距离数据

| 列 | 类型 | 精度 | 单位 | 说明 |
|-----|------|------|------|------|
| `timestamp` | float | `.6f` | s | 相对时间 |
| `distance_to_obstacle_m` | float | `.4f` | m | 正前方 ±15° 内最近障碍物距离 |

#### radar.csv — 毫米波雷达数据

| 列 | 类型 | 精度 | 单位 | 说明 |
|-----|------|------|------|------|
| `timestamp` | float | `.6f` | s | 相对时间 |
| `bearing_deg` | float | `.2f` | ° | 目标方位角 |
| `distance_m` | float | `.4f` | m | 目标径向距离 |
| `radial_velocity_ms` | float | `.3f` | m/s | 目标径向速度 |

#### fusion.csv — 融合算法输出

| 列 | 类型 | 精度 | 单位 | 说明 |
|-----|------|------|------|------|
| `timestamp` | float | `.6f` | s | 相对时间 |
| `fused_px` | float | `.4f` | m | 融合后目标 X 坐标 (base_link) |
| `fused_py` | float | `.4f` | m | 融合后目标 Y 坐标 (base_link) |
| `fused_vx` | float | `.3f` | m/s | 融合后目标 X 方向速度 |
| `fused_vy` | float | `.3f` | m/s | 融合后目标 Y 方向速度 |
| `fusion_variance` | float | `.6f` | — | 融合后方差 σ²_fused |

#### ttc.csv — TTC 与决策状态

| 列 | 类型 | 精度 | 单位 | 说明 |
|-----|------|------|------|------|
| `timestamp` | float | `.6f` | s | 相对时间 |
| `ttc_value_s` | float | `.4f` | s | TTC 计算值 |
| `risk_level` | string | — | — | 风险等级: SAFE/WARNING/SLOWDOWN/EMERGENCY/RECOVERY |

---

### 4.3 关键帧采集逻辑

关键帧由 `_ttc_callback` 触发：当从决策节点收到的风险等级字符串与上一次不同时（`risk != self._last_risk_level`），调用 `_capture_keyframe(risk)` 将 `cv_bridge` 转换后的当前相机帧保存为 JPEG。

**命名规则**：
```
scene_{scene}_phase_{phase_number}_{risk_level}.jpg
```

其中 `phase_number` 的映射关系为：

| 风险等级 | 阶段编号 | 文件名示例 |
|---------|---------|-----------|
| SAFE | 1 | `scene_1_phase_1_SAFE.jpg` |
| WARNING | 2 | `scene_1_phase_2_WARNING.jpg` |
| SLOWDOWN | 3 | `scene_1_phase_3_SLOWDOWN.jpg` |
| EMERGENCY | 4 | `scene_1_phase_4_EMERGENCY.jpg` |
| RECOVERY | 5 | `scene_1_phase_5_RECOVERY.jpg` |

若 `cv_bridge` 不可用（Python 环境未安装），关键帧功能自动禁用，不影响其他数据采集。

---

## 5. 绘图模块 — plot_py

### 5.1 共用工具模块 `plot_utils.py`

#### 5.1.1 在设计中的地位

`plot_utils.py` 是整个绘图系统的核心，采用 **"功能定义 + 薄入口脚本"** 的二层架构：

- **`plot_utils.py`**：定义所有实际绘图逻辑（函数），配置 matplotlib 全局样式，提供路径计算、数据加载、风险背景着色等共用工具；
- **各 `scene_n_*Plot.py`**：每个脚本仅 5 行代码——导入 `plot_utils`，调用对应的绘图函数并传入场景编号。这种设计使每张图可独立运行、独立调试。

这样设计的目的是：
1. 避免在 20+ 个脚本中重复绘图逻辑，维护成本从 O(n) 降至 O(1)；
2. 每个场景的每张图仍然是独立可运行的入口脚本，符合"每个图有对应 py 代码"的设计要求；
3. 实验人员如需为某个场景的某张图定制特殊样式，只需修改对应的薄入口脚本，不影响其他场景。

#### 5.1.2 matplotlib 全局配置

```python
matplotlib.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "SimSun", "STSong", "serif"],
    "mathtext.fontset": "stix",          # LaTeX 数学字体（与 Times 风格一致）
    "axes.titlesize": 13,                # 图题字号
    "axes.labelsize": 11,                # 坐标轴标签字号
    "xtick.labelsize": 9,                # 刻度字号
    "ytick.labelsize": 9,
    "legend.fontsize": 9,                # 图例字号
    "figure.dpi": 150,                   # 屏幕显示 DPI
    "savefig.dpi": 300,                  # 保存 DPI（满足出版级别）
    "savefig.bbox": "tight",             # 自动裁剪白边
    "savefig.pad_inches": 0.1,           # 少量内边距
})
```

字体回退链为 `Times New Roman → SimSun(宋体) → STSong(华文宋体) → serif`，确保英文/数字优先使用 Times New Roman，中文 fallback 到宋体，数学公式使用 STIX 字体（与 Times 风格视觉一致）。

#### 5.1.3 TTC 阈值常量与风险色表

```python
TTC_SAFE = 5.0       # TTC > 5.0 → SAFE
TTC_WARNING = 3.0    # 3.0 < TTC ≤ 5.0 → WARNING
TTC_SLOWDOWN = 1.0   # 1.0 < TTC ≤ 3.0 → SLOWDOWN
TTC_EMERGENCY = 1.0  # TTC ≤ 1.0 → EMERGENCY

RISK_COLORS = {
    "SAFE":      "#4CAF50",   # green
    "WARNING":   "#FFC107",   # amber
    "SLOWDOWN":  "#FF9800",   # orange
    "EMERGENCY": "#F44336",   # red
    "RECOVERY":  "#2196F3",   # blue
}
```

这些常量为所有绘图函数提供统一的阈值标准和色彩映射，确保跨场景图表的视觉一致性。

#### 5.1.4 共用工具函数清单

| 函数 | 功能 | 被调用方 |
|------|------|---------|
| `get_raw_data_path(scene)` | 计算 `raw_data/scene_n/` 路径 | 数据加载 |
| `get_image_path(scene, filename)` | 计算 `Image/scene_n/` 输出路径（自动创建目录） | 所有绘图函数 |
| `get_image_path_root(filename)` | 计算 `Image/` 根输出路径（用于跨场景图） | `fusion_variance_plot`, `scene_1_vs_5_comparison` |
| `load_csv(scene, csv_name)` | 加载 CSV，返回 `(timestamps, data_dict)` | 旧版兼容（各入口脚本可选使用） |
| `_find_run_dir(scene)` | 自动查找场景下的第一个 `run_*` 子目录 | 内部使用 |
| `_load_csv_from_run(scene, csv_name)` | 从 `run_*` 子目录加载 CSV | 所有绘图函数 |
| `add_risk_backgrounds(ax, t_min, t_max)` | 用估计比例绘制风险背景色带（无 TTC 数据时的 fallback） | LiDAR/Radar/Motion 图 |
| `add_risk_backgrounds_by_ttc(ax, timestamps, ttc_values)` | 根据实际 TTC 数据精确定位风险区域并着色 | LiDAR/TTC 图 |
| `add_ttc_threshold_lines(ax, t_min, t_max)` | 绘制 TTC=5.0/3.0/1.0s 三条水平虚线 | TTC 图 |
| `format_time_axis(ax, axis)` | 格式化坐标轴刻度为 `%.3f`（毫秒级精度） | 所有绘图函数 |
| `save_figure(fig, path)` | 保存图像 @ 300dpi | 所有绘图函数 |
| `format_sci_notation(val, precision)` | 格式化浮点数为 LaTeX 科学计数法 | 需要时使用 |

#### 5.1.5 风险背景着色算法

`add_risk_backgrounds_by_ttc(ax, timestamps, ttc_values)` 是图表中风险区域可视化的核心技术：

1. 将每个时间点的 TTC 值按阈值判定为 0(SAFE), 1(WARNING), 2(SLOWDOWN), 3(EMERGENCY) 四级；
2. 使用 `np.diff` 查找相邻相同分类的连续区间，通过差分标记检测边界（从 0 变 1 为区间开始，从 1 变 0 为区间结束）；
3. 对每个连续区间调用 `ax.axvspan(t_start, t_end, alpha=0.10, color=...)` 绘制半透明色带。

这保证了风险背景精确反映实际 TTC 分类，而非基于时间比例估计。当 TTC 数据不可用时（如 LiDAR 图中可能尚未采集 TTC 数据），退而使用 `add_risk_backgrounds()` 基于时间比例做近似着色。

#### 5.1.6 图片跨场景标题映射

```python
SCENE_TITLES = {
    1: "Scene 1: Constant-speed Following (Straight Road)",
    2: "Scene 2: Static Broken-down Vehicle (Intersection Approach)",
    3: "Scene 3: Lead Vehicle Sudden Braking (Curve Approach)",
    4: "Scene 4: Cross-intersection Lateral Cut-in",
    5: "Scene 5: Dark Environment — Constant-speed Following",
}
```

所有绘图函数通过此字典获取场景标题，确保标题风格统一。

---

### 5.2 LiDAR 距离图脚本

**入口脚本**：`scene_n_lidarPlot.py`（调用 `plot_lidar(scene)`）

**数据来源**：`raw_data/scene_n/run_m/lidar.csv`

**图像规格**：
- 尺寸：`(10, 4.5)` 英寸
- 横轴：时间 (s)，毫秒精度
- 纵轴：距障碍物距离 (m)
- 曲线：蓝色实线（`#1976D2`，线宽 1.2）
- 背景：若有 `ttc.csv` 同目录存在，按实际 TTC 着色；否则按时间比例估计着色
- 方差标注：右上角 LaTeX 公式 `σ_lidar = X.XXXX`（标准差，白底圆角框）
- 网格：开启，30% 透明度
- 图例：右上角

**输出**：`Image/scene_n/scene_n_lidar.jpg`

**代码说明**：
- 首先调用 `_load_csv_from_run(scene, "lidar.csv")` 加载 LiDAR 距离数据；
- 检查 `distance_to_obstacle_m` 列是否存在，不存在则抛出 `KeyError`；
- 尝试从同场景的 `ttc.csv` 加载 TTC 数据用于精确风险背景着色；若文件不存在则 fallback 到 `add_risk_backgrounds`（时间比例估计）；
- 使用 `np.nanstd(distance)` 计算距离观测的标准差作为 σ_lidar 的近似值标注在图上。

---

### 5.3 雷达方位与速度图脚本

**入口脚本**：`scene_n_radarPlot.py`（调用 `plot_radar(scene)`）

**数据来源**：`raw_data/scene_n/run_m/radar.csv`

**图像规格**：
- 尺寸：`(14, 4.8)` 英寸，**左右子图布局** (`plt.subplots(1, 2)`)
- **左图 — 目标方位角**：
  - 横轴：时间 (s)
  - 纵轴：方位角 (°)，精确到小数点后两位
  - 曲线：紫色实线（`#7B1FA2`）
- **右图 — 目标径向速度**：
  - 横轴：时间 (s)
  - 纵轴：径向速度 (m/s)，精确到小数点后三位
  - 曲线：粉红实线（`#C2185B`）
  - 方差标注：`σ_radar = X.XXXX`（基于速度标准差）
- 全局图题：由 `fig.suptitle` 统一设置
- 图例：右上角（每个子图独立）

**输出**：`Image/scene_n/scene_n_radar.jpg`

**代码说明**：
- 子图排列使用 `plt.subplots(1, 2, figsize=(14, 4.8))`，1 行 2 列的左右布局；
- 左图和右图分别独立配置坐标轴、图例和标题；
- 方差 σ_radar 由 `np.nanstd(d.get("radial_velocity_ms", [0]))` 计算径向速度的标准差，标注在右子图的右上角；
- 若 `bearing_deg` 或 `radial_velocity_ms` 列不存在，对应子图留空但不报错。

---

### 5.4 TTC 变化图脚本

**入口脚本**：`scene_n_TTCPlot.py`（调用 `plot_ttc(scene)`）

**数据来源**：`raw_data/scene_n/run_m/ttc.csv`

**图像规格**：
- 尺寸：`(10, 5)` 英寸
- 横轴：时间 (s)
- 纵轴：TTC (s)，范围从 0 起
- 曲线：深灰实线（`#212121`，线宽 1.5）+ 浅灰填充区域（增加视觉厚度）
- 阈值线：三条水平灰色虚线，分别标注 TTC=5.0s, 3.0s, 1.0s
- 风险区域：按 `risk_level` 列的实际值标注阶段名称（SAFE/WARNING/SLOWDOWN/EMERGENCY），颜色与风险色表一致
- 背景：按实际 TTC 值着色的风险色带

**输出**：`Image/scene_n/scene_n_TTC.jpg`

**代码说明**：
- 若 `ttc_value_s` 列不存在，抛出 `KeyError`，因为此图必须以 TTC 数据为核心；
- `add_risk_backgrounds_by_ttc` 对待整个 TTC 序列进行四级分类并着色；
- `add_ttc_threshold_lines` 在右侧边缘标注阈值标签；
- `risk_level` 列的阶段切换点通过 `ax.annotate` 在曲线上方标注风险等级名称，颜色取自 `RISK_COLORS` 字典。

---

### 5.5 运动状态图脚本

**入口脚本**：`scene_n_motionPlot.py`（调用 `plot_motion(scene)`）

**数据来源**：`raw_data/scene_n/run_m/motion.csv`

**图像规格**：
- 尺寸：`(18, 4.8)` 英寸，**三栏左右布局** (`plt.subplots(1, 3)`)
- **左图 — 位移 (Displacement)**：
  - 纵轴：位移 (m)
  - 曲线：深绿实线（`#1B5E20`）
- **中图 — 速度 (Velocity)**：
  - 纵轴：速度 (m/s)，精确到小数点后三位
  - 曲线：深蓝实线（`#0D47A1`）
- **右图 — 加速度 (Acceleration)**：
  - 纵轴：加速度 (m/s²)，精确到小数点后三位
  - 曲线：深红实线（`#B71C1C`）
- 全局图题：`fig.suptitle`

**输出**：`Image/scene_n/scene_n_motion.jpg`

**代码说明**：
- 三栏布局 (`1×3`) 将位移、速度、加速度并列展示，便于横向对比运动状态随时间的变化；
- 每列独立配置坐标轴标签、图例和网格；
- 若某列数据（如 `acceleration_ms2`）不存在，该列留空但不报错；
- 这是每个场景必须生成的图表之一（依据 `Exp_design.md` §5 的要求）。

---

### 5.6 融合方差对比图脚本

**入口脚本**：`fusion_variance_plot.py`（调用 `plot_fusion_variance(scenes=(1,2,3,4,5))`）

**数据来源**：所有 5 个场景的 `raw_data/scene_n/run_m/fusion.csv`

**图像规格**：
- 尺寸：`(12, 5)` 英寸
- 横轴：时间 (s)
- 纵轴：融合方差 σ²_fused
- 曲线：5 条不同颜色的曲线分别代表 5 个场景
- 图例：自动选择最佳位置

**输出**：`Image/fusion_variance_comparison.jpg`

**代码说明**：
- 遍历 `scenes=(1,2,3,4,5)`，对每个场景尝试加载 `fusion.csv`；
- 若某场景数据不可用，在控制台打印 `[skip]` 并继续下一场景（容错设计）；
- 颜色循环使用 5 种区分度高的色值：蓝 `#1976D2`、紫 `#7B1FA2`、粉 `#C2185B`、橙 `#E64A19`、青 `#00796B`。

---

### 5.7 有光/暗环境对照图脚本

**入口脚本**：`scene_1_vs_scene_5_comparison.py`（调用 `plot_scene_1_vs_5()`）

**数据来源**：场景一和场景五的 `ttc.csv` 和 `fusion.csv`

**图像规格**：
- 尺寸：`(14, 9)` 英寸，**2×2 子图布局**
- **第 1 行（场景一 — 有光）**：
  - 左：TTC 图（蓝色 `#1976D2`）+ 风险背景 + 阈值线
  - 右：融合方差图（蓝色 `#1976D2`）
- **第 2 行（场景五 — 暗环境）**：
  - 左：TTC 图（橙色 `#F57C00`）+ 风险背景 + 阈值线
  - 右：融合方差图（橙色 `#F57C00`）
- 全局标题：强调 "Light vs Dark — Fusion Algorithm Adaptability"

**输出**：`Image/scene_1_vs_scene_5_comparison.jpg`

**代码说明**：
- 使用双层循环：`for row, scene in enumerate([1, 5])` 依次处理有光和暗环境场景；
- 颜色固定分配：场景一（有光）蓝色，场景五（暗环境）橙色，形成鲜明对比；
- TTC 子图复用 `add_risk_backgrounds_by_ttc` 和 `add_ttc_threshold_lines`，保持与其他 TTC 图一致的视觉风格；
- 此图旨在验证算法在视觉传感器失效时是否仍能维持 TTC 计算精度和融合稳定性（预期：暗环境下 σ²_fused 因相机退出融合而略有增大，但仍保持在可接受范围内）。

---

## 6. 图像输出模块 — Image

`Image/` 目录是所有绘图脚本的默认输出位置。按场景分子目录存储，跨场景对比图放置在 `Image/` 根目录。

| 目录/文件 | 来源脚本 | 说明 |
|----------|---------|------|
| `Image/scene_n/scene_n_lidar.jpg` | `scene_n_lidarPlot.py` | LiDAR 距离-时间曲线 |
| `Image/scene_n/scene_n_radar.jpg` | `scene_n_radarPlot.py` | 雷达方位角+径向速度（左右） |
| `Image/scene_n/scene_n_TTC.jpg` | `scene_n_TTCPlot.py` | TTC 变化 + 阈值线 + 风险着色 |
| `Image/scene_n/scene_n_motion.jpg` | `scene_n_motionPlot.py` | 位移\|速度\|加速度（三栏） |
| `Image/fusion_variance_comparison.jpg` | `fusion_variance_plot.py` | 5 场景融合方差对比 |
| `Image/scene_1_vs_scene_5_comparison.jpg` | `scene_1_vs_scene_5_comparison.py` | 有光/暗环境对照 |

所有图像均以 **300 dpi** 保存，满足论文/报告级别的印刷质量要求。

---

## 7. 评价报告模块 — evaluation

每个场景对应一个 Markdown 格式的评价报告模板，文件位于 `evaluation/scene_n_evaluation.md`。模板包含以下固定章节：

### 7.1 TTC 分级准确性

一个表格，以风险等级（SAFE/WARNING/SLOWDOWN/EMERGENCY）为行，列出设定 TTC 范围、实测 TTC 范围、是否符合预期。其中设定范围使用 LaTeX 数学公式（如 `$> 5.0\ \mathrm{s}$`）。实验完成后，根据 `TTC 变化图` 和 `ttc.csv` 的原始数据填入实测值。

### 7.2 停车距离

记录关键时间节点和物理量：
- 预警信号发出时刻（TTC 首次 ≤ 5.0s 的时间戳）
- 完全停车时刻（速度降至 0 的时间戳）
- 预警至停车行驶距离（从以上两时刻配合位移图计算）
- 是否在碰撞前成功停车（是/否）

### 7.3 传感器方差分析

以风险阶段为行、各传感器及融合方差为列的表格。σ² 值来源于融合节点在对应阶段的噪声模型输出。至少列出 σ²_cam、σ²_lidar、σ²_radar 和 σ²_fused 四项。

### 7.4 综合结论

三个要点的定性评估：
- 融合算法在本场景下的表现
- TTC 决策有效性
- 是否达到设计预期

### 7.5 附图索引

列出本场景所有相关图像的路径，方便查阅。

---

## 8. 启动脚本 collect.sh

`collect.sh` 位于项目根目录（与 `RUN.sh` 并列），是数据采集的单一入口。

### 8.1 参数说明

```bash
./collect.sh --scene <1-5> --run <run_id> [--duration <sec>] [--odom_topic <name>]
```

| 参数 | 必选 | 默认 | 说明 |
|------|:--:|------|------|
| `--scene <1-5>` | 是 | — | 场景编号 |
| `--run <id>` | 是 | — | 运行编号（同一场景多次采集） |
| `--duration <sec>` | 否 | 120 | 采集时长（秒） |
| `--odom_topic <name>` | 否 | `/odom` | 里程计话题名 |

### 8.2 脚本行为步骤

1. **参数解析**：`while/case` 循环解析所有 `--key value` 对，未知参数报错退出；
2. **参数校验**：`--scene` 必须在 1-5 范围内，`--scene` 和 `--run` 不能为空；
3. **ROS2 环境 source**：若 `ROS_DISTRO` 未设置，依次尝试 `/opt/ros/humble/setup.bash` 和 `/opt/ros/jazzy/setup.bash`；
4. **工作空间 source**：若项目根目录下存在 `install/setup.bash`，自动 source；
5. **启动采集节点**：以 `python3` 运行 `data_collector.py`，传入解析后的参数；
6. **等待完成**：采集节点在 duration 秒后自动退出，或 Ctrl+C 手动中断；
7. **输出路径确认**：打印数据保存位置。

### 8.3 使用示例

```bash
# 场景一，第 1 次运行，默认 120 秒
./collect.sh --scene 1 --run 1

# 场景三，第 2 次运行，180 秒
./collect.sh --scene 3 --run 2 --duration 180

# 场景五（暗环境），使用 TurtleBot4 特定里程计话题
./collect.sh --scene 5 --run 1 --odom_topic /turtlebot4/odom
```

---

## 9. 完整使用流程

### 9.1 前置条件

1. ROS2 环境已安装并配置（Humble 或 Jazzy）；
2. ADAS Fusion 项目已编译（`colcon build`），`install/setup.bash` 存在；
3. Python 依赖已安装：`numpy`, `matplotlib`, `rclpy`, `sensor_msgs`, `nav_msgs`, `std_msgs`, `cv_bridge`, `opencv-python`；
4. TurtleBot4 平台及相关传感器已上电并联调；
5. 沙盘环境按场景要求布置完毕（道路标线、障碍物/第二台小车就位）；
6. （场景五）环境光源已关闭。

### 9.2 数据采集

**步骤 1**：启动 ADAS Fusion 系统

```bash
cd /path/to/ADAS_Fusion
./RUN.sh
```

**步骤 2**：在另一终端中，启动数据采集

```bash
cd /path/to/ADAS_Fusion
./collect.sh --scene 1 --run 1
```

**步骤 3**：操作 TurtleBot4 按场景要求行驶（手柄控制或自动轨迹跟踪）。采集程序会在 120 秒后自动停止（或用 Ctrl+C 提前终止）。控制台输出类似：

```
══════════════════════════════════════════════
  ADAS Fusion — Experiment Data Collection
  Scene:       1
  Run ID:      1
  Duration:    120s
  Odom topic:  /odom
  Output:      Experiment/Data_collection/raw_data/scene_1/run_1/
══════════════════════════════════════════════
[data_collector]: Output directory: .../raw_data/scene_1/run_1
[data_collector]: DataCollector initialized — recording at 10 Hz
[data_collector]: Keyframe saved: scene_1_phase_2_WARNING.jpg
[data_collector]: Keyframe saved: scene_1_phase_3_SLOWDOWN.jpg
[data_collector]: Keyframe saved: scene_1_phase_4_EMERGENCY.jpg
[data_collector]: Duration (120s) reached — shutting down.
```

**步骤 4**：重复步骤 2-3 完成其余场景的数据采集。同一场景可多次运行（`--run 1`, `--run 2`, ...），绘图脚本默认使用第一个 `run_*` 目录。

### 9.3 生成图表

采集完成后，进入 `plot_py/` 目录，按需运行绘图脚本：

```bash
cd Expriment/plot_py

# 生成场景一的所有图表
python3 scene_1_lidarPlot.py
python3 scene_1_radarPlot.py
python3 scene_1_TTCPlot.py
python3 scene_1_motionPlot.py

# 生成场景二至场景五的所有图表（同理）
# ...

# 生成跨场景对比图
python3 fusion_variance_plot.py
python3 scene_1_vs_scene_5_comparison.py
```

每个脚本运行后在控制台打印输出路径：

```
Plotting LiDAR distance — Scene 1 ...
  Saved: .../Image/scene_1/scene_1_lidar.jpg
```

生成的 JPG 文件位于 `Image/scene_n/` 目录下。

### 9.4 填写评价报告

1. 打开 `evaluation/scene_n_evaluation.md`；
2. 参照对应场景的原始 CSV 数据和生成的图表，填写各章节的数据表格和分析结论；
3. 将 `YYYY-MM-DD` 替换为实验日期，`run_X` 替换为实际运行编号；
4. 在"综合结论"部分撰写定性评估；
5. 填写评价人和审核人姓名及日期。

---

## 10. 数据与图像流转全景

```
┌────────────────────────────────────────────────────────────────────┐
│                        一次完整实验                                  │
│                                                                    │
│   1. 布置沙盘 → 2. 启动融合系统 → 3. collect.sh 采集数据           │
│       ↓                                                            │
│   raw_data/scene_n/run_m/                                          │
│   ├── motion.csv     (位移、速度、加速度)                            │
│   ├── lidar.csv      (LiDAR 前方距离)                               │
│   ├── radar.csv      (雷达方位、距离、径向速度)                       │
│   ├── fusion.csv     (融合位置、速度、方差)                           │
│   ├── ttc.csv        (TTC 值、风险等级)                              │
│   └── camera_keyframes/  (风险切换时刻的 RGB 关键帧)                  │
│       ↓                                                            │
│   4. 运行 plot_py/ 中各脚本                                         │
│       ↓                                                            │
│   Image/scene_n/                                                    │
│   ├── scene_n_lidar.jpg    (LiDAR 距离-时间曲线)                     │
│   ├── scene_n_radar.jpg    (雷达方位+速度，左右子图)                  │
│   ├── scene_n_TTC.jpg      (TTC 变化+阈值线+风险着色)                │
│   └── scene_n_motion.jpg   (位移|速度|加速度，三栏子图)              │
│       +                                                            │
│   Image/fusion_variance_comparison.jpg (5场景方差对比)               │
│   Image/scene_1_vs_scene_5_comparison.jpg (有光/暗对照)              │
│       ↓                                                            │
│   5. 参照图表 + 原始数据 → 填写 evaluation/scene_n_evaluation.md    │
│       ↓                                                            │
│   6. 完成实验报告                                                    │
└────────────────────────────────────────────────────────────────────┘
```

---

## 附录 A：绘图脚本命名规范速查表

| 脚本文件 | 调用函数 | 输入 CSV | 输出 JPG |
|---------|---------|---------|---------|
| `scene_n_lidarPlot.py` | `plot_lidar(n)` | `lidar.csv` | `scene_n_lidar.jpg` |
| `scene_n_radarPlot.py` | `plot_radar(n)` | `radar.csv` | `scene_n_radar.jpg` |
| `scene_n_TTCPlot.py` | `plot_ttc(n)` | `ttc.csv` | `scene_n_TTC.jpg` |
| `scene_n_motionPlot.py` | `plot_motion(n)` | `motion.csv` | `scene_n_motion.jpg` |
| `fusion_variance_plot.py` | `plot_fusion_variance()` | 全部 `fusion.csv` | `fusion_variance_comparison.jpg` |
| `scene_1_vs_scene_5_comparison.py` | `plot_scene_1_vs_5()` | 场景1+5 的 `ttc.csv` + `fusion.csv` | `scene_1_vs_scene_5_comparison.jpg` |

其中 `n` = 1, 2, 3, 4, 5 对应五个实验场景。

## 附录 B：精度规范速查表

| 数据项 | 写入精度 | 显示精度 | 说明 |
|-------|---------|---------|------|
| 时间 | `.6f` | `.3f` | 存储微秒，显示毫秒 |
| 距离/位移 | `.6f` 或 `.4f` | 与写入一致 | 场景尺度为 cm 级 |
| 速度 | `.3f` | `.3f` | m/s |
| 方位角 | `.2f` | `.2f` | ° |
| 加速度 | `.3f` | `.3f` | m/s² |
| 融合方差 | `.6f` | `.6f` | 数量级可能很小 |
| 图像 DPI | — | 300 | 满足出版要求 |

---

*设计: Moneve | 日期: 2026-05-08 | 版本: v1.0*
