# mmWRadar_ros 需求文档

## 1. 概述

为觅感科技 MS60-3015S80M4-3V3-B-NLS-1T2R-S7136H 60GHz 毫米波雷达模块开发 ROS 2 Humble 驱动。该雷达用于户外低速车盲区监测(BSD)和后侧方靠近预警(RCW)，可同时探测最多8个目标，输出距离、速度、角度信息。

**目标平台:** Jetson NANO, ROS 2 Humble, Python  
**物理连接:** 雷达 UART -> CH340 USB转串口 -> Jetson NANO USB口  
**默认串口参数:** 921600bps, 8N1

## 2. 硬件参数摘要

| 参数 | 数值 |
|------|------|
| 工作频段 | 59~64 GHz |
| 调制方式 | FMCW |
| 水平视角 | ±40° |
| 俯仰视角 | ±30° |
| 最大探测目标数 | ≤8 |
| 测距范围(汽车) | 0.5~50 m |
| 测速范围 | 6~90 km/h |
| 刷新周期 | 100 ms |
| 工作电压 | 3.3V |
| 平均电流 | 14 mA |
| 通信接口 | UART (TX/RX) |

## 3. 通信协议

基于 AT6010 SOC HCI Protocol V1.4。

### 3.1 命令模式 (配置/控制)

用于驱动初始化时配置雷达参数。

- **发送帧:** `HEAD(0x58) | CMD | PARAM_LEN | PARAM[0..n] | CHECK`
- **回复帧:** `HEAD(0x59) | CMD | PARAM_LEN | PARAM[0..n] | CHECK`
- **校验:** CHECK = sum of all preceding bytes (低8位)

### 3.2 主动上报模式 (数据接收)

雷达配置完成后以数据流形式上报检测结果。

- **帧格式:** `HEAD(0x5A) | LEN | TYPE | PAYLOAD(F0..FM) | CHECK`
- **校验:** CHECK = sum(HEAD + LEN + TYPE + PAYLOAD) 低8位
- **数据类型:** TYPE=7 (BSD 目标检测信息)

### 3.3 BSD 数据帧结构 (TYPE=7)

```
Payload:
  u16 obj_num          # 目标数量 (最大8)
  u16 reserved         # 保留
  bsd_obj_info_t obj[obj_num]  # 仅上报检测到的目标, 变长

bsd_obj_info_t (每目标4字节):
  s8 range_val         # 距离, 单位: m
  s8 angle_val         # 角度, 单位: °
  s8 velo_val          # 速度, 单位: m/s
  s8 objId             # 目标ID
```

### 3.4 配置命令列表

驱动需支持以下配置命令:

| 功能 | 指令码 | 说明 |
|------|--------|------|
| 打开/关闭雷达感应 | 0xD1 | para: 0x01=开, 0x00=关 |
| 获取雷达感应状态 | 0xD0 | - |
| 设置感应等级 | 0x02 | para: 0~15 |
| 获取感应等级 | 0x03 | - |
| 设置运动检测最远距离 | 0xD2 | para: 16-bit, 单位 cm |
| 设置运动检测最近距离 | 0x34 | para: 16-bit, 单位 cm |
| 设置运动检测灵敏度 | 0x35 | para: 0~10, 越小越灵敏 |
| 设置微动检测最远距离 | 0x36 | para: 16-bit, 单位 cm |
| 设置微动检测最近距离 | 0x37 | para: 16-bit, 单位 cm |
| 设置微动检测灵敏度 | 0x38 | para: 0~10 |
| 设置呼吸检测最远距离 | 0x39 | para: 16-bit, 单位 cm |
| 设置呼吸检测最近距离 | 0x3A | para: 16-bit, 单位 cm |
| 设置呼吸检测灵敏度 | 0x3B | para: 0~10 |
| 获取感应配置 | 0x33 | 返回运动/微动/呼吸检测的距离和灵敏度 |
| 获取算法边界值 | 0x32 | 返回检测边界(不可修改) |
| 获取软硬件版本 | 0xFE | 返回版本信息 |
| 波特率切换 | 0x19 | para: 32-bit波特率值(小端) |
| 保存设置至Flash | 0x08 | para: 0x01=保存, 0x00=不保存 |
| System Reset | 0x13 | 系统复位 |

## 4. 软件架构

```
mmWRadar_ros/
├── mmw_radar_msgs/        # 自定义 ROS 2 消息包
│   ├── msg/
│   │   ├── RadarTarget.msg        # 单个目标 (id, range, angle, velocity)
│   │   └── RadarTargetArray.msg   # 目标数组 (header, targets[])
│   ├── CMakeLists.txt
│   └── package.xml
│
├── mmw_radar_driver/      # 驱动节点包
│   ├── mmw_radar_driver/
│   │   ├── __init__.py
│   │   ├── radar_node.py          # ROS 2 主节点
│   │   ├── serial_driver.py       # 串口通信与协议解析
│   │   ├── protocol.py            # AT6010 协议封装 (命令帧构建/回复帧解析)
│   │   └── radar_config.py        # 雷达参数配置管理
│   ├── launch/
│   │   └── radar.launch.py        # 启动文件
│   ├── config/
│   │   └── radar_params.yaml      # 默认参数配置
│   ├── CMakeLists.txt
│   └── package.xml
│
└── README.md
```

## 5. ROS 2 节点设计

### 5.1 节点名称: `mmw_radar_node`

### 5.2 发布话题

| 话题 | 消息类型 | 描述 |
|------|---------|------|
| `/radar/targets` | `mmw_radar_msgs/RadarTargetArray` | 探测到的目标数组 (主要的输出) |

### 5.3 自定义消息定义

```
# RadarTarget.msg
int8 id                  # 目标ID
float32 range            # 距离, 单位: m
float32 angle            # 水平角度, 单位: ° (正值=左侧, 负值=右侧)
float32 velocity         # 径向速度, 单位: m/s (正值=靠近, 负值=远离)

# RadarTargetArray.msg
std_msgs/Header header   # 时间戳和frame_id
RadarTarget[] targets    # 目标列表
```

### 5.4 参数 (全参数配置)

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `serial_port` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `baud_rate` | int | 921600 | 串口波特率 |
| `frame_id` | string | `radar_link` | TF 坐标系名称 |
| `radar_enabled` | bool | true | 雷达感应开关 |
| `detection_level` | int | 15 (0~15) | 感应等级档位 |
| `mot_det_max_range` | int | 5000 (cm) | 运动检测最远距离 |
| `mot_det_min_range` | int | 50 (cm) | 运动检测最近距离 |
| `mot_det_sensitivity` | int | 5 (0~10) | 运动检测灵敏度 |
| `micro_det_max_range` | int | 3000 (cm) | 微动检测最远距离 |
| `micro_det_min_range` | int | 50 (cm) | 微动检测最近距离 |
| `micro_det_sensitivity` | int | 5 (0~10) | 微动检测灵敏度 |
| `bhr_det_max_range` | int | 2000 (cm) | 呼吸检测最远距离 |
| `bhr_det_min_range` | int | 50 (cm) | 呼吸检测最近距离 |
| `bhr_det_sensitivity` | int | 5 (0~10) | 呼吸检测灵敏度 |
| `publish_rate` | float | 10.0 (Hz) | 发布频率(雷达刷新周期100ms) |
| `auto_standby` | bool | true | 无订阅者时暂停处理 |

### 5.5 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/radar/reset` | `std_srvs/Empty` | 系统复位 |
| `/radar/save_settings` | `std_srvs/Empty` | 保存参数至Flash |

## 6. 功能需求

### 6.1 初始化流程

1. 打开串口 (基于 `serial_port` 和 `baud_rate` 参数)
2. 发送配置命令序列:
   - 打开雷达感应 (`0xD1 01`)
   - 设置感应等级 (`0x02`)
   - 设置运动/微动/呼吸检测距离和灵敏度
   - 若参数值与默认不同, 逐项配置
3. 进入数据接收循环, 解析 0x5A 主动上报帧 (TYPE=7)

### 6.2 数据解析

- 从串口字节流中按 0x5A 帧头同步
- 校验 CHECK 字段
- 解析 TYPE=7 BSD 目标数据
- 转换为 ROS 2 消息发布

### 6.3 健康管理

- **串口断连检测:** 捕获串口异常, 通过 ROS 2 日志报警
- **自动重连:** 断连后按指数退避策略重试 (1s/2s/4s...最大30s)
- **订阅者计数:** 当 `auto_standby=true` 时:
  - 无订阅者 → 停止解析/发布, 可选关闭雷达感应以降低功耗
  - 有订阅者接入 → 恢复工作
- **帧同步保护:** 连续校验失败超过阈值(如100次)时告警

### 6.4 动态参数更新

- 所有配置参数支持 `rclpy.parameter.ParameterEventHandler` 运行时更新
- 参数变更时自动发送对应配置命令至雷达
- 部分参数(如 `serial_port`, `baud_rate`)需重启节点生效

### 6.5 错误处理

- 串口打开失败 → 日志报错并重试
- 命令发送后超时无回复 → 重试3次, 仍失败则告警
- 数据帧校验失败 → 丢弃该帧, 记录warning(可节流)
- 缓冲区溢出 → 清空并重新同步

## 7. 启动文件

`radar.launch.py`:
- 加载 `radar_params.yaml` 默认配置
- 启动 `mmw_radar_node`
- 可选: 包含 RViz 可视化配置

## 8. 依赖

- `rclpy`
- `std_msgs`
- `sensor_msgs`
- `pyserial` (串口通信)
- 自定义消息包 `mmw_radar_msgs`

## 9. 注意事项

- 雷达默认仅上报靠近目标 (速度>6km/h), 不支持远离目标探测
- 角度极性: 需明确正负角度对应车辆左右侧的定义
- CH340 芯片在 Jetson NANO 上通常识别为 `/dev/ttyUSB0`, 若系统有其他USB串口设备需区分
- 需安装于金属物上方净空区域, 避免干扰
- 多雷达安装需保持1m以上间距
