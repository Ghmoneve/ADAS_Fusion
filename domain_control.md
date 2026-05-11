# 底盘控制 (Domain Control)

## 硬件平台

- **机器人**: Turtlebot4 (Standard 型号)
- **底盘**: iRobot Create3
- **计算平台**: NVIDIA Jetson (arm64)
- **系统**: Ubuntu 22.04 + ROS2 Humble

## 控制架构

```
键盘/控制端                    服务器                      机器人(ROS2)                  底盘硬件
┌──────────┐   WebSocket    ┌──────────┐   WebSocket    ┌──────────────┐   /cmd_vel    ┌──────────────┐
│ keyboard │  ────────────► │  server  │  ────────────► │ robot_client │  ──────────► │ diff_drive   │
│ _control │   /control     │  :8765   │   /robot       │  (ROS2 Node) │   Twist      │ _controller  │
│   .py    │               │   .py    │               │              │             │              │
└──────────┘               └──────────┘               └──────────────┘             └──────┬───────┘
                                                                                          │
                                                                                     ┌────▼───────┐
                                                                                     │  Create3   │
                                                                                     │  Motors    │
                                                                                     └────────────┘
```

## 三层通信

### 1. 控制端 → 服务器 (WebSocket `/control`)

控制端（键盘、手柄等）通过 WebSocket 连接服务器的 `/control` 路径，发送 JSON 指令：

```json
{"linear_x": 0.5, "angular_z": 0.0}
```

- **linear_x**: 线速度 (m/s)，正=前进，负=后退
- **angular_z**: 角速度 (rad/s)，正=左转，负=右转

### 2. 服务器 → 机器人 (WebSocket `/robot`)

服务器广播指令到所有已连接的机器人，消息带时间戳：

```json
{
  "timestamp": "2025-01-01T00:00:00.000Z",
  "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
  "angular": {"x": 0.0, "y": 0.0, "z": 0.5}
}
```

### 3. 机器人 → 底盘 (ROS2 `/cmd_vel`)

`robot_client.py` 将 WebSocket 指令转为 ROS2 `geometry_msgs/Twist` 消息，发布到 `/cmd_vel` topic。

```python
twist = Twist()
twist.linear.x = linear_x
twist.angular.z = angular_z
publisher.publish(twist)
```

## 底盘参数 (Create3)

| 参数 | 值 |
|------|-----|
| 轮距 (wheel_separation) | 0.233 m |
| 轮半径 (wheel_radius) | 0.03575 m |
| 最大线速度 | 0.46 m/s（安全限制下实际 0.306 m/s）|
| 最大角速度 | 1.9 rad/s |
| 最大线加速度 | 0.9 m/s² |
| 最大角加速度 | 7.725 rad/s² |
| cmd_vel 超时 | 0.5 s（超时自动停止）|

## 安全机制

1. **指令超时**: 超过 0.5s 未收到 `/cmd_vel`，diff_drive_controller 自动停止
2. **心跳检测**: 每 5s 发送心跳，3 次超时判定断连
3. **WebSocket 安全停止**: `robot_client.py` 超过 2s 未收到新指令，主动发送零速
4. **重连机制**: 指数退避，初始 3s，最大 60s

## 文件位置 (远程机器)

| 文件 | 路径 | 作用 |
|------|------|------|
| robot_server.py | `~/websocket_control/` | WebSocket 服务器，转发控制指令 |
| robot_client.py | `~/websocket_control/` | ROS2 节点，接收指令并发布到 `/cmd_vel` |
| keyboard_control.py | `~/websocket_control/` | 键盘控制端，WASD 操控 |

## 键盘控制键位

| 按键 | 功能 |
|------|------|
| W | 前进 |
| S | 后退 |
| A | 左转 |
| D | 右转 |
| 空格 | 停止 |
| Q | 退出 |
| 1/2/3 | 低速(0.2)/中速(0.5)/高速(1.0) |
| +/- | 增减速度 0.1 |

## 直接控制底盘的方式

如果不需要 WebSocket 转发，可以**直接在 ROS2 节点中发布** `/cmd_vel` topic：

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DirectController(Node):
    def __init__(self):
        super().__init__('direct_controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def move(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.pub.publish(twist)

    def stop(self):
        self.move(0.0, 0.0)
```

发布到 `/cmd_vel` 即可控制底盘，`diff_drive_controller` 会自动处理运动学解算（将线速度/角速度转为左右轮转速）。

## 启动命令

```bash
# 远程机器上
# 启动完整 Turtlebot4 系统（包括底盘驱动）
ros2 launch turtlebot4_bringup standard.launch.py

# 启动 WebSocket 控制系统
cd ~/websocket_control
python3 robot_server.py &    # 启动服务器
python3 robot_client.py      # 启动机器人客户端（连接服务器）

# 控制端（可在任意机器）
python3 keyboard_control.py   # 需要修改 server_ip 指向服务器
```
