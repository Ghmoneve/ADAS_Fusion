#!/usr/bin/env python3
"""
serial_bridge.py -- Jetson ↔ STM32 串口桥接节点
================================================

功能：
  - 订阅 /cmd_vel (geometry_msgs/Twist)
  - 将速度指令转换为 STM32 串口协议帧
  - 通过 UART (USART3) 发送给 STM32F407 底盘控制器
  - 可选：接收 STM32 上报的里程计/IMU/电池数据并发布

硬件连接：
  Jetson ORIN NX (USART3: /dev/ttyTHS2)
      ↓ 3.3V TTL UART
  STM32F407VGT6 (USART3)

串口协议 (参考 智能导航教学实验平台 用户手册):

  帧格式:
  ┌──────────┬────────┬──────────┬──────────┬──────────┐
  │ Header   │ Cmd ID │ Data Len │ Data     │ Checksum │
  │ 2 bytes  │ 1 byte │ 1 byte   │ N bytes  │ 1 byte   │
  ├──────────┼────────┼──────────┼──────────┼──────────┤
  │ 0x55 0xAA│  ...   │   ...    │   ...    │   ...    │
  └──────────┴────────┴──────────┴──────────┴──────────┘

  校验和 = (Header[0] XOR Header[1] XOR CmdID XOR DataLen XOR Data[0]... XOR Data[N-1])

  速度控制指令 (Cmd ID = 0x01):
    Data[0:2] = v_linear  (int16, 单位: mm/s,  范围: -5000 ~ +5000)
    Data[2:4] = v_angular  (int16, 单位: 0.001 rad/s, 范围: -2000 ~ +2000)

  示例: 发送 v=0.2m/s, ω=0.1rad/s
    v_linear  = 200 (mm/s)   → 0x00 0xC8
    v_angular = 100 (0.001rad/s) → 0x00 0x64
    帧: 55 AA 01 04 00 C8 00 64 CS

参数:
  serial_port:  串口设备路径 (默认 /dev/ttyTHS2)
  baud_rate:    波特率 (默认 115200)
  cmd_vel_topic: 订阅的 cmd_vel 话题 (默认 /cmd_vel)
"""

import math
import struct
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    import serial
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False
    serial = None


# ==============================================================================
# STM32 协议封装
# ==============================================================================

class Stm32Protocol:
    """STM32 串口协议帧编码/解码。"""

    HEADER = bytes([0x55, 0xAA])

    CMD_VELOCITY = 0x01       # 速度控制
    CMD_SERVO = 0x02          # 舵机控制
    CMD_LED = 0x03            # LED/蜂鸣器
    CMD_READ_ENCODER = 0x10   # 读取编码器 (里程计)
    CMD_READ_IMU = 0x11       # 读取 IMU
    CMD_READ_BATTERY = 0x12   # 读取电池电压

    RESP_ENCODER = 0x90       # 编码器数据上报
    RESP_IMU = 0x91           # IMU 数据上报
    RESP_BATTERY = 0x92       # 电池数据上报

    @classmethod
    def checksum(cls, data: bytes) -> int:
        """计算 XOR 校验和。"""
        result = 0
        for b in data:
            result ^= b
        return result & 0xFF

    @classmethod
    def encode_velocity(cls, v_linear: float, v_angular: float) -> bytes:
        """
        编码速度控制帧。

        Args:
            v_linear:  线速度 (m/s), 转换为 mm/s 后发送
            v_angular: 角速度 (rad/s), 转换为 0.001 rad/s 后发送

        Returns:
            完整的协议帧 bytes
        """
        # 单位转换 & 限幅
        vl = max(-5000, min(5000, int(v_linear * 1000)))     # m/s → mm/s
        va = max(-2000, min(2000, int(v_angular * 1000)))    # rad/s → 0.001 rad/s

        payload = struct.pack('>hh', vl, va)  # 大端序 int16 x2

        frame = bytearray()
        frame.extend(cls.HEADER)                          # header
        frame.append(cls.CMD_VELOCITY)                    # cmd
        frame.append(len(payload))                        # len
        frame.extend(payload)                             # data
        frame.append(cls.checksum(frame))                 # checksum

        return bytes(frame)

    @classmethod
    def decode_encoder(cls, data: bytes):
        """解码编码器数据帧 → (left_ticks, right_ticks, timestamp_ms)。"""
        if len(data) < 10:
            return None
        try:
            left = struct.unpack('>i', data[:4])[0]
            right = struct.unpack('>i', data[4:8])[0]
            ts = struct.unpack('>H', data[8:10])[0]
            return (left, right, ts)
        except struct.error:
            return None

    @classmethod
    def decode_imu(cls, data: bytes):
        """解码 IMU 数据帧 → (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)。"""
        if len(data) < 12:
            return None
        try:
            vals = struct.unpack('>hhhhhh', data[:12])
            return vals
        except struct.error:
            return None

    @classmethod
    def decode_battery(cls, data: bytes):
        """解码电池电压 → voltage_mV (uint16)。"""
        if len(data) < 2:
            return None
        try:
            return struct.unpack('>H', data[:2])[0]
        except struct.error:
            return None


# ==============================================================================
# 串口桥接节点
# ==============================================================================

class SerialBridge(Node):
    """Jetson ↔ STM32 串口桥接节点。"""

    def __init__(self):
        super().__init__('serial_bridge')

        if not HAS_PYSERIAL:
            self.get_logger().fatal(
                'pyserial not installed. Run: pip3 install pyserial')
            raise RuntimeError('pyserial required')

        # ---- 参数 ----
        self.declare_parameter('serial_port', '/dev/ttyTHS2')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('wheel_base', 0.2)        # 轮距 (m)
        self.declare_parameter('ticks_per_meter', 2000)  # 编码器每米脉冲数
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('reconnect_interval', 1.0)
        self.declare_parameter('watchdog_timeout', 0.5)  # cmd_vel 超时自动停车

        # ---- 串口 ----
        self._port_name = self.get_parameter('serial_port').value
        self._baud = self.get_parameter('baud_rate').value
        self._ser: 'serial.Serial | None' = None
        self._ser_lock = threading.Lock()

        # ---- 状态 ----
        self._last_cmd_vel_time = time.time()
        self._watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self._last_v = 0.0
        self._last_w = 0.0

        # ---- /odom 发布 ----
        if self.get_parameter('publish_odom').value:
            odom_topic = self.get_parameter('odom_topic').value
            self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
            self._odom_x = 0.0
            self._odom_y = 0.0
            self._odom_yaw = 0.0
            self._last_odom_time = time.time()
        else:
            self._odom_pub = None

        # ---- /cmd_vel 订阅 ----
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self._cmd_sub = self.create_subscription(
            Twist, cmd_topic, self._cmd_vel_cb, 10)

        # ---- 定时器 ----
        self._timer = self.create_timer(0.05, self._control_loop)   # 20Hz
        self._reconnect_timer = self.create_timer(
            self.get_parameter('reconnect_interval').value,
            self._reconnect_check)

        # ---- 初始连接 ----
        self._connect()

        self.get_logger().info(
            f'SerialBridge: {self._port_name} @ {self._baud} baud')

    # ---- 串口管理 ----

    def _connect(self) -> bool:
        """打开串口。"""
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                return True
            try:
                self._ser = serial.Serial(
                    port=self._port_name,
                    baudrate=self._baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.01,
                )
                self.get_logger().info(f'Serial {self._port_name} opened')
                return True
            except serial.SerialException as e:
                self.get_logger().error(
                    f'Cannot open {self._port_name}: {e}',
                    throttle_duration_sec=5.0)
                return False

    def _reconnect_check(self):
        """周期性检查串口连接状态。"""
        if self._ser is None or not self._ser.is_open:
            self._connect()

    def _send_frame(self, frame: bytes):
        """发送一帧数据到 STM32。"""
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                return False
            try:
                self._ser.write(frame)
                self._ser.flush()
                return True
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}',
                                        throttle_duration_sec=2.0)
                try:
                    self._ser.close()
                except Exception:
                    pass
                return False

    # ---- 速度指令回調 ----

    def _cmd_vel_cb(self, msg: Twist):
        self._last_v = msg.linear.x
        self._last_w = msg.angular.z
        self._last_cmd_vel_time = time.time()

    # ---- 主控制循环 ----

    def _control_loop(self):
        """
        20Hz 控制循环:
          - 编码 / 发送速度控制帧
          - 看门狗: 若超时未收到 cmd_vel → 发送停车指令
          - 尝试接收 STM32 上报数据
        """
        now = time.time()

        # 看门狗: 超时后自动停车
        if now - self._last_cmd_vel_time > self._watchdog_timeout:
            v, w = 0.0, 0.0
        else:
            v, w = self._last_v, self._last_w

        # 发布里程计 (cmd_vel 积分, 近似)
        if self._odom_pub is not None:
            dt = now - self._last_odom_time
            self._last_odom_time = now
            if dt > 0.0 and dt < 0.5:
                self._odom_x += v * math.cos(self._odom_yaw) * dt
                self._odom_y += v * math.sin(self._odom_yaw) * dt
                self._odom_yaw += w * dt
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.get_parameter('odom_frame').value
            odom_msg.child_frame_id = self.get_parameter('base_frame').value
            odom_msg.pose.pose.position.x = self._odom_x
            odom_msg.pose.pose.position.y = self._odom_y
            odom_msg.twist.twist.linear.x = v
            odom_msg.twist.twist.angular.z = w
            self._odom_pub.publish(odom_msg)

        frame = Stm32Protocol.encode_velocity(v, w)
        self._send_frame(frame)

        # 尝试接收反馈数据
        self._read_feedback()

    def _read_feedback(self):
        """读取 STM32 上报的反馈数据。"""
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                return
            try:
                waiting = self._ser.in_waiting
            except (serial.SerialException, OSError):
                return

        if waiting < 6:  # 最小帧长: header(2) + cmd(1) + len(1) + data(>=0) + cs(1)
            return

        with self._ser_lock:
            try:
                raw = self._ser.read(waiting)
            except serial.SerialException:
                return

        # 帧解析状态机 (简化: 查找 header 后按长度解析)
        i = 0
        n = len(raw)
        while i < n - 4:
            if raw[i] == 0x55 and raw[i+1] == 0xAA:
                cmd = raw[i+2]
                dlen = raw[i+3]
                frame_end = i + 4 + dlen + 1  # header + cmd + len + data + cs
                if frame_end > n:
                    i += 1
                    continue
                data = raw[i+4:i+4+dlen]
                rx_cs = raw[i+4+dlen]
                # 验证校验和
                frame_bytes = raw[i:frame_end-1]
                calc_cs = Stm32Protocol.checksum(frame_bytes)
                if calc_cs == rx_cs:
                    self._handle_response(cmd, data)
                i = frame_end
            else:
                i += 1

    def _handle_response(self, cmd: int, data: bytes):
        """处理 STM32 上报的数据帧。"""
        # 目前仅 log，完整里程计发布可后续扩展
        if cmd == Stm32Protocol.RESP_ENCODER:
            result = Stm32Protocol.decode_encoder(data)
            if result:
                self.get_logger().debug(
                    f'Encoder: L={result[0]} R={result[1]}')
        elif cmd == Stm32Protocol.RESP_BATTERY:
            result = Stm32Protocol.decode_battery(data)
            if result:
                self.get_logger().debug(
                    f'Battery: {result}mV = {result/1000:.1f}V')


# ==============================================================================
# 入口
# ==============================================================================

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SerialBridge()
        rclpy.spin(node)
    except RuntimeError as e:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
