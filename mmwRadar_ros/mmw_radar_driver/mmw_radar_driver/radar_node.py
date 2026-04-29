"""ROS 2 node for MS60-3015S80M4 mmWave radar (AT6010 SOC).

Lifecycle:
  1. Declare parameters from defaults
  2. Open serial port (CH340 USB-UART)
  3. Configure radar via command mode
  4. Read auto-report frames (TYPE=7 BSD)
  5. Publish RadarTargetArray messages
  6. Handle dynamic parameter updates
  7. Monitor health (connection, checksum failures)
  8. Auto-standby when no subscribers
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Header
from std_srvs.srv import Empty

from mmw_radar_msgs.msg import RadarTarget, RadarTargetArray

from .config_manager import ConfigManager, DEFAULT_PARAMS
from .protocol import RadarFrame
from .serial_driver import SerialDriver

logger = logging.getLogger(__name__)


class MmwRadarNode(Node):
    """ROS 2 driver node for MS60-3015S80M4 mmWave radar."""

    # Health monitoring thresholds
    MAX_CHECKSUM_FAILURES = 100   # Consecutive failures before warning
    HEALTH_PUBLISH_PERIOD = 5.0   # seconds between health checks

    def __init__(self):
        super().__init__('mmw_radar_node')

        self._declare_params()

        self._last_health_log = time.time()
        self._active = False

        # Serial driver (not opened yet)
        self._serial = SerialDriver(
            port=self.get_parameter('serial_port').value,
            baud_rate=self.get_parameter('baud_rate').value,
            on_frame=self._on_frame_received,
        )

        # Config manager
        self._config = ConfigManager(self._serial)

        # Publisher
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(RadarTargetArray, '/radar/targets', qos)

        # Services
        self._srv_reset = self.create_service(Empty, '/radar/reset',
                                              self._on_reset)
        self._srv_save = self.create_service(Empty, '/radar/save_settings',
                                             self._on_save_settings)

        # Timer for health checks and subscriber monitoring
        self._health_timer = self.create_timer(1.0, self._health_check)

        # Parameter change handler
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info('mmw_radar_node initialized')

    # ---- Lifecycle ----

    def start(self) -> bool:
        """Open serial, configure radar, start reading. Returns True on success."""
        if not self._serial.open():
            self.get_logger().error(f'Cannot open serial port '
                                    f'{self.get_parameter("serial_port").value}')
            return False

        self._serial.start_reading()

        # Build params dict for config sequence
        params = self._collect_params()
        if not self._config.initialize_radar(params):
            self.get_logger().error('Radar configuration failed')
            return False

        # Log version info
        ver = self._config.request_version()
        if ver:
            self.get_logger().info(f'Radar firmware: {ver}')

        self._active = True
        self.get_logger().info('Radar driver started successfully')
        return True

    def stop(self):
        """Shutdown: close serial, stop threads."""
        self._active = False
        self._serial.close()
        self.get_logger().info('Radar driver stopped')

    # ---- Frame callback (called from serial thread) ----

    def _on_frame_received(self, frame: RadarFrame):
        """Callback from SerialDriver when a valid BSD frame is parsed."""
        if not self._active:
            return
        if self.get_parameter('auto_standby').value and self._pub.get_subscription_count() == 0:
            return

        msg = RadarTargetArray()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.get_parameter('frame_id').value,
        )
        msg.targets = [
            RadarTarget(
                id=t.id,
                range=t.range_m,
                angle=t.angle_deg,
                velocity=t.velocity_ms,
            )
            for t in frame.targets
        ]
        try:
            self._pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')

    # ---- Parameter handling ----

    def _declare_params(self):
        """Declare all ROS 2 parameters with defaults."""
        for name, default in DEFAULT_PARAMS.items():
            self.declare_parameter(name, default)

    def _collect_params(self) -> dict:
        """Collect current parameter values into a dict."""
        return {name: self.get_parameter(name).value
                for name in DEFAULT_PARAMS}

    def _on_param_change(self, params):
        """Handle runtime parameter updates."""
        result = SetParametersResult(successful=True)
        for param in params:
            try:
                if not self._config.update_param(param.name, param.value):
                    self.get_logger().warn(f'Failed to apply parameter: '
                                           f'{param.name}={param.value}')
            except Exception as e:
                self.get_logger().error(f'Parameter {param.name} error: {e}')
                result.successful = False
        return result

    # ---- Services ----

    def _on_reset(self, request, response):
        self.get_logger().info('System reset requested')
        self._config.reset_system()
        return response

    def _on_save_settings(self, request, response):
        self.get_logger().info('Save settings requested')
        if self._config.save_settings():
            self.get_logger().info('Settings saved to flash')
        else:
            self.get_logger().warn('Failed to save settings')
        return response

    # ---- Health monitoring ----

    def _health_check(self):
        """Periodic health check: connection status, checksum failures,
        subscriber count changes.
        """
        now = time.time()

        # Connection health
        if not self._serial.is_connected():
            self.get_logger().error('Serial connection lost', throttle_duration_sec=10.0)
            self._active = False
            return
        self._active = True

        # Checksum failure rate
        failures = self._serial.checksum_failures
        if failures > self.MAX_CHECKSUM_FAILURES:
            self.get_logger().error(f'Checksum failures: {failures} '
                                    f'(threshold: {self.MAX_CHECKSUM_FAILURES})',
                                    throttle_duration_sec=10.0)

        # Periodic health log
        if now - self._last_health_log > self.HEALTH_PUBLISH_PERIOD:
            sub_count = self._pub.get_subscription_count()
            self.get_logger().info(
                f'Health: connected={self._serial.is_connected()}, '
                f'subscribers={sub_count}, checksum_fails={failures}',
                throttle_duration_sec=self.HEALTH_PUBLISH_PERIOD)
            self._last_health_log = now


def main(args=None):
    """Entry point for ROS 2 node."""
    rclpy.init(args=args)
    node = MmwRadarNode()

    try:
        if node.start():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
