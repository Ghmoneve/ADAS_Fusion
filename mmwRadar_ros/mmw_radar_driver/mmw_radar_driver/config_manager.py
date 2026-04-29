"""Radar configuration manager.

Maps ROS 2 parameters to AT6010 protocol commands.
Handles the initialization sequence (command mode) and
runtime parameter updates.

Configuration sequence on init:
  1. Open radar sensing
  2. Set detection level
  3. Set motion detection ranges & sensitivity
  4. Set micro-motion detection ranges & sensitivity
  5. Set breath detection ranges & sensitivity

After configuration, the radar outputs TYPE=7 (BSD) auto-report frames.
"""

import logging
import struct
from typing import Dict, Any, Optional

from .protocol import (
    CMD_RADAR_ENABLE,
    CMD_SET_DET_LEVEL,
    CMD_SET_MOT_MAX_RANGE,
    CMD_SET_MOT_MIN_RANGE,
    CMD_SET_MOT_SENSITIVITY,
    CMD_SET_MICRO_MAX_RANGE,
    CMD_SET_MICRO_MIN_RANGE,
    CMD_SET_MICRO_SENSITIVITY,
    CMD_SET_BHR_MAX_RANGE,
    CMD_SET_BHR_MIN_RANGE,
    CMD_SET_BHR_SENSITIVITY,
    CMD_GET_VERSION,
    CMD_GET_SENSING_CONFIG,
    CMD_SAVE_SETTINGS,
    CMD_RESET,
)
from .serial_driver import SerialDriver

logger = logging.getLogger(__name__)

# Default parameter values matching radar factory settings
DEFAULT_PARAMS: Dict[str, Any] = {
    'serial_port': '/dev/ttyUSB0',
    'baud_rate': 921600,
    'frame_id': 'radar_link',
    'radar_enabled': True,
    'detection_level': 15,
    'mot_det_max_range': 5000,     # cm
    'mot_det_min_range': 50,       # cm
    'mot_det_sensitivity': 5,
    'micro_det_max_range': 3000,   # cm
    'micro_det_min_range': 50,     # cm
    'micro_det_sensitivity': 5,
    'bhr_det_max_range': 2000,     # cm
    'bhr_det_min_range': 50,       # cm
    'bhr_det_sensitivity': 5,
    'publish_rate': 10.0,          # Hz
    'auto_standby': True,
}


class ConfigManager:
    """Manages radar configuration and synchronization with ROS parameters.

    Tracks current radar state to avoid sending redundant commands.
    """

    # Configuration commands that accept u16 values in cm
    _RANGE_COMMANDS = {
        'mot_det_max_range': CMD_SET_MOT_MAX_RANGE,
        'mot_det_min_range': CMD_SET_MOT_MIN_RANGE,
        'micro_det_max_range': CMD_SET_MICRO_MAX_RANGE,
        'micro_det_min_range': CMD_SET_MICRO_MIN_RANGE,
        'bhr_det_max_range': CMD_SET_BHR_MAX_RANGE,
        'bhr_det_min_range': CMD_SET_BHR_MIN_RANGE,
    }

    # Sensitivity commands that accept u8 values (0-10)
    _SENSITIVITY_COMMANDS = {
        'mot_det_sensitivity': CMD_SET_MOT_SENSITIVITY,
        'micro_det_sensitivity': CMD_SET_MICRO_SENSITIVITY,
        'bhr_det_sensitivity': CMD_SET_BHR_SENSITIVITY,
    }

    def __init__(self, serial: SerialDriver):
        self._serial = serial
        # Track sent values to avoid redundant reconfiguration
        self._applied: Dict[str, Any] = {}

    def initialize_radar(self, params: Dict[str, Any]) -> bool:
        """Run full configuration sequence. Returns True if all critical
        commands succeed.
        """
        logger.info('Starting radar initialization sequence...')

        # Step 1: Enable radar sensing
        if not self._send_cmd(CMD_RADAR_ENABLE, bytes([0x01]),
                              'Enable radar sensing'):
            logger.error('Failed to enable radar sensing - critical error')
            return False

        # Step 2: Set detection level
        level = int(params.get('detection_level', 15))
        if not self._set_detection_level(level):
            logger.warning('Failed to set detection level (non-critical)')

        # Step 3-5: Apply all range and sensitivity settings
        for key, cmd in self._RANGE_COMMANDS.items():
            self._apply_range_param(key, cmd, params)

        for key, cmd in self._SENSITIVITY_COMMANDS.items():
            self._apply_sensitivity_param(key, cmd, params)

        logger.info('Radar initialization complete')
        return True

    def update_param(self, key: str, value) -> bool:
        """Handle a single parameter change at runtime.

        Returns True if the parameter was applied successfully.
        """
        if key == 'radar_enabled':
            return self._set_radar_enabled(bool(value))

        if key == 'detection_level':
            return self._set_detection_level(int(value))

        if key in self._RANGE_COMMANDS:
            return self._apply_range_param(key, self._RANGE_COMMANDS[key],
                                           {key: value})

        if key in self._SENSITIVITY_COMMANDS:
            return self._apply_sensitivity_param(key,
                                                 self._SENSITIVITY_COMMANDS[key],
                                                 {key: value})

        # Port/baud/frame_id changes require node restart — no-op here
        logger.debug(f'Parameter {key}={value} does not require command (runtime skip)')
        return True

    def reset_system(self) -> bool:
        """Send system reset command."""
        return self._send_cmd(CMD_RESET, bytes([0x01]), 'System reset')

    def save_settings(self) -> bool:
        """Save current settings to radar flash.

        Flash write takes ~1 second per protocol spec; use 1.5s timeout.
        """
        return self._send_cmd(CMD_SAVE_SETTINGS, bytes([0x01]),
                              'Save settings to flash', timeout=1.5)

    def request_version(self) -> Optional[str]:
        """Request software/hardware version from radar.

        Returns version string like 'v0.3.2 (hw v1.2)' or None on failure.
        """
        params = self._serial.send_command(CMD_GET_VERSION)
        if params is None or len(params) < 7:
            return None
        sw_major, sw_minor, sw_rev, hw_major, hw_minor = struct.unpack_from(
            '<BBBBB', params)
        return f'v{sw_major}.{sw_minor}.{sw_rev} (hw v{hw_major}.{hw_minor})'

    # ---- Internal helpers ----

    def _send_cmd(self, cmd: int, params: bytes, label: str = '',
                  timeout: float = 0.5) -> bool:
        """Send a command and log result. Returns True on success."""
        success = self._serial.send_command(cmd, params, timeout=timeout) is not None
        if success:
            logger.info(f'{label} OK' if label else f'Command 0x{cmd:02X} OK')
        else:
            logger.warning(f'{label} FAILED' if label else f'Command 0x{cmd:02X} FAILED')
        return success

    def _set_radar_enabled(self, enable: bool) -> bool:
        val = 0x01 if enable else 0x00
        return self._send_cmd(CMD_RADAR_ENABLE, bytes([val]),
                              f'Radar {"ON" if enable else "OFF"}')

    def _set_detection_level(self, level: int) -> bool:
        level = max(0, min(15, level))
        if self._applied.get('detection_level') == level:
            return True
        ok = self._send_cmd(CMD_SET_DET_LEVEL, bytes([level]),
                            f'Set detection level={level}')
        if ok:
            self._applied['detection_level'] = level
        return ok

    def _apply_range_param(self, key: str, cmd: int,
                           params: Dict[str, Any]) -> bool:
        """Apply a u16 range parameter (unit: cm)."""
        if key not in params:
            return True
        value = int(params[key])
        if self._applied.get(key) == value:
            return True
        data = struct.pack('<H', value)
        ok = self._send_cmd(cmd, data, f'Set {key}={value}cm')
        if ok:
            self._applied[key] = value
        return ok

    def _apply_sensitivity_param(self, key: str, cmd: int,
                                 params: Dict[str, Any]) -> bool:
        """Apply a u8 sensitivity parameter (range 0-10)."""
        if key not in params:
            return True
        value = max(0, min(10, int(params[key])))
        if self._applied.get(key) == value:
            return True
        ok = self._send_cmd(cmd, bytes([value]),
                            f'Set {key}={value}')
        if ok:
            self._applied[key] = value
        return ok
