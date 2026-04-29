"""Serial port I/O layer for mmWave radar.

Manages CH340 USB-to-UART connection on Jetson NANO.
Handles byte-level I/O, frame synchronization (0x5A header detection),
checksum verification, command sending / response waiting,
and auto-reconnect with exponential backoff.
"""

import logging
import threading
import time
from typing import Callable, Optional

import serial

from .protocol import (
    HEAD_RESP,
    build_cmd_frame,
    find_frame_start,
    parse_auto_report,
    parse_response,
    RadarFrame,
)

logger = logging.getLogger(__name__)


class SerialDriver:
    """Manages serial connection to the radar via CH340 USB-UART.

    Runs a background read thread that accumulates bytes into a ring buffer,
    then extracts complete 0x5A auto-report frames for processing.

    Supports command-response mode (0x58 send / 0x59 response) with timeout,
    used during initialization and parameter updates.
    """

    # Buffer and frame limits
    MAX_BUFFER_SIZE = 4096
    MAX_FRAME_SIZE = 100     # BSD with 8 targets = 4 + 8*4 = 36 bytes + overhead

    # Reconnect parameters
    RECONNECT_BASE_DELAY = 1.0   # seconds
    RECONNECT_MAX_DELAY = 30.0   # seconds

    def __init__(self,
                 port: str = '/dev/ttyUSB0',
                 baud_rate: int = 921600,
                 on_frame: Optional[Callable[[RadarFrame], None]] = None):
        self._port = port
        self._baud_rate = baud_rate
        self._on_frame = on_frame

        self._serial: Optional[serial.Serial] = None
        self._buffer = bytearray()
        self._lock = threading.Lock()
        self._read_thread: Optional[threading.Thread] = None
        self._running = False
        self._connected = False

        # Consecutive checksum failures for health monitoring
        self._checksum_failures = 0

    # ---- Public API ----

    def open(self) -> bool:
        """Open serial port. Returns True on success."""
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
            )
            logger.info(f'Serial port {self._port} opened at {self._baud_rate} bps')
            self._connected = True
            return True
        except (serial.SerialException, OSError) as e:
            logger.error(f'Failed to open serial port {self._port}: {e}')
            self._connected = False
            return False

    def close(self):
        """Close serial port and stop read thread."""
        self._running = False
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
            logger.info(f'Serial port {self._port} closed')
        self._connected = False

    def start_reading(self):
        """Start background read thread for auto-report frames."""
        if self._running:
            return
        if not self._serial or not self._serial.is_open:
            logger.error('Cannot start reading: serial port not open')
            return
        self._running = True
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()
        logger.info('Background read thread started')

    def send_command(self, cmd: int, params: Optional[bytes] = None,
                     timeout: float = 0.5) -> Optional[bytes]:
        """Send a command frame and wait for response.

        Note: the auto-report read thread may be running; we reset the input
        buffer and filter for 0x59 response header explicitly to avoid
        misinterpreting auto-report (0x5A) frames.

        Args:
            cmd: Command byte.
            params: Parameter bytes (little-endian), or None.
            timeout: Maximum seconds to wait for response.

        Returns:
            Response parameter bytes if valid response received, None on failure.
        """
        if not self._serial or not self._serial.is_open:
            logger.error('Cannot send command: serial port not open')
            return None

        frame = build_cmd_frame(cmd, params)
        retries = 3
        for attempt in range(retries):
            try:
                self._serial.reset_input_buffer()
                self._serial.write(frame)
                logger.debug(f'Sent command 0x{cmd:02X}: {frame.hex()}')

                start = time.time()
                buf = bytearray()
                while time.time() - start < timeout:
                    b = self._serial.read(1)
                    if not b:
                        continue
                    # Only accumulate bytes after seeing 0x59 response header
                    if buf or b[0] == HEAD_RESP:
                        buf.extend(b)

                    if len(buf) >= 3:
                        param_len = buf[2]
                        expected = 3 + param_len + 1  # header+cmd+len + params+check
                        if len(buf) >= expected:
                            resp_cmd, params, valid = parse_response(bytes(buf[:expected]))
                            if valid and resp_cmd == cmd:
                                logger.debug(f'Valid response for 0x{cmd:02X}')
                                return params
                            # Invalid response: discard parsed bytes, keep scanning
                            buf = buf[expected:]

                return None  # timeout
            except (serial.SerialException, OSError) as e:
                logger.warning(f'Command send attempt {attempt+1}/{retries} failed: {e}')
                time.sleep(0.05)
        return None

    def is_connected(self) -> bool:
        return self._connected and self._serial is not None and self._serial.is_open

    @property
    def checksum_failures(self) -> int:
        with self._lock:
            return self._checksum_failures

    def reset_failure_count(self):
        with self._lock:
            self._checksum_failures = 0

    # ---- Internal ----

    def _read_loop(self):
        """Background thread: read bytes from serial, extract frames,
        call on_frame callback for each valid BSD frame.
        """
        while self._running:
            try:
                if not self._serial or not self._serial.is_open:
                    if not self._attempt_reconnect():
                        time.sleep(0.1)
                        continue

                raw = self._serial.read(self._serial.in_waiting or 1)
                if raw:
                    self._process_bytes(raw)
            except (serial.SerialException, OSError) as e:
                logger.warning(f'Serial read error: {e}')
                self._connected = False
                time.sleep(0.1)

    def _process_bytes(self, raw: bytes):
        """Accumulate bytes, extract complete frames by 0x5A header sync."""
        with self._lock:
            self._buffer.extend(raw)
            if len(self._buffer) > self.MAX_BUFFER_SIZE:
                logger.warning(f'Buffer overflow ({len(self._buffer)}), clearing')
                self._buffer.clear()
                return

            while True:
                # Find frame start
                idx = find_frame_start(self._buffer)
                if idx < 0:
                    # No header found, discard all
                    self._buffer.clear()
                    break
                if idx > 0:
                    # Discard bytes before header
                    logger.debug(f'Skipping {idx} bytes before 0x5A header')
                    del self._buffer[:idx]

                # Try to parse a frame
                frame, consumed = self._extract_frame(self._buffer)
                if frame is not None:
                    self._checksum_failures = 0
                    del self._buffer[:consumed]
                    if self._on_frame:
                        try:
                            self._on_frame(frame)
                        except Exception as e:
                            logger.error(f'Frame callback error: {e}')
                elif consumed == 0:
                    # Incomplete frame, wait for more data
                    break
                else:
                    # Invalid frame, skip the header byte and re-scan
                    del self._buffer[:1]
                    self._checksum_failures += 1

    def _extract_frame(self, data: bytearray) -> tuple:
        """Try to parse a complete auto-report frame from data.

        Returns (RadarFrame | None, consumed_bytes).
        consumed_bytes = 0 means incomplete frame (need more data).
        """
        if len(data) < 2:
            return None, 0

        frame_len = data[1]  # LEN = payload byte count
        total_len = 3 + frame_len + 1  # HEAD + LEN + payload + CHECK

        if len(data) < total_len:
            return None, 0  # Incomplete

        frame = parse_auto_report(bytes(data[:total_len]))
        return frame, total_len

    def _attempt_reconnect(self) -> bool:
        """Try to reconnect with exponential backoff. Returns True if reconnected."""
        if self._connected or (self._serial and self._serial.is_open):
            return True

        delay = self.RECONNECT_BASE_DELAY
        while self._running and not self._connected:
            logger.info(f'Attempting reconnect to {self._port} (delay={delay:.1f}s)...')
            time.sleep(min(delay, self.RECONNECT_MAX_DELAY))
            if self.open():
                return True
            delay = min(delay * 2, self.RECONNECT_MAX_DELAY)

        return False
