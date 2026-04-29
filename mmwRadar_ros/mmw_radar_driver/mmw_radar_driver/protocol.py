"""AT6010 SOC HCI Protocol layer for MS60-3015S80M4 mmWave radar.

Handles frame construction (command mode) and parsing (response + auto-report).

Protocol frame formats (all fields little-endian):

Command frame (send):
    HEAD(0x58) | CMD(1 byte) | PARAM_LEN(1 byte) | PARAM[0..n] | CHECK(1 byte)

Response frame (reply):
    HEAD(0x59) | CMD(1 byte) | PARAM_LEN(1 byte) | PARAM[0..n] | CHECK(1 byte)

Auto-report frame (data):
    HEAD(0x5A) | LEN(1 byte) | TYPE(1 byte) | PAYLOAD[0..LEN-1] | CHECK(1 byte)

CHECK = sum of all preceding bytes, low 8 bits.
"""

import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple


# --- Frame markers ---
HEAD_SEND = 0x58
HEAD_RESP = 0x59
HEAD_REPORT = 0x5A

# --- Auto-report types ---
REPORT_TYPE_FULL = 0       # Full detection info
REPORT_TYPE_BSD = 7        # BSD target info (the one used by this radar)

# --- Command bytes ---
CMD_GET_VERSION = 0xFE
CMD_RADAR_ENABLE = 0xD1    # Open (1) / Close (0) radar sensing
CMD_RADAR_STATUS = 0xD0    # Get radar sensing status
CMD_SET_DET_LEVEL = 0x02   # Set detection level (0-15)
CMD_GET_DET_LEVEL = 0x03   # Get detection level
CMD_SET_MOT_MAX_RANGE = 0xD2     # Set motion detection max range (cm, u16)
CMD_SET_MOT_MIN_RANGE = 0x34     # Set motion detection min range (cm, u16)
CMD_SET_MOT_SENSITIVITY = 0x35   # Set motion sensitivity (0-10)
CMD_SET_MICRO_MAX_RANGE = 0x36   # Set micro-motion max range (cm, u16)
CMD_SET_MICRO_MIN_RANGE = 0x37   # Set micro-motion min range (cm, u16)
CMD_SET_MICRO_SENSITIVITY = 0x38 # Set micro-motion sensitivity (0-10)
CMD_SET_BHR_MAX_RANGE = 0x39     # Set breath max range (cm, u16)
CMD_SET_BHR_MIN_RANGE = 0x3A     # Set breath min range (cm, u16)
CMD_SET_BHR_SENSITIVITY = 0x3B   # Set breath sensitivity (0-10)
CMD_GET_SENSING_CONFIG = 0x33    # Get sensing configuration
CMD_GET_BOUNDARY = 0x32          # Get algorithm boundary values
CMD_SAVE_SETTINGS = 0x08         # Save settings to flash
CMD_RESET = 0x13                 # System reset
CMD_BAUD_RATE = 0x19             # Switch baud rate


@dataclass
class RadarTarget:
    """Single radar target with speed, position, and angle."""
    id: int          # Target ID
    range_m: float   # Distance in meters
    angle_deg: float # Angle in degrees (positive = left)
    velocity_ms: float  # Velocity in m/s (positive = approaching)


@dataclass
class RadarFrame:
    """Parsed BSD auto-report frame containing target array."""
    obj_count: int
    targets: List[RadarTarget]


def _checksum(data: bytes) -> int:
    """Compute 8-bit checksum: sum of all bytes, low 8 bits."""
    return sum(data) & 0xFF


def build_cmd_frame(cmd: int, params: Optional[bytes] = None) -> bytes:
    """Build a command frame (0x58 header) for sending to radar.

    Args:
        cmd: Command byte (e.g., CMD_RADAR_ENABLE).
        params: Raw parameter bytes (little-endian encoded), or None.

    Returns:
        Complete frame bytes ready to send over serial.
    """
    if params is None:
        params = b''
    param_len = len(params)
    payload = bytes([cmd, param_len]) + params
    head_and_payload = bytes([HEAD_SEND]) + payload
    chk = _checksum(head_and_payload)
    return head_and_payload + bytes([chk])


def parse_response(raw: bytes) -> Tuple[int, bytes, bool]:
    """Parse a response frame (0x59 header) from radar.

    Args:
        raw: Raw bytes from serial. Must start with 0x59 + payload + check.

    Returns:
        Tuple of (cmd_byte, param_bytes, is_valid).
        is_valid is True if checksum matches.
    """
    if len(raw) < 4:
        return 0, b'', False
    if raw[0] != HEAD_RESP:
        return 0, b'', False
    cmd = raw[1]
    param_len = raw[2]
    params_end = 3 + param_len
    if len(raw) < params_end + 1:
        return cmd, b'', False
    params = raw[3:params_end]
    check = raw[params_end]
    expected = _checksum(raw[:params_end])
    return cmd, params, check == expected


def parse_auto_report(raw: bytes) -> Optional[RadarFrame]:
    """Parse a BSD auto-report frame (0x5A header, TYPE=7).

    Only handles TYPE=7 (BSD target information). Other types return None.

    BSD frame payload layout (little-endian):
        u16 obj_num
        u16 reserved
        bsd_obj_info_t obj[obj_num]  (4 bytes each: s8 range, s8 angle, s8 velo, s8 id)

    Args:
        raw: Raw bytes starting with 0x5A header.

    Returns:
        RadarFrame if valid TYPE=7 frame, None otherwise.
    """
    if len(raw) < 4:
        return None
    if raw[0] != HEAD_REPORT:
        return None

    frame_len = raw[1]   # LEN = payload bytes (TYPE + PAYLOAD)
    report_type = raw[2]

    if report_type != REPORT_TYPE_BSD:
        return None

    # Verify minimum: TYPE(1) + obj_num(2) + reserved(2) = 5 bytes payload
    if frame_len < 5:
        return None

    # CHECK position: HEAD(1) + LEN(1) + PAYLOAD(LEN) = 2 + frame_len
    check_idx = 2 + frame_len
    if len(raw) < check_idx + 1:
        return None

    chk = raw[check_idx]
    expected = _checksum(raw[:check_idx])
    if chk != expected:
        return None

    # Parse payload
    payload = raw[3:check_idx]
    if len(payload) < 4:
        return None

    obj_num = struct.unpack_from('<H', payload, 0)[0]
    # reserved = struct.unpack_from('<H', payload, 2)[0]  # unused

    # obj_num may be 0-8, but actual data present only for detected targets
    # Each object is 4 bytes, variable output length
    obj_data = payload[4:]
    actual_obj_count = len(obj_data) // 4
    # Sanity: output obj count should match obj_num, but protocol says
    # only detected targets are output; trust the actual data length
    count = min(obj_num, actual_obj_count, 8)

    targets = []
    for i in range(count):
        offset = i * 4
        if offset + 4 > len(obj_data):
            break
        range_val, angle_val, velo_val, obj_id = struct.unpack_from('<bbbb', obj_data, offset)
        targets.append(RadarTarget(
            id=obj_id,
            range_m=float(range_val),
            angle_deg=float(angle_val),
            velocity_ms=float(velo_val),
        ))

    return RadarFrame(obj_count=count, targets=targets)


def find_frame_start(data: bytes) -> int:
    """Find the index of the first 0x5A auto-report frame header in data.

    Returns -1 if not found.
    """
    for i, b in enumerate(data):
        if b == HEAD_REPORT:
            return i
    return -1
