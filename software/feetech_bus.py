"""Minimal Feetech STS-series (STS3215) serial-bus driver built on pyserial.

Protocol: Dynamixel-1.0-style packets, little-endian multi-byte values.
    TX: FF FF ID LEN INSTR PARAMS... CHECKSUM     (LEN = len(params) + 2)
    RX: FF FF ID LEN ERROR PARAMS... CHECKSUM
    CHECKSUM = ~(ID + LEN + INSTR/ERROR + sum(PARAMS)) & 0xFF

Register addresses are for the STS3215 memory table. Verify them against the
Feetech docs for the exact servo you receive before trusting them.
"""

import time

import serial

# Instructions
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_WRITE = 0x83

# Registers (STS3215)
REG_ID = 5
REG_TORQUE_ENABLE = 40
REG_ACC = 41
REG_GOAL_POS = 42  # 2 bytes; followed by goal time (44) and goal speed (46)
REG_TORQUE_LIMIT = 48  # 2 bytes, 0-1000
REG_LOCK = 55  # 0 = EEPROM unlocked, 1 = locked
REG_PRESENT_POS = 56  # block read of 15 bytes: 56..70

STATE_BLOCK_LEN = 15
CURRENT_LSB_MA = 6.5


class BusError(Exception):
    pass


def checksum(body: bytes) -> int:
    """Checksum over ID..last param (everything between header and checksum)."""
    return (~sum(body)) & 0xFF


def build_packet(servo_id: int, instruction: int, params: bytes = b"") -> bytes:
    body = bytes([servo_id, len(params) + 2, instruction]) + params
    return b"\xff\xff" + body + bytes([checksum(body)])


def le16(value: int) -> bytes:
    return int(value).to_bytes(2, "little")


def decode_signed(value: int, sign_bit: int) -> int:
    """Feetech sign-magnitude: bit `sign_bit` is the sign, lower bits the magnitude."""
    mag = value & ((1 << sign_bit) - 1)
    return -mag if value & (1 << sign_bit) else mag


class FeetechBus:
    def __init__(self, port: str = "COM3", baud: int = 1_000_000,
                 timeout: float = 0.02, serial_obj=None):
        self.ser = serial_obj or serial.Serial(port, baud, timeout=timeout)

    def close(self):
        self.ser.close()

    # ---- low-level -----------------------------------------------------
    def _read_status(self, expect_id: int, expect_params: int) -> bytes:
        """Read one status packet and return its params. Raises BusError."""
        header = self.ser.read(4)  # FF FF ID LEN
        if len(header) < 4:
            raise BusError(f"timeout waiting for servo {expect_id}")
        if header[0] != 0xFF or header[1] != 0xFF:
            raise BusError(f"bad header from servo {expect_id}: {header.hex()}")
        sid, length = header[2], header[3]
        rest = self.ser.read(length)  # ERROR + params + checksum
        if len(rest) < length:
            raise BusError(f"short packet from servo {expect_id}")
        body = bytes([sid, length]) + rest[:-1]
        if checksum(body) != rest[-1]:
            raise BusError(f"bad checksum from servo {expect_id}")
        if sid != expect_id:
            raise BusError(f"expected servo {expect_id}, got {sid}")
        error = rest[0]
        params = rest[1:-1]
        if error:
            raise BusError(f"servo {sid} reported error flags 0x{error:02x}")
        if len(params) != expect_params:
            raise BusError(f"servo {sid}: expected {expect_params} bytes, got {len(params)}")
        return params

    def ping(self, servo_id: int) -> bool:
        self.ser.reset_input_buffer()
        self.ser.write(build_packet(servo_id, INST_PING))
        try:
            self._read_status(servo_id, 0)
            return True
        except BusError:
            return False

    def read(self, servo_id: int, address: int, length: int) -> bytes:
        self.ser.reset_input_buffer()
        self.ser.write(build_packet(servo_id, INST_READ, bytes([address, length])))
        return self._read_status(servo_id, length)

    def write(self, servo_id: int, address: int, data: bytes):
        """Write and wait for the status reply (so errors are not silent)."""
        self.ser.reset_input_buffer()
        self.ser.write(build_packet(servo_id, INST_WRITE, bytes([address]) + data))
        self._read_status(servo_id, 0)

    def sync_write(self, address: int, data_by_id: dict):
        """Write the same register span to several servos. No reply is sent."""
        if not data_by_id:
            return
        lengths = {len(d) for d in data_by_id.values()}
        if len(lengths) != 1:
            raise ValueError("sync_write needs equal-length data for every servo")
        n = lengths.pop()
        params = bytes([address, n])
        for sid, data in data_by_id.items():
            params += bytes([sid]) + data
        self.ser.write(build_packet(0xFE, INST_SYNC_WRITE, params))

    # ---- helpers -------------------------------------------------------
    def set_torque(self, servo_id: int, enabled: bool):
        self.write(servo_id, REG_TORQUE_ENABLE, bytes([1 if enabled else 0]))

    def set_torque_all(self, ids, enabled: bool):
        """Broadcast-style torque on/off with sync write (no replies, fast)."""
        self.sync_write(REG_TORQUE_ENABLE, {i: bytes([1 if enabled else 0]) for i in ids})

    def set_torque_limit(self, servo_id: int, limit: int):
        self.write(servo_id, REG_TORQUE_LIMIT, le16(max(0, min(1000, limit))))

    def goal_bytes(self, position: int, speed: int, acc: int) -> bytes:
        """acc(1) + goal pos(2) + goal time(2, unused) + goal speed(2) = 7 bytes @ REG_ACC."""
        position = max(0, min(4095, int(position)))
        return bytes([max(0, min(254, int(acc)))]) + le16(position) + le16(0) + le16(speed)

    def sync_goals(self, goals: dict, speed: int, acc: int):
        """goals: {servo_id: position_counts}."""
        self.sync_write(REG_ACC, {sid: self.goal_bytes(p, speed, acc) for sid, p in goals.items()})

    def read_state(self, servo_id: int) -> dict:
        raw = self.read(servo_id, REG_PRESENT_POS, STATE_BLOCK_LEN)
        u16 = lambda off: int.from_bytes(raw[off:off + 2], "little")
        return {
            "pos": u16(0),
            "speed": decode_signed(u16(2), 15),
            "load": decode_signed(u16(4), 10),  # -1000..1000 (sign = direction)
            "voltage_v": raw[6] / 10.0,
            "temp_c": raw[7],
            "moving": raw[10],
            "current_ma": decode_signed(u16(13), 15) * CURRENT_LSB_MA,
        }

    def set_id(self, old_id: int, new_id: int):
        """Change a servo's ID (EEPROM). Connect ONE servo at a time."""
        self.write(old_id, REG_LOCK, bytes([0]))
        self.write(old_id, REG_ID, bytes([new_id]))
        time.sleep(0.05)
        self.write(new_id, REG_LOCK, bytes([1]))
