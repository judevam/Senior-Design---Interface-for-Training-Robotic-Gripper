# sts.py — minimal MicroPython driver for Feetech STS-series bus servos (e.g. STS3215)
# Protocol: FF FF ID LEN INSTR PARAMS... CHECKSUM, little-endian 16-bit values.
from machine import UART
import time

# Instructions
PING = 0x01
READ = 0x02
WRITE = 0x03
SYNC_WRITE = 0x83
BROADCAST = 0xFE

# Register addresses (STS memory table)
REG_ID = 5
REG_TORQUE = 40
REG_ACC = 41
REG_GOAL_POS = 42
REG_LOCK = 55
REG_POS = 56
REG_SPEED = 58
REG_LOAD = 60
REG_VOLT = 62
REG_TEMP = 63
REG_MOVING = 66


def _signed(value, sign_bit):
    """Feetech uses a sign bit rather than two's complement."""
    if value & (1 << sign_bit):
        return -(value & ~(1 << sign_bit))
    return value


class STSBus:
    def __init__(self, uart_id=2, tx=17, rx=16, baud=1_000_000, timeout_ms=20):
        self.uart = UART(uart_id, baudrate=baud, tx=tx, rx=rx, timeout=0)
        self.timeout_ms = timeout_ms

    # ---------- low level ----------
    @staticmethod
    def _packet(sid, instr, params=b""):
        body = bytes([sid, len(params) + 2, instr]) + bytes(params)
        return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])

    def _send(self, pkt):
        while self.uart.any():          # clear stale bytes
            self.uart.read()
        self.uart.write(pkt)

    @staticmethod
    def _parse(buf, sent, sid):
        """Find a valid status packet from sid in buf, skipping our own echo.
        Returns (error, params) when found, or None if more bytes are needed."""
        i = 0
        n = len(sent)
        while i + 1 < len(buf):
            if buf[i] != 0xFF or buf[i + 1] != 0xFF:
                i += 1
                continue
            # Our own transmission echoed back (some adapters do this)
            if buf[i:i + n] == sent:
                i += n
                continue
            if len(buf) - i < n and sent.startswith(buf[i:]):
                return None             # partial echo, wait for more
            if len(buf) < i + 4:
                return None
            length = buf[i + 3]
            end = i + 4 + length
            if len(buf) < end:
                return None
            pkt_id = buf[i + 2]
            body = buf[i + 2:end - 1]
            if pkt_id == sid and ((~sum(body)) & 0xFF) == buf[end - 1]:
                return buf[i + 4], bytes(buf[i + 5:end - 1])
            i += 1
        return None

    def _transact(self, sid, instr, params=b""):
        pkt = self._packet(sid, instr, params)
        self._send(pkt)
        buf = b""
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < self.timeout_ms:
            if self.uart.any():
                buf += self.uart.read()
                result = self._parse(buf, pkt, sid)
                if result:
                    return result
        return None

    # ---------- basic ops ----------
    def ping(self, sid):
        return self._transact(sid, PING) is not None

    def read(self, sid, addr, length):
        r = self._transact(sid, READ, bytes([addr, length]))
        if r is None or len(r[1]) != length:
            return None
        return r[1]

    def write(self, sid, addr, data):
        return self._transact(sid, WRITE, bytes([addr]) + bytes(data)) is not None

    def read_u16(self, sid, addr):
        d = self.read(sid, addr, 2)
        return None if d is None else d[0] | (d[1] << 8)

    # ---------- motion ----------
    @staticmethod
    def _move_data(position, speed, acc):
        position = max(0, min(4095, int(position)))
        speed = max(0, min(32767, int(speed)))
        acc = max(0, min(254, int(acc)))
        # ACC, POS_L, POS_H, TIME_L, TIME_H, SPEED_L, SPEED_H
        return bytes([acc, position & 0xFF, position >> 8, 0, 0,
                      speed & 0xFF, speed >> 8])

    def move(self, sid, position, speed=2000, acc=50):
        """position 0-4095 (2048 = center), speed in steps/s, acc 0-254."""
        return self.write(sid, REG_ACC, self._move_data(position, speed, acc))

    def sync_move(self, ids, positions, speed=2000, acc=50):
        """Move several servos at once in a single packet (no reply)."""
        params = bytearray([REG_ACC, 7])
        for sid, pos in zip(ids, positions):
            params.append(sid)
            params += self._move_data(pos, speed, acc)
        self._send(self._packet(BROADCAST, SYNC_WRITE, params))
        time.sleep_ms(1)

    def torque(self, sid, on=True):
        return self.write(sid, REG_TORQUE, [1 if on else 0])

    # ---------- feedback ----------
    def read_pos(self, sid):
        v = self.read_u16(sid, REG_POS)
        return None if v is None else _signed(v, 15)

    def read_speed(self, sid):
        v = self.read_u16(sid, REG_SPEED)
        return None if v is None else _signed(v, 15)

    def read_load(self, sid):
        v = self.read_u16(sid, REG_LOAD)
        return None if v is None else _signed(v, 10)

    def read_voltage(self, sid):
        d = self.read(sid, REG_VOLT, 1)
        return None if d is None else d[0] / 10      # volts

    def read_temp(self, sid):
        d = self.read(sid, REG_TEMP, 1)
        return None if d is None else d[0]           # °C

    def is_moving(self, sid):
        d = self.read(sid, REG_MOVING, 1)
        return None if d is None else bool(d[0])

    # ---------- config ----------
    def change_id(self, old_id, new_id):
        self.write(old_id, REG_LOCK, [0])            # unlock EEPROM
        self.write(old_id, REG_ID, [new_id])
        time.sleep_ms(50)
        self.write(new_id, REG_LOCK, [1])            # lock EEPROM
        return self.ping(new_id)
