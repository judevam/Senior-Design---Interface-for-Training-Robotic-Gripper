"""Offline tests (no hardware). Run from software/:  python -m unittest discover -s tests -v"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from feetech_bus import (BusError, FeetechBus, build_packet, checksum,  # noqa: E402
                         decode_signed, INST_PING)
from teleop_receiver import parse_packet, rate_limit, to_position  # noqa: E402


class FakeSerial:
    def __init__(self, reply=b""):
        self.written = b""
        self.reply = reply

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.written += data

    def read(self, n):
        out, self.reply = self.reply[:n], self.reply[n:]
        return out

    def close(self):
        pass


def status(sid, params=b"", error=0):
    body = bytes([sid, len(params) + 2, error]) + params
    return b"\xff\xff" + body + bytes([checksum(body)])


class ProtocolTests(unittest.TestCase):
    def test_ping_packet_known_vector(self):
        self.assertEqual(build_packet(1, INST_PING), bytes.fromhex("ff ff 01 02 01 fb"))

    def test_ping_ok_and_timeout(self):
        self.assertTrue(FeetechBus(serial_obj=FakeSerial(status(1))).ping(1))
        self.assertFalse(FeetechBus(serial_obj=FakeSerial(b"")).ping(1))

    def test_bad_checksum_raises(self):
        bad = bytearray(status(1)); bad[-1] ^= 0xFF
        with self.assertRaises(BusError):
            FeetechBus(serial_obj=FakeSerial(bytes(bad))).read(1, 56, 0)

    def test_sync_goals_layout(self):
        ser = FakeSerial(); bus = FeetechBus(serial_obj=ser)
        bus.sync_goals({1: 2048, 2: 100}, speed=1500, acc=50)
        pkt = ser.written
        self.assertEqual(pkt[:5], bytes([0xFF, 0xFF, 0xFE, len(pkt) - 4, 0x83]))
        self.assertEqual(pkt[5:7], bytes([41, 7]))  # start addr, bytes per servo
        self.assertEqual(pkt[7:15], bytes([1, 50, 0x00, 0x08, 0, 0, 0xDC, 0x05]))  # id, acc, pos 2048, time, speed 1500
        self.assertEqual(pkt[-1], checksum(pkt[2:-1]))

    def test_goal_clamped(self):
        bus = FeetechBus(serial_obj=FakeSerial())
        self.assertEqual(bus.goal_bytes(99999, 10, 0)[1:3], (4095).to_bytes(2, "little"))
        self.assertEqual(bus.goal_bytes(-5, 10, 0)[1:3], (0).to_bytes(2, "little"))

    def test_read_state_decode(self):
        raw = bytearray(15)
        raw[0:2] = (2100).to_bytes(2, "little")
        raw[4:6] = (0x400 | 250).to_bytes(2, "little")   # load -250
        raw[6] = 121; raw[7] = 33
        raw[13:15] = (100).to_bytes(2, "little")
        st = FeetechBus(serial_obj=FakeSerial(status(3, bytes(raw)))).read_state(3)
        self.assertEqual((st["pos"], st["load"], st["temp_c"]), (2100, -250, 33))
        self.assertAlmostEqual(st["voltage_v"], 12.1)
        self.assertAlmostEqual(st["current_ma"], 650.0)

    def test_decode_signed(self):
        self.assertEqual(decode_signed(0x8005, 15), -5)
        self.assertEqual(decode_signed(5, 15), 5)


class ReceiverLogicTests(unittest.TestCase):
    def test_parse_good(self):
        seq, t, vals = parse_packet(b"7,123.5,0.25,1.5,-0.2,0.5")
        self.assertEqual((seq, t), (7, 123.5))
        self.assertEqual(vals, {"idx": 0.25, "mid": 1.0, "thumb": 0.0, "base": 0.5})

    def test_parse_bad(self):
        for bad in (b"", b"1,2,3", b"a,b,c,d,e,f", b"1,2,nan,0,0,0", b"\xff\xfe"):
            self.assertIsNone(parse_packet(bad))

    def test_to_position_both_directions(self):
        self.assertEqual(to_position(0.5, 1000, 2000), 1500)
        self.assertEqual(to_position(1.0, 3000, 2000), 2000)
        self.assertEqual(to_position(2.0, 1000, 2000), 2000)

    def test_rate_limit(self):
        self.assertEqual(rate_limit(0.0, 1.0, 0.1), 0.1)
        self.assertEqual(rate_limit(1.0, 0.0, 0.25), 0.75)
        self.assertEqual(rate_limit(0.5, 0.55, 0.1), 0.55)


if __name__ == "__main__":
    unittest.main()
