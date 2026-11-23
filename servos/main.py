# main.py — 3x STS3215 on the URT-2 bus + 1x PWM micro servo (SG90/MG90 style)
from machine import Pin, PWM
import time
from sts import STSBus

# ---------- bus servos ----------
bus = STSBus(uart_id=2, tx=17, rx=16)
IDS = [1, 2, 3]

# ---------- micro servo ----------
class MicroServo:
    def __init__(self, pin, min_us=500, max_us=2400):
        self.pwm = PWM(Pin(pin), freq=50)
        self.min_us = min_us
        self.max_us = max_us

    def angle(self, deg):
        deg = max(0, min(180, deg))
        us = self.min_us + (self.max_us - self.min_us) * deg / 180
        self.pwm.duty_u16(int(us * 65535 / 20000))   # 20 ms period

    def off(self):
        self.pwm.duty_u16(0)

micro = MicroServo(18)

# ---------- helpers ----------
def deg_to_pos(deg):
    """-180..180 degrees from center -> 0..4095 steps."""
    return int(2048 + deg * 4096 / 360)

def check_servos():
    for sid in IDS:
        print("Servo", sid, "OK" if bus.ping(sid) else "NOT FOUND")

def report():
    for sid in IDS:
        print("ID {}: pos={} load={} temp={}C volt={}V".format(
            sid, bus.read_pos(sid), bus.read_load(sid),
            bus.read_temp(sid), bus.read_voltage(sid)))

# ---------- program ----------
check_servos()
bus.sync_move(IDS, [2048, 2048, 2048])   # center all
micro.angle(90)
time.sleep(1.5)

try:
    while True:
        bus.sync_move(IDS, [deg_to_pos(-90)] * 3, speed=2000, acc=50)
        micro.angle(30)
        time.sleep(1.5)

        bus.sync_move(IDS, [deg_to_pos(90)] * 3, speed=2000, acc=50)
        micro.angle(150)
        time.sleep(1.5)

        report()
except KeyboardInterrupt:
    # Stop button in Thonny lands here
    for sid in IDS:
        bus.torque(sid, False)
    micro.off()
    print("Stopped, torque off.")
