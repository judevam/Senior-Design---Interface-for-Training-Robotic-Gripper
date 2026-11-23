# main.py — 3x STS3215 on the URT-2 bus + 1x PWM micro servo (SG90/MG90 style)
# Each motor runs its own independent motion pattern at the same time.
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

# ---------- per-motor patterns ----------
# Each pattern is a list of (angle_degrees, hold_ms). The motor goes to each
# angle in order, waits hold_ms, then moves on and repeats. Angles are from
# center for bus servos (-180..180) and 0..180 for the micro servo.
# speed is in steps/s (4096 = one full turn per second), acc is 0-254.
# Keep speed low while testing on a small power supply.
PATTERNS = {
    # Servo 1: slow, small back-and-forth
    1: {"speed": 800,  "acc": 30, "steps": [(-30, 2000), (30, 2000)]},
    # Servo 2: wider and faster swing
    2: {"speed": 2000, "acc": 50, "steps": [(-90, 1200), (90, 1200)]},
    # Servo 3: stepped through four positions
    3: {"speed": 1200, "acc": 40, "steps": [(-60, 1000), (-20, 1000), (20, 1000), (60, 1000)]},
}
MICRO_STEPS = [(30, 700), (90, 700), (150, 700), (90, 700)]   # (angle 0-180, hold_ms)

class Runner:
    """Steps one motor through its list of (angle, hold_ms) forever, without blocking."""
    def __init__(self, steps, send):
        self.steps = steps
        self.send = send
        self.i = 0
        self.due = time.ticks_ms()      # time to move to the next step

    def update(self, now):
        if time.ticks_diff(now, self.due) >= 0:
            angle, hold = self.steps[self.i]
            self.send(angle)
            self.due = time.ticks_add(now, hold)
            self.i = (self.i + 1) % len(self.steps)

def bus_sender(sid):
    p = PATTERNS[sid]
    return lambda deg: bus.move(sid, deg_to_pos(deg), p["speed"], p["acc"])

runners = [Runner(PATTERNS[sid]["steps"], bus_sender(sid)) for sid in IDS]
runners.append(Runner(MICRO_STEPS, micro.angle))

# ---------- program ----------
check_servos()
bus.sync_move(IDS, [2048, 2048, 2048])   # center all
micro.angle(90)
time.sleep(1.5)

last_report = time.ticks_ms()
try:
    while True:
        now = time.ticks_ms()
        for r in runners:
            r.update(now)
        if time.ticks_diff(now, last_report) >= 5000:
            report()
            last_report = now
        time.sleep_ms(10)
except KeyboardInterrupt:
    # Stop button in Thonny lands here
    for sid in IDS:
        bus.torque(sid, False)
    micro.off()
    print("Stopped, torque off.")
