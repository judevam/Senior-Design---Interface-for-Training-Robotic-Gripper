# main.py - PICO controller -> MQTT -> 3x STS3215 bus servos
import network, json, time
from machine import reset
from umqtt.simple import MQTTClient
from sts import STSBus

# ================= CONFIG =================
SSID, PASSWORD = "Tufts_Robot", "Jumbos1852!"
BROKER_IP = "10.247.137.84"            # ip -4 addr show wlan0

IDS = [1, 2, 3]                    # 1 = trigger, 2 = squeeze, 3 = joystick Y

# Travel in degrees from servo center (2048). KEEP SMALL until you've checked
# the spools/cables. Negative is one direction, positive the other.
# Servo 1 and 2: 0 (released) -> max (fully pressed)
# Servo 3: joystick down -> min, joystick up -> max
LIMITS = {
    1: (0, 90),
    2: (0, 90),
    3: (-45, 45),
}
REST_DEG = {1: 0, 2: 0, 3: 0}      # where servos go on start / timeout

SPEED = 2000                       # steps/s (4096 = 1 rev/s). Lower for gentler moves
ACC = 50                           # 0-254

IN_DEAD = 0.05                     # analog dead zone (trigger/squeeze)
STICK_DEAD = 0.15                  # joystick dead zone
SMOOTH = 0.4                       # 0..1, higher = more responsive, lower = smoother
SEND_HZ = 30                       # servo command rate
MIN_DELTA = 4                      # steps; ignore smaller changes (reduces jitter)
TIMEOUT_MS = 500                   # laptop silent -> go to rest pose
# ==========================================

# ---------- Wi-Fi ----------
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
try:                                   # disable Wi-Fi power save (lower latency)
    wlan.config(pm=network.WLAN.PM_NONE)
except Exception as e:
    print("power save not changed:", e)
wlan.connect(SSID, PASSWORD)
t0 = time.ticks_ms()
while not wlan.isconnected():
    if time.ticks_diff(time.ticks_ms(), t0) > 20000:
        reset()
    time.sleep(0.1)
print("Wi-Fi OK", wlan.ifconfig())

# ---------- Servos ----------
bus = STSBus(uart_id=2, tx=17, rx=16)

def deg_to_pos(deg):
    return int(2048 + deg * 4096 / 360)

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

for sid in IDS:
    print("Servo", sid, "OK" if bus.ping(sid) else "NOT FOUND")

rest_pos = [deg_to_pos(REST_DEG[s]) for s in IDS]
bus.sync_move(IDS, rest_pos, 800, 30)       # slow move to rest at startup
time.sleep(1.5)

# ---------- Input -> target angle ----------
inputs = {"r": 0.0, "g": 0.0, "y": 0.0}

def analog_to_unit(x):
    x = clamp(x, 0.0, 1.0)
    return 0.0 if x < IN_DEAD else (x - IN_DEAD) / (1 - IN_DEAD)

def stick_to_unit(y):                        # -1..1 with deadband
    if abs(y) < STICK_DEAD:
        return 0.0
    s = (abs(y) - STICK_DEAD) / (1 - STICK_DEAD)
    return s if y > 0 else -s

def target_deg():
    lo1, hi1 = LIMITS[1]; lo2, hi2 = LIMITS[2]; lo3, hi3 = LIMITS[3]
    d1 = lo1 + analog_to_unit(inputs["r"]) * (hi1 - lo1)
    d2 = lo2 + analog_to_unit(inputs["g"]) * (hi2 - lo2)
    u = (stick_to_unit(inputs["y"]) + 1) / 2          # 0..1, 0.5 = stick centered
    d3 = lo3 + u * (hi3 - lo3)
    return [d1, d2, d3]

# ---------- MQTT ----------
last_msg_ms = time.ticks_ms()
trig_down = False
PRESS_ON, PRESS_OFF = 0.2, 0.1         # hysteresis so it doesn't spam

def on_msg(topic, msg):
    global last_msg_ms, trig_down
    try:
        c = json.loads(msg)
        r = float(c["r"])
        if not trig_down and r > PRESS_ON:
            trig_down = True
            print("TRIGGER PRESSED  t=%d ms  r=%.2f" % (time.ticks_ms(), r))
        elif trig_down and r < PRESS_OFF:
            trig_down = False
            print("TRIGGER RELEASED t=%d ms" % time.ticks_ms())
        inputs["r"] = r
        inputs["g"] = float(c["g"])
        inputs["y"] = float(c["y"])
        last_msg_ms = time.ticks_ms()
    except Exception as e:
        print("bad msg:", e)

client = MQTTClient("gripper-esp32", BROKER_IP, keepalive=30)
client.set_callback(on_msg)
client.connect()
client.subscribe(b"gripper/cmd")
print("MQTT OK")

# ---------- Main loop ----------
period_ms = 1000 // SEND_HZ
smooth = list(REST_DEG[s] for s in IDS)      # filtered angles (deg)
last_sent = list(rest_pos)
at_rest = True

try:
    while True:
        start = time.ticks_ms()
        try:
            for _ in range(20):          # drain backlog so we act on newest msg
                client.check_msg()
        except OSError:
            reset()

        timed_out = time.ticks_diff(start, last_msg_ms) > TIMEOUT_MS
        if timed_out:
            goal = [REST_DEG[s] for s in IDS]
        else:
            goal = target_deg()

        # Smooth, then clamp to limits (safety)
        for i, sid in enumerate(IDS):
            smooth[i] += SMOOTH * (goal[i] - smooth[i])
            lo, hi = LIMITS[sid]
            smooth[i] = clamp(smooth[i], lo, hi)

        pos = [deg_to_pos(d) for d in smooth]

        # Only send if something moved enough
        if any(abs(pos[i] - last_sent[i]) >= MIN_DELTA for i in range(3)):
            bus.sync_move(IDS, pos, SPEED, ACC)
            last_sent = pos

        elapsed = time.ticks_diff(time.ticks_ms(), start)
        if elapsed < period_ms:
            time.sleep_ms(period_ms - elapsed)

except KeyboardInterrupt:
    for sid in IDS:
        bus.torque(sid, False)
    print("Stopped, torque off.")
