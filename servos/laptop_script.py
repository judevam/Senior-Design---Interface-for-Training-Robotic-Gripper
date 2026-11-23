import time, json
import xrobotoolkit_sdk as xrt
import paho.mqtt.client as mqtt

SQUEEZE_NAMES = ["get_right_squeeze", "get_right_grip", "get_right_grip_value"]
squeeze_fn = next((getattr(xrt, n) for n in SQUEEZE_NAMES if hasattr(xrt, n)), None)
if squeeze_fn is None:
    raise SystemExit("No squeeze/grip function found.")

try:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
except AttributeError:
    client = mqtt.Client()
client.connect("localhost", 1883)
client.loop_start()

xrt.init()
trig_down = False
t0 = time.time()
try:
    while True:
        r = xrt.get_right_trigger()
        g = squeeze_fn()
        x, y = xrt.get_right_axis()
        if not trig_down and r > 0.2:
            trig_down = True
            print("TRIGGER PRESSED  %s  r=%.2f" % (time.strftime("%H:%M:%S") + ".%03d" % (time.time() % 1 * 1000), r))
        elif trig_down and r < 0.1:
            trig_down = False
            print("TRIGGER RELEASED %s" % (time.strftime("%H:%M:%S") + ".%03d" % (time.time() % 1 * 1000)))
        client.publish("gripper/cmd", json.dumps({"r": r, "g": g, "y": y, "t": int((time.time() - t0) * 1000)}))
        time.sleep(1/30)
finally:
    client.publish("gripper/cmd", json.dumps({"r": 0, "g": 0, "y": 0}))
    xrt.close()
