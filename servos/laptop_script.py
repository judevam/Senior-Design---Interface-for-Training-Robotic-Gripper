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
try:
    while True:
        r = xrt.get_right_trigger()
        g = squeeze_fn()
        x, y = xrt.get_right_axis()
        client.publish("gripper/cmd", json.dumps({"r": r, "g": g, "y": y}))
        time.sleep(1/30)
finally:
    client.publish("gripper/cmd", json.dumps({"r": 0, "g": 0, "y": 0}))
    xrt.close()
