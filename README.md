# Interface for Training a Robotic Gripper

Senior design project: a motorized tendon-drive hand interface for collecting data to train a robotic gripper (UMI / DexUMI / YUBI style). Design background is in [DESIGN_NOTES.md](DESIGN_NOTES.md).

## Pipeline

PICO controller -> XRoboToolkit -> PC -> ESP32 -> Feetech STS3215 servos

- **PICO data:** XRoboToolkit provides the controller data (trigger, grip, joystick, IMU) to the PC.
- **ESP32:** Receives finger targets from the PC and drives the servos (MicroPython).

The PC-to-ESP32 link is not built yet (see the status note in `software/README.md`).

## Layout

| Folder | Contents |
|---|---|
| [servos/](servos/) | ESP32 MicroPython: bus driver (`sts.py`), ID setup (`set_id.py`), demo (`main.py`), wiring notes |
| [software/](software/) | PC-side Python: Feetech USB driver, bench tests, legacy teleop receiver |
