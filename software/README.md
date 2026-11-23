# PC-side servo software

Current pipeline: PICO controller -> XRoboToolkit -> PC -> ESP32 -> Feetech STS3215 servos. The ESP32 side is in [../servos/](../servos/).

**Status:** The old Unity/UDP sender and its tools were removed, since XRoboToolkit now supplies the PICO data. Nothing here has been run against real hardware. Register addresses in `feetech_bus.py` come from the STS3215 memory table and should be verified on first power-up.

**Legacy, to be reworked:** `teleop_receiver.py` still receives the old UDP packet (`"seq,t_ms,idx,mid,thumb,base"`, values 0..1) and drives servos over a USB Feetech board. It needs to be replaced with a XRoboToolkit client that forwards finger targets to the ESP32. Its helpers (`clamp01`, `to_position`, `rate_limit`, logging, safety timeout) are worth reusing.

## Contents
- `feetech_bus.py` - Feetech serial protocol over a USB adapter (PC-side)
- `bench_test.py` - scan / set-id / read / sweep for bringing up servos over USB
- `teleop_receiver.py` - legacy UDP receiver (see above)
- `config.json` - servo IDs, limits, open/closed positions (placeholders, calibrate before attaching tendons)
- `tests/` - offline unit tests

## Setup
```
pip install -r requirements.txt
python -m unittest discover -s tests -v
```

## Bench bring-up over USB (no tendons, one servo at a time)
1. `python bench_test.py scan` - lists responding IDs. New servos default to ID 1.
2. `python bench_test.py set-id 1 2` - give each servo a unique ID.
3. `python bench_test.py read 1` - live position/load/current/temp.
4. `python bench_test.py sweep 1 --lo 1900 --hi 2200` - small slow sweep at the low torque limit.
5. Calibrate `open_pos` / `closed_pos` per finger into `config.json`; raise `torque_limit` gradually.

## Safety (carry over to the new pipeline)
Torque is off after 0.2 s without data and on exit, per-servo over-temperature cutoff, capped slew rate, low hardware torque limit at start.
