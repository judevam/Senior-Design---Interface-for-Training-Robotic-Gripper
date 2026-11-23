"""UDP receiver: PICO controller values -> Feetech servos, with CSV logging.

LEGACY: written for the old Unity sender (removed). To be replaced by an XRoboToolkit client.

Packet (ASCII, from the old Unity app): "seq,t_ms,idx,mid,thumb,base"
    idx/mid/thumb/base are normalized 0..1 (0 = open, 1 = closed/forward).

Safety:
  * hardware torque limit set on every servo at start
  * if no valid packet for `packet_timeout_s`, torque is switched off (springs open the fingers)
  * torque is switched off for a servo above `temp_cutoff_c`, and on exit / Ctrl-C

Run from the software/ folder:   python teleop_receiver.py [--no-servos]
--no-servos runs the network + logging path with no hardware attached.
"""

import argparse
import csv
import json
import socket
import time
from pathlib import Path

from feetech_bus import BusError, FeetechBus

CONFIG = Path(__file__).with_name("config.json")
CHANNELS = ("idx", "mid", "thumb", "base")


def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x


def parse_packet(data: bytes):
    """Return (seq, t_ms, {channel: 0..1}) or None if malformed."""
    try:
        parts = data.decode("ascii").strip().split(",")
        if len(parts) != 2 + len(CHANNELS):
            return None
        seq, t_ms = int(parts[0]), float(parts[1])
        vals = [float(v) for v in parts[2:]]
        if any(v != v for v in vals):  # NaN
            return None
        return seq, t_ms, {c: clamp01(v) for c, v in zip(CHANNELS, vals)}
    except (ValueError, UnicodeDecodeError):
        return None


def to_position(norm: float, open_pos: int, closed_pos: int) -> int:
    return int(round(open_pos + clamp01(norm) * (closed_pos - open_pos)))


def rate_limit(current: float, target: float, max_step: float) -> float:
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + max_step if delta > 0 else current - max_step


def main():
    cfg = json.loads(CONFIG.read_text())
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-servos", action="store_true")
    ap.add_argument("--port", default=cfg["serial_port"])
    args = ap.parse_args()

    ch_cfg = cfg["channels"]
    ids = {c: ch_cfg[c]["id"] for c in CHANNELS}

    bus = None
    if not args.no_servos:
        bus = FeetechBus(args.port, cfg["baud"])
        for c in CHANNELS:
            if not bus.ping(ids[c]):
                raise SystemExit(f"Servo '{c}' (ID {ids[c]}) did not respond. Check wiring/power/config.")
            bus.set_torque_limit(ids[c], cfg["torque_limit"])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", cfg["udp_port"]))
    sock.settimeout(0.0)

    Path("logs").mkdir(exist_ok=True)
    log_path = Path("logs") / f"teleop_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log = open(log_path, "w", newline="")
    w = csv.writer(log)
    w.writerow(["t_pc", "pico_seq", "pico_t_ms"]
               + [f"{c}_cmd" for c in CHANNELS]
               + [f"{c}_{k}" for c in CHANNELS for k in ("pos", "load", "current_ma", "temp_c")])
    print(f"Listening on UDP {cfg['udp_port']}; logging to {log_path}", flush=True)

    dt = 1.0 / cfg["control_hz"]
    fb_every = max(1, round(cfg["control_hz"] / cfg["feedback_hz"]))
    max_step = cfg["max_rate_per_s"] * dt
    target = {c: 0.0 for c in CHANNELS}
    cmd = {c: 0.0 for c in CHANNELS}
    last_pkt = None  # monotonic time of last valid packet
    last_seq, last_t_ms = -1, 0.0
    torque_on = False
    hot = set()
    tick = 0
    next_t = time.perf_counter()

    try:
        while True:
            # Drain socket, keep only the newest valid packet.
            while True:
                try:
                    data, _ = sock.recvfrom(256)
                except (BlockingIOError, socket.timeout):
                    break
                pkt = parse_packet(data)
                if pkt and pkt[0] > last_seq:
                    last_seq, last_t_ms, target = pkt[0], pkt[1], pkt[2]
                    last_pkt = time.monotonic()

            alive = last_pkt is not None and (time.monotonic() - last_pkt) < cfg["packet_timeout_s"]

            if alive:
                for c in CHANNELS:
                    cmd[c] = rate_limit(cmd[c], target[c], max_step)
                if bus:
                    if not torque_on:
                        bus.set_torque_all([i for c, i in ids.items() if i not in hot], True)
                        torque_on = True
                    goals = {ids[c]: to_position(cmd[c], ch_cfg[c]["open_pos"], ch_cfg[c]["closed_pos"])
                             for c in CHANNELS if ids[c] not in hot}
                    bus.sync_goals(goals, cfg["servo_speed"], cfg["servo_acc"])
            elif torque_on:
                # Link lost: let go so the return springs open the fingers.
                if bus:
                    bus.set_torque_all(list(ids.values()), False)
                torque_on = False
                cmd = {c: 0.0 for c in CHANNELS}
                print("Packet timeout: torque OFF", flush=True)

            if tick % fb_every == 0:
                row = [f"{time.time():.4f}", last_seq, last_t_ms] + [f"{cmd[c]:.4f}" for c in CHANNELS]
                for c in CHANNELS:
                    if bus:
                        try:
                            st = bus.read_state(ids[c])
                            row += [st["pos"], st["load"], f"{st['current_ma']:.0f}", st["temp_c"]]
                            if st["temp_c"] >= cfg["temp_cutoff_c"] and ids[c] not in hot:
                                hot.add(ids[c])
                                bus.set_torque(ids[c], False)
                                print(f"{c}: {st['temp_c']}C over limit, torque OFF for this servo")
                        except BusError:
                            row += ["", "", "", ""]
                    else:
                        row += ["", "", "", ""]
                w.writerow(row)
                log.flush()

            tick += 1
            next_t += dt
            time.sleep(max(0.0, next_t - time.perf_counter()))
    except KeyboardInterrupt:
        pass
    finally:
        if bus:
            try:
                bus.set_torque_all(list(ids.values()), False)
            except Exception:
                pass
            bus.close()
        log.close()
        print("\nStopped; torque off.")


if __name__ == "__main__":
    main()
