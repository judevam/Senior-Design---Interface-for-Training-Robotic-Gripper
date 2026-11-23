"""Bench-test tool for Feetech STS3215 servos, no VR required.

Examples (run from the software/ folder):
    python bench_test.py scan                      # find which IDs respond
    python bench_test.py set-id 1 2                # change ID 1 -> 2 (ONE servo connected!)
    python bench_test.py read 1                    # live pos/load/current/temp
    python bench_test.py sweep 1 --lo 1800 --hi 2300   # slow sweep, small range
    python bench_test.py move 1 2048               # go to a position

Always start with a small range and a low torque limit, with NO tendon attached.
"""

import argparse
import json
import time
from pathlib import Path

from feetech_bus import BusError, FeetechBus

CONFIG = Path(__file__).with_name("config.json")


def load_cfg():
    return json.loads(CONFIG.read_text())


def main():
    cfg = load_cfg()
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=cfg["serial_port"])
    p.add_argument("--baud", type=int, default=cfg["baud"])
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    s = sub.add_parser("set-id"); s.add_argument("old", type=int); s.add_argument("new", type=int)
    s = sub.add_parser("read"); s.add_argument("id", type=int)
    s = sub.add_parser("move"); s.add_argument("id", type=int); s.add_argument("pos", type=int)
    s = sub.add_parser("sweep"); s.add_argument("id", type=int)
    s.add_argument("--lo", type=int, default=1900); s.add_argument("--hi", type=int, default=2200)
    s.add_argument("--period", type=float, default=3.0)
    for sp in sub.choices.values():
        sp.add_argument("--torque-limit", type=int, default=cfg["torque_limit"])
    args = p.parse_args()

    bus = FeetechBus(args.port, args.baud)
    try:
        if args.cmd == "scan":
            found = [i for i in range(1, 11) if bus.ping(i)]
            print("Responding IDs:", found or "none (check power, wiring, port, baud)")
        elif args.cmd == "set-id":
            bus.set_id(args.old, args.new)
            print(f"ID {args.old} -> {args.new}; ping: {bus.ping(args.new)}")
        elif args.cmd == "read":
            while True:
                st = bus.read_state(args.id)
                print(f"pos={st['pos']:4d} load={st['load']:5d} cur={st['current_ma']:7.1f}mA "
                      f"temp={st['temp_c']}C V={st['voltage_v']:.1f}", end="\r")
                time.sleep(0.05)
        elif args.cmd in ("move", "sweep"):
            bus.set_torque_limit(args.id, args.torque_limit)
            bus.set_torque(args.id, True)
            if args.cmd == "move":
                bus.sync_goals({args.id: args.pos}, cfg["servo_speed"], cfg["servo_acc"])
                time.sleep(1.0)
                print(bus.read_state(args.id))
            else:
                t0 = time.time()
                while True:
                    phase = ((time.time() - t0) % args.period) / args.period
                    tri = 1 - abs(2 * phase - 1)  # 0..1..0
                    target = args.lo + tri * (args.hi - args.lo)
                    bus.sync_goals({args.id: target}, cfg["servo_speed"], cfg["servo_acc"])
                    st = bus.read_state(args.id)
                    print(f"goal={target:6.0f} pos={st['pos']:4d} load={st['load']:5d} "
                          f"cur={st['current_ma']:7.1f}mA", end="\r")
                    time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    except BusError as e:
        print(f"\nBus error: {e}")
    finally:
        if args.cmd in ("move", "sweep"):
            try:
                bus.set_torque(args.id, False)
            except BusError:
                pass
        bus.close()


if __name__ == "__main__":
    main()
