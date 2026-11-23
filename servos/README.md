# Servos

Setup notes for 3x Feetech STS3215 bus servos (via URT-2 adapter) plus one micro servo, driven by an ESP32.

## Wiring

- **Servo power:** 12V supply into the URT-2 power terminals (not USB, not the ESP32). Budget ~2A per STS3215 under load; a 12V 5-6A supply is comfortable for three. Servos daisy-chain off the TTL-BUS connector.
- **ESP32 <-> URT-2 UART header:**
  - G -> GND
  - ESP32 TX (GPIO17) -> URT TX
  - ESP32 RX (GPIO16) -> URT RX
  - Feetech quirk: MCU TX goes to adapter TX, RX to RX (not crossed). If the servos don't respond, swap the two wires (safe).
- Set the URT-2 signal level to **3.3V** (ESP32 is not 5V-tolerant).
- **Unplug the URT-2 USB cable** while the ESP32 drives it, so the onboard USB chip doesn't fight the bus.
- **Micro servo (SG90/MG90 style PWM):** signal -> GPIO18, powered from a separate 5V supply, with that supply's ground tied to ESP32 ground. Not on the bus.

## Step 1: Unique servo IDs

Every STS3215 ships as ID 1 at 1,000,000 baud. Each needs a unique ID before chaining (we use IDs 1, 2, 3). Either:
- Use Feetech's FD software on a PC over the URT-2 USB, or
- Use an ID-setting sketch: connect one servo at a time, set `NEW_ID`, upload.

## Step 2: Main sketch

Ping each servo, then sweep the three bus servos together with a sync write, along with the micro servo. STS3215 positions run 0-4095 over 360 degrees (2048 = center).

Libraries:
- Feetech **SCServo / FTServo** (provides `SMS_STS`), from Library Manager or Feetech's GitHub
- **ESP32Servo** for the micro servo

## MicroPython workflow (Thonny)

Planned files (save onto the ESP32 via Thonny):
- `sts.py` - servo bus driver (ping, move, sync move, read position/load/temp, change ID)
- `set_id.py` - assigns unique IDs, one servo at a time; also scans the bus and prints IDs found
- `main.py` - three bus servos + PWM micro servo; runs on boot

Steps:
1. Flash MicroPython: Tools -> Options -> Interpreter -> "MicroPython (ESP32)" -> "Install or update MicroPython".
2. Save `sts.py` to the device (File -> Save as -> MicroPython device, same name).
3. With one servo connected, run `set_id.py` (green Run; no need to save to device). Change `NEW_ID` and repeat for IDs 1, 2, 3.
4. Chain all three servos, save `main.py` to the device under that exact name. Pressing Stop in Thonny turns servo torque off cleanly.

Main calls:
- `bus.move(id, position, speed, acc)` - move one servo
- `bus.sync_move(ids, positions)` - move several together
- `deg_to_pos(degrees)` - degrees to position, 0 deg = center
- `micro.angle(0-180)` - micro servo

The driver is untested on this hardware. If the ID scan finds nothing even after swapping TX/RX, raise `timeout_ms` to 50 in `STSBus(...)`.

## Troubleshooting ("NOT FOUND")

1. Is the 12V supply on and connected to the URT-2?
2. Are TX and RX swapped?
3. Does each servo have its unique ID?
4. Is the URT-2 USB cable unplugged?

## Open question

Is the micro servo a standard PWM one (SG90) or a Feetech serial micro (SCS0009)? A serial SCS servo uses a different protocol and shouldn't share the bus with STS servos.
