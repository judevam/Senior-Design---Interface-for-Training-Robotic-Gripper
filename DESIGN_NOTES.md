# Design Notes: Motorized Tendon-Drive Hand Interface

Summary of the brainstorming conversation on the next prototype of the robotic interface for training a robotic gripper (UMI / DexUMI / YUBI style).

## Project Context

- Group is building a handheld robotic interface for data collection and imitation learning.
- The previous prototype was a fully mechanical tendon-drive 3-finger hand representing the Unitree Dex3-1. The human hand actuated a 3D-printed replica through a gauntlet that sticks out further than the human hand.
- **Next prototype goals:**
  - Drive each finger with a motor (considering Feetech 30 kg·cm serial bus servos) and put a servo beneath the thumb for base rotation.
  - More intuitive, stronger interface that also gives servo data (position, load, current).
  - Add camera mounting.
- **Later, if time allows:** use the Pico VR controller to control motor actuation and use the IMU inside the Pico.

## Target Robot: Unitree Dex3-1

- 3 fingers, 7 DoF: thumb 3, index 2, middle 2.
- Force-controlled, with a tactile edition (33 pressure sensors).
- Controlled over Unitree's own interface, not hobby PWM.

## Update (Nov 2026)

XRoboToolkit can supply the PICO data to the PC, replacing the custom Unity sender. Current path: PICO -> XRoboToolkit -> PC -> ESP32 -> Feetech servos. The Unity app and UDP tools were removed from the repo.

## Notes on the Google AI Conversation

The earlier chat proposed PICO controller to Unity to UDP over Wi-Fi to ESP32 to PWM servos. Caveats:

- It blurred the Dex3-1 (the target robot) with the replica hand (the human-side interface).
- It assumed hobby PWM servos. The plan now uses Feetech serial bus servos, so the ESP32 PWM code does not apply directly.
- A Pico controller only gives trigger, grip and joystick (few analog inputs) versus 7 DoF. Hand tracking gives richer per-finger data.
- Useful ideas to keep: UDP for low latency, normalized 0 to 1 finger commands, logging with timestamps.

## Control Concept

- Human holds a switch or control per finger. The motor pulls the tendon to curl the finger.
- Springs provide joint return.
- Thumb has an extra control for base rotation.
- Each finger has its own motor.

### Recommendation: decouple input from actuation

Firmware takes one normalized target (0 to 1) per finger plus a thumb base target. Any input source fills those values. The servo layer handles position control, ramp/velocity limits and current/load limits. Log command, measured position and load.

### Input options (simple to intuitive)

1. **On/off switches.** Simple. Add ramping (held switch ramps closed, release ramps open) for pseudo-proportional control.
2. **Analog triggers/sliders** (Hall-effect or potentiometer). True proportional but ergonomically hard with 4 to 5 controls in one hand.
3. **Grasp synergies.** One trigger closes all fingers along a preset curve with per-finger offsets. Thumb base stays separate.
4. **Leader-follower.** Sense the human's finger curl (Hall/potentiometer on the gauntlet or tendon travel) and have servos follow and amplify. Most intuitive, needs more mechanical design, gives human-hand signal as data.
5. **Pico controller (later).** Trigger to index, grip to middle, joystick to thumb. Ties up the second hand for a handheld device. IMU gives pose.

**Suggested path:** build option 1 with ramping now on the abstraction above, prototype option 3 or 4 next, add Pico later as another input source.

### Spring-return notes

- Servo only needs to pull. The spring opens the finger when the servo releases.
- Servo must be able to let the tendon go fully slack.
- Servo load rises as the finger closes, giving a rough force signal.

## Servo Selection: Feetech 30 kg·cm Serial Bus (12 V) + Driver Board

*Figures from memory for the common STS3215-type. Verify against the listing.*

- **Quantity:** pack of 2 is not enough. Need 4 (3 fingers plus thumb base), ideally 5 for a spare. One driver board is enough, since servos daisy-chain with unique IDs on a single bus.
- **Strength:** about 2.9 N·m. Tendon force equals torque divided by spool radius, so about 290 N at a 10 mm spool. Large margin versus the few to tens of newtons needed. The limit is tendon line strength, printed parts and springs, so add firmware current/torque limits.
- **Travel:** 360° (4096 counts). Size the spool (roughly 8 to 12 mm radius) for full finger curl.
- **Speed:** about 0.2 s per 60° at 12 V, which is sufficient.
- **Power:** 12 V supply. Stall current is a few amps per servo (up to about 10 A for four at once, though real use is far lower). For a battery, check the rated voltage range before using 3S LiPo.
- **Size and weight:** about 55 g each (roughly 45 × 25 × 35 mm). About 220 g for four. Check gauntlet fit and balance.
- **Data:** serial bus servos report position, load, current, temperature, which matches the data-collection goal.

## Open Questions

- Exact Feetech model and whether it is the serial bus type.
- Total servo count and whether one spool drives both joints of each finger.
- What reads inputs and drives servos (ESP32, Raspberry Pi, laptop) and whether it is wired or wireless.
- Power setup: tethered or battery on the gauntlet.
- Which fingers of the human hand are free to operate controls; is the hand inside the gauntlet or holding it?
- What the human feels versus sees of finger motion.
- What gets logged and at what rate; training targets (servo commands versus human pose) and the mapping to Dex3-1 joint angles.
- Cameras (GoPro, wrist RGB, depth), timestamp sync with servo data, and pose tracking (SLAM, tracker, IMU).
- Deliverables, deadline, and team software background.

## Possible Next Steps

- Spool size and tendon travel calculation.
- Firmware layout for 4 servos on one serial bus with the normalized-target abstraction.
- Logging pipeline with timestamps.
- Test that servo rest position leaves the tendon slack so springs open the fingers.
