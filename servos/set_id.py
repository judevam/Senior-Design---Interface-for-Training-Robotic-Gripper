# set_id.py — run from Thonny with ONLY ONE servo connected to the bus.
# New STS3215s all ship as ID 1 at 1,000,000 baud.
from sts import STSBus

OLD_ID = 1
NEW_ID = 2      # change for each servo: 1, 2, 3

bus = STSBus(tx=17, rx=16)

print("Scanning bus...")
found = [i for i in range(0, 20) if bus.ping(i)]
print("Found IDs:", found)

if OLD_ID not in found:
    print("No servo at ID", OLD_ID, "- check 12V power, wiring, try swapping TX/RX.")
elif OLD_ID == NEW_ID:
    print("Servo already has ID", NEW_ID)
elif bus.change_id(OLD_ID, NEW_ID):
    print("Success: servo is now ID", NEW_ID)
else:
    print("ID change failed.")
