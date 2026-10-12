# LEARNING GUIDE: send disable commands to motors 1 through 4.
# Existing behavior: creating CyberGear first enables these motors, so this is
# not a disable-only CAN transaction. That behavior is preserved here.
# Import the repository motor wrapper; time supplies the delay below.
from cybergear import CyberGear
import time

# Addresses for left knee, right knee, left hip, and right hip, respectively.
MOTOR_IDS = [0x1, 0x2, 0x3, 0x4]

# Construct an interface to all four motors using the default Linux can0 bus.
motor = CyberGear(
    # A dictionary comprehension builds one ID-to-model entry for each list item.
    {motor_id: "Cybergear" for motor_id in MOTOR_IDS},
    verbose=True
)

# Wait half a second after construction before disabling.
time.sleep(0.5)

# Send a disable-motion CAN frame to each registered motor.
# No position target is sent by this method; the robot is not moved to zero.
motor.cleanup()

# This reports that the calls completed; the script does not verify a disable
# acknowledgment from every motor. All statements run immediately on import.
print("All four motors disabled")