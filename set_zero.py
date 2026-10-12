# LEARNING GUIDE: establish the encoder reference for all four joints.
# Place the robot in the intended reference pose before accepting the zero prompt.
# Zeroing records the current angle as zero; it does not move the robot to a pose.
# Motor creation and this script's top-level statements also execute on import.
# CyberGear provides CAN commands and a cache of reported motor states.
# time is used for short pauses between commands.
from cybergear import CyberGear
import time

# Motor IDs
# CAN addresses: LH = left hip, LK = left knee, RH = right hip, RK = right knee.
# The 0x prefix means hexadecimal; these IDs are decimal 3, 1, 4, and 2.
LH_MOTOR_ID = 0x3
LK_MOTOR_ID = 0x1
RH_MOTOR_ID = 0x4
RK_MOTOR_ID = 0x2

# This list sets the order in which the four zero commands are sent.
motor_ids = [LH_MOTOR_ID,LK_MOTOR_ID,RH_MOTOR_ID,RK_MOTOR_ID]

# Create CyberGear object
# The dictionary associates each address with its motor model.
# The constructor configures can0 and automatically enables the listed motors.
motors = CyberGear({
    LH_MOTOR_ID: "Cybergear",
    LK_MOTOR_ID: "Cybergear",
    RH_MOTOR_ID: "Cybergear",
    RK_MOTOR_ID: "Cybergear"})

# The matching finally block sends disable commands after normal exit or an
# exception in this block; construction above occurs outside that protection.
try:
    # This repeats the enable operation already performed by the constructor.
    motors.enable_for_motion()

    print("CyberGear motors enabled.")
    print()
    print("Put the robot in the desired STANDING position.")
    print("Make sure all four legs are positioned correctly.")
    print()

    # input() pauses here until Enter. The user chooses the physical reference pose;
    # there is no sensor-based check that the robot is standing correctly.
    input("Press ENTER to set all four motors to ZERO.")

    # Visit one motor at a time rather than broadcasting a single zero command.
    for motor_id in motor_ids:
        print(f"Setting motor {hex(motor_id)} zero...")
        # Send the zero-reference command, followed by a zero-torque command.
        # All later angle targets and readings are relative to this reference.
        motors.set_zero_position(motor_id)
        # Pause for 0.1 seconds before processing the next motor.
        time.sleep(0.1)

    print()
    print("All four motors zeroed!")
    print()
    input("Press ENTER to disable motors...")

finally:

    print("Cleaning up...")
    # Disable all motors registered in this CyberGear instance.
    motors.cleanup()
    print("Motors disabled.")