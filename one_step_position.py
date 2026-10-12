# LEARNING GUIDE: inspect a saved trajectory one pose at a time.
# This version sends position targets and waits for Enter after every sample.
# It does not play the saved trajectory with its original timing or velocities.
# Motor setup and data loading are at module scope and also execute on import.
'''
Position control for a one-step controller (from Julie's code)
'''

# CyberGear commands motors; NumPy loads binary .npy trajectory arrays.
# time is imported, but its playback sleep below is commented out.
from cybergear import CyberGear
import numpy as np
import time

# Motor IDs
# Motor IDs: LH=3, LK=1, RH=4, RK=2; 0x denotes hexadecimal.
LH_MOTOR_ID = 0x3
LK_MOTOR_ID = 0x1

RH_MOTOR_ID = 0x4
RK_MOTOR_ID = 0x2

# Create CyberGear object
# Construction configures can0 and enables all four motors immediately.
motors = CyberGear({LH_MOTOR_ID: "Cybergear", LK_MOTOR_ID: "Cybergear",
                RH_MOTOR_ID: "Cybergear", RK_MOTOR_ID: "Cybergear"})

# Change file to load here
# joint_angles = np.load("Xsol_without-height.npy")  
# Load a 40x10 state array relative to the current working directory.
# Columns 0..4 are [ankle,LK,LH,RH,RK]; columns 5..9 are velocities.
# Only the four actuated position columns are used in this script.
joint_angles = np.load("Xsol_full_0.2_periodic_0.2_wheight.npy")
# joint_angles = np.load("Xsol_0.3_periodic_0.4.npy")

# Negate each actuated position column to change the angle sign convention.
# [:,i] means every row in column i. The loaded array changes in memory;
# the .npy file on disk is not rewritten.
joint_angles[:, 1] = - joint_angles[:, 1]
joint_angles[:, 2] = - joint_angles[:, 2]
joint_angles[:, 3] = - joint_angles[:, 3]
joint_angles[:, 4] = - joint_angles[:, 4]

# Despite the name, inputs are four joint angles, not a height z.
# Argument order is hip,knee for each leg, unlike the saved column order.
def command_z(lhip_angle, lknee_angle, rhip_angle, rknee_angle):
    # Left leg
    # Send angle targets with local position gain 3 and damping gain 0.2.
    # set_position supplies zero target velocity and zero added torque.
    motors.set_position(LH_MOTOR_ID, lhip_angle,kp=3,kd=0.2)
    motors.set_position(LK_MOTOR_ID, lknee_angle,kp=3,kd=0.2)

    # Right leg
    motors.set_position(RH_MOTOR_ID, rhip_angle,kp=3,kd=0.2)
    motors.set_position(RK_MOTOR_ID, rknee_angle,kp=3,kd=0.2)

# Reads the actual motor positions
# Return an ID-to-angle dictionary using the latest cached feedback.
# An existing MotorData object can still contain a None position field.
def get_actual_positions():
    # Read cached feedback immediately after sending; it may precede the command.
    actual = {}
    for motor_id in [LK_MOTOR_ID,LH_MOTOR_ID,RH_MOTOR_ID,RK_MOTOR_ID]:
        data = motors.get_data(motor_id)

        if data is not None:
            actual[motor_id] = data.current_position
        else:
            actual[motor_id] = None
    return actual

# Main
# Set an initial pose, wait for user confirmation, then visit each saved row.
def main():
    try:
        # Repeat the motor enable already performed during construction.
        motors.enable_for_motion()
        print("CyberGear motors enabled.")

        print()
        input("Press ENTER to start the step.")

        # The first row supplies the initial pose; index 0 is the passive ankle
        # and is skipped when extracting the four motor angles.
        row = joint_angles[0]
        lknee_angle = row[1]
        lhip_angle = row[2]
        rhip_angle = row[3]
        rknee_angle = row[4]
        command_z(lhip_angle, lknee_angle, rhip_angle, rknee_angle)

        # Read cached feedback immediately after sending; it may precede the command.
        actual = get_actual_positions()

        input("Press ENTER to start the step.")

        # Iterate saved samples, numbering them starting at 1. The first pose is
        # sent again because this loop includes row 0.
        for step, row in enumerate(joint_angles, start=1):
            lknee_angle = row[1]
            lhip_angle = row[2]
            rhip_angle = row[3]
            rknee_angle = row[4]

            command_z(lhip_angle, lknee_angle, rhip_angle, rknee_angle)
            # Collect the target angles in state order; this local list is not used later.
            commanded = [lknee_angle, lhip_angle, rhip_angle, rknee_angle]
            # Read cached feedback immediately after sending; it may precede the command.
            actual = get_actual_positions()
            
            print(f"commanded right knee: {rknee_angle}")
            print(f"actual right knee: {actual.get(2)}")
            # time.sleep(0.01)
            # Pause after each sample. This active prompt replaces timed playback;
            # the time.sleep line above is commented out.
            input("enter")


        
    # Existing behavior: the cleanup actions below are commented out, so this
    # block only prints a message on ordinary completion; it does not disable.
    finally:
        print("\nCleaning up...")
        # command_z(0,0,0,0)

        # motors.cleanup()


# Run main only when launched directly. Module-level motor creation still
# occurs when another script imports this file.
if __name__=="__main__":
    main()