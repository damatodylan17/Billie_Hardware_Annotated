# LEARNING GUIDE: an experimental saved-torque playback script.
# The array called joint_angles actually contains four torque columns from Usol.
# Xsol supplies the initial pose. Commands here are open-loop torque playback,
# not the feedback controller used by billie_hardware.py.
# Setup and file loading occur on import as well as direct execution.
# CyberGear supplies position/torque commands; NumPy loads arrays; time
# provides the delay between torque samples.
from cybergear import CyberGear
import numpy as np
import time

# Motor IDs
# Motor IDs: left hip=3, left knee=1, right hip=4, right knee=2.
LH_MOTOR_ID = 0x3
LK_MOTOR_ID = 0x1

RH_MOTOR_ID = 0x4
RK_MOTOR_ID = 0x2

# Create CyberGear object
# Constructing the interface opens can0 and enables the registered motors.
motors = CyberGear({LH_MOTOR_ID: "Cybergear", LK_MOTOR_ID: "Cybergear",
                RH_MOTOR_ID: "Cybergear", RK_MOTOR_ID: "Cybergear"})


# Usol_without-height.npy is a 20x4 array used here as [LK,LH,RH,RK]
# torque samples, despite the variable name joint_angles.
joint_angles = np.load("Usol_without-height.npy")  
# Xsol is 20x10: five positions followed by five velocities.
# Only its first row and four actuated position columns set the initial pose.
initial_joint_angles = np.load("Xsol_without-height.npy")  
# joint_angles[:, 1] = - joint_angles[:, 1]
# joint_angles[:, 2] = - joint_angles[:, 2]
# joint_angles[:, 3] = - joint_angles[:, 3]
# joint_angles[:, 4] = - joint_angles[:, 4]
# Negate all four torque columns to switch the input sign convention.
joint_angles[:, 0] = - joint_angles[:, 0]
joint_angles[:, 1] = - joint_angles[:, 1]
joint_angles[:, 2] = - joint_angles[:, 2]
joint_angles[:, 3] = - joint_angles[:, 3]

# Negate the four actuated position columns; leave ankle and velocity
# columns unchanged. These edits affect loaded arrays, not saved files.
initial_joint_angles[:, 1] = - initial_joint_angles[:, 1]
initial_joint_angles[:, 2] = - initial_joint_angles[:, 2]
initial_joint_angles[:, 3] = - initial_joint_angles[:, 3]
initial_joint_angles[:, 4] = - initial_joint_angles[:, 4]


# The parameter names say angle, but this function sends torque values.
# The commented alternatives inside are inactive experiments.
def command_z(lhip_angle, lknee_angle, rhip_angle, rknee_angle):
    # # Left leg
    # motors.set_position(LH_MOTOR_ID, lhip_angle,kp=3,kd=0.2)
    # motors.set_position(LK_MOTOR_ID, lknee_angle,kp=3,kd=0.2)

    # # # Right leg
    # motors.set_position(RH_MOTOR_ID, rhip_angle,kp=3,kd=0.2)
    # motors.set_position(RK_MOTOR_ID, rknee_angle,kp=3,kd=0.2)

    # start_time = time.perf_counter()
    # motors.set_torque(LK_MOTOR_ID, lknee_angle)
    # end_time = time.perf_counter()
    # elapsed_time = start_time - end_time
    # print(f"elapsed time: {elapsed_time}")

    # Existing behavior: left knee always receives zero torque here.
    # The supplied lknee_angle argument is ignored by the active commands.
    motors.set_torque(LK_MOTOR_ID, 0)
    # Send the other three torque values in N*m; set_torque uses zero kp/kd,
    # so these commands contain no position-holding feedback term.
    motors.set_torque(LH_MOTOR_ID, lhip_angle)
    motors.set_torque(RK_MOTOR_ID, rknee_angle)
    motors.set_torque(RH_MOTOR_ID, rhip_angle)

# Use motor-local position control only for the initial pose, with kp=3
# and kd=0.2. This function interprets its arguments as angles in radians.
def position_command_z(lhip_angle, lknee_angle, rhip_angle, rknee_angle):
    # # Left leg
    motors.set_position(LH_MOTOR_ID, lhip_angle,kp=3,kd=0.2)
    motors.set_position(LK_MOTOR_ID, lknee_angle,kp=3,kd=0.2)

    # # # Right leg
    motors.set_position(RH_MOTOR_ID, rhip_angle,kp=3,kd=0.2)
    motors.set_position(RK_MOTOR_ID, rknee_angle,kp=3,kd=0.2)

# Reads the actual motor positions
# Build a dictionary of cached motor angles. The calls to this helper in
# the playback loop are currently commented out.
def get_actual_positions():
    actual = {}
    for motor_id in [LK_MOTOR_ID,LH_MOTOR_ID,RH_MOTOR_ID,RK_MOTOR_ID]:
        data = motors.get_data(motor_id)

        if data is not None:
            actual[motor_id] = data.current_position
        else:
            actual[motor_id] = None
    return actual

# Main
# After prompts, command the first Xsol pose and then play Usol samples.
def main():
    try:
        # Repeat the constructor's enable operation.
        motors.enable_for_motion()
        print("CyberGear motors enabled.")

        print()
        input("Press ENTER to start the step.")

        # row = joint_angles[0]
        # lknee_angle = row[1]
        # lhip_angle = row[2]
        # rhip_angle = row[3]
        # rknee_angle = row[4]
        # command_z(lhip_angle, lknee_angle, rhip_angle, rknee_angle)

        # Extract initial angles in saved state order, skipping passive ankle index 0.
        row = initial_joint_angles[0]
        lknee_angle = row[1]
        lhip_angle = row[2]
        rhip_angle = row[3]
        rknee_angle = row[4]
        position_command_z(lhip_angle, lknee_angle, rhip_angle, rknee_angle)

        input("Press ENTER to start the step.")

        # Read each torque row in order. step is a display counter starting at 1.
        for step, row in enumerate(joint_angles, start=1):
            # lknee_angle = row[1]
            # lhip_angle = row[2]
            # rhip_angle = row[3]
            # rknee_angle = row[4]

            # These four angle-named variables now hold torque values, not positions.
            lknee_angle = row[0]
            lhip_angle = row[1]
            rhip_angle = row[2]
            rknee_angle = row[3]

            command_z(lhip_angle, lknee_angle, rhip_angle, rknee_angle)
            # The printed left-knee value is the saved reference; command_z actually
            # sends zero to this motor, so this is not the torque sent.
            print(f"l knee torque command: {lknee_angle}")

            # Read actual motor positions
            # actual = get_actual_positions()
            
            # print(f"----------------- Step {step} ----------------------")
            # print("Commanded Angle")
            # print(f"LH: {lhip_angle}")
            # print(f"LK: {lknee_angle}")
            # print(f"RH: {rhip_angle}")
            # print(f"RK: {rknee_angle}")

            # print()

            # print("Actual Angle")
            # print(f"LH: {actual[LH_MOTOR_ID]}")
            # print(f"LK: {actual[LK_MOTOR_ID]}")
            # print(f"RH: {actual[RH_MOTOR_ID]}")
            # print(f"RK: {actual[RK_MOTOR_ID]}")

            # print()

            # Pause 12 ms after each row; processing time adds to that interval.
            # This timing is set here, not read from the saved trajectory file.
            time.sleep(0.012)
            # input("Enter")
        
    finally:
        print("\nCleaning up...")
        # On leaving the protected block, request zero torque at all four joints.
        # The following cleanup call is commented out: zero torque is not disable.
        command_z(0,0,0,0)

        # motors.cleanup()


# Execute main only when run directly; top-level motor setup still occurs
# if this module is imported.
if __name__=="__main__":
    main()