# LEARNING GUIDE: turn a desired squat height into four joint targets.
# Inverse kinematics converts height to hip/knee angles; motor-local PD tracks
# those targets. This is prescribed motion, not a balance or height-feedback loop.
# The logger compares commanded joint angles with reported joint angles.
# All setup and the final try block execute at module scope, including on import.
'''
squat_test.py

Tests a squat motion for the biped BILLIE.
--> The desired height (z) is given, and inverse kinematics is used to 
    develop joint angles for each of the motors.
--> When power-cycling the system, remember to set_zero.py the 
'''

# CyberGear sends motor commands; NumPy supplies array math and trigonometry.
# time is used for trajectory timing and short pauses.
from cybergear import CyberGear
import numpy as np
import time
import matplotlib
# Use the noninteractive Agg backend to save plots without a GUI window.
# os builds directories; datetime gives each run a timestamp.
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import os
from datetime import datetime

# Motor IDs
# Addresses: left hip=3, left knee=1, right hip=4, right knee=2.
LH_MOTOR_ID = 0x3
LK_MOTOR_ID = 0x1

RH_MOTOR_ID = 0x4
RK_MOTOR_ID = 0x2

# Motor-level position gain and velocity damping; these are passed into
# set_position, not applied in a separate Python torque controller.
KP = 10.0
KD = 0.1

# Create CyberGear object
# Initialize the Linux CAN interface and enable the four registered motors.
motors = CyberGear({LH_MOTOR_ID: "Cybergear", LK_MOTOR_ID: "Cybergear",
                RH_MOTOR_ID: "Cybergear", RK_MOTOR_ID: "Cybergear"})

# Joint order here is [LK,LH,RH,RK]. The values match indices 1..4 of the
# five-coordinate model; the start-pose loop uses dictionary key order.
STATE_IDX = {LK_MOTOR_ID: 1, LH_MOTOR_ID: 2, RH_MOTOR_ID: 3, RK_MOTOR_ID: 4}

# Link lengths
# Link lengths in meters for this two-link squat geometry. They are
# independent constants, not loaded from the repository's URDF.
l1 = 0.183
l2 = 0.221

# Straight-leg length: 0.183 + 0.221 = 0.404 m.
standing_height = l1 + l2   # 0.404 m

# Inverse kinematics
# Input: desired leg endpoint distance z_des in meters.
# Output: hip angle and negative left-knee bending angle in radians.
def hip_inverse_kinematics(z_des):
    # Check reachable range
    # Existing check covers nonpositive and overly long distances. Full two-link
    # reachability also requires z_des >= abs(l1-l2), not checked here.
    if z_des <= 0 or z_des > l1 + l2:
        raise ValueError(f"z_des={z_des} is outside reachable range")

    # Hip
    # Use the law of cosines for the angle between the first link and the
    # vertical endpoint direction: (l1^2+z^2-l2^2)/(2*l1*z).
    cos_theta1 = (l2**2 - l1**2 - z_des**2) / (-2 * l1 * z_des)
    # Clamp the cosine to [-1,1] before arccos to prevent roundoff-domain errors.
    theta1 = np.arccos(np.clip(cos_theta1, -1.0, 1.0))
    hip_angle = theta1

    # Knee
    # Find the triangle's internal knee angle. A straight leg has theta2=pi.
    cos_theta2 = (z_des**2 - l1**2 - l2**2) / (-2 * l1 * l2)
    theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))
    # Convert the internal triangle angle to the chosen joint convention:
    # straight knee=0; bending the left knee gives a negative value.
    knee_angle = -(np.pi - theta2)

    return hip_angle, knee_angle


# Command both legs
# Compute one pair of angles and apply mirrored signs to the other leg.
def command_z(z_des):
    hip_angle, knee_angle = hip_inverse_kinematics(z_des)

    # Left leg
    # Position targets use radians; set_position requests zero target velocity
    # and supplies KP/KD for the motor's local feedback loop.
    motors.set_position(LH_MOTOR_ID,hip_angle,kp=KP,kd=KD)
    motors.set_position(LK_MOTOR_ID,knee_angle,kp=KP,kd=KD)

    # Right leg
    # The right-side angle signs are reversed to match this script's convention.
    motors.set_position(RH_MOTOR_ID,-hip_angle,kp=KP,kd=KD)
    motors.set_position(RK_MOTOR_ID,-knee_angle,kp=KP,kd=KD)

# plots the squat, actual vs desired position
# Collect samples in memory and save one desired-versus-actual plot per joint.
class SquatLogger:
    # Start empty lists and create a timestamped output folder under plots/.
    def __init__(self):
        self.time = []
        self.desired = []
        self.actual = []

        # Create timestamped folder
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.plot_dir = os.path.join("plots", f"squat_{timestamp}")
        os.makedirs(self.plot_dir, exist_ok=True)

        print(f"Saving squat plot to: {self.plot_dir}")

    # Append copies of arrays so later changes cannot alter saved samples.
    def log(self, t, desired, actual):
        self.time.append(t)
        self.desired.append(np.asarray(desired).copy())
        self.actual.append(np.asarray(actual).copy())

    # Convert accumulated lists into arrays: rows are time samples, columns joints.
    def plot(self):
        time_data = np.asarray(self.time)
        # Keep the logger's target vector in [left knee,left hip,right hip,right knee]
        # order, including the right-side sign reversals.
        desired = np.asarray(self.desired)
        actual = np.asarray(self.actual)

        # Names follow the exact [LK,LH,RH,RK] order used when collecting data.
        joint_names = ["Left Knee","Left Hip", "Right Hip","Right Knee"]

        # Create a separate plot for each joint; [:,i] selects all samples for it.
        for i, name in enumerate(joint_names):
            plt.figure(figsize=(10, 4))

            plt.plot(time_data,desired[:, i],label="Desired")

            plt.plot(time_data,actual[:, i],"--",label="Actual")

            plt.xlabel("Time [s]")
            plt.ylabel("Position [rad]")
            plt.title(f"{name} Position")
            plt.grid()
            plt.legend()

            # Turn a display name into a filename by lowercasing and replacing spaces.
            filename = f"{name.lower().replace(' ', '_')}.png"
            filepath = os.path.join(self.plot_dir, filename)

            # Write a PNG rather than opening a window; close() then frees the figure.
            plt.savefig(filepath,dpi=150,bbox_inches="tight")

            plt.close()
            print(f"Saved: {filepath}")

# Read the most recent cached state of each motor, then assemble four angles.
# These are not guaranteed to be simultaneous or newly received measurements.
# This function does not check for missing feedback before accessing fields.
def get_actual_positions():
    lk_data = motors.get_data(LK_MOTOR_ID)
    lh_data = motors.get_data(LH_MOTOR_ID)
    rh_data = motors.get_data(RH_MOTOR_ID)
    rk_data = motors.get_data(RK_MOTOR_ID)

    # Match STATE_IDX ordering:
    # [LK, LH, RH, RK]
    actual = np.array([
        lk_data.current_position,
        lh_data.current_position,
        rh_data.current_position,
        rk_data.current_position
    ])

    return actual

# Sinusoidal squat
# Generate a sinusoidal target height for duration seconds. Units: frequency
# in Hz, amplitude in meters, angles from inverse kinematics in radians.
def squat_sinusoid(duration=10,frequency=0.2,amplitude=0.02, logger=None):
    start_time = time.time()
    while time.time() - start_time < duration:
        t = time.time() - start_time

        # 0.404 m = standing
        # At sin = 1: z_des = 0.404  (standing)
        # At sin = -1: z_des = 0.364  (squat)

        # z(t)=standing_height-amplitude+amplitude*sin(2*pi*frequency*t).
        # With amplitude=0.02, the range is 0.364..0.404 m; t=0 starts at 0.384 m.
        z_des = (standing_height - amplitude + amplitude * np.sin(
                2 * np.pi * frequency * t))

        # Calculate desired joint angles
        hip_angle, knee_angle = hip_inverse_kinematics(z_des)

        # Keep the logger's target vector in [left knee,left hip,right hip,right knee]
        # order, including the right-side sign reversals.
        desired = np.array([
            knee_angle,
            hip_angle,
            -hip_angle,
            -knee_angle
        ])

        # Send all four target angles for this time sample.
        command_z(z_des)

        # Get actual motor positions
        actual = get_actual_positions()

        # Logging is optional; run the same motion if no logger was supplied.
        if logger is not None:
            logger.log(t, desired, actual)

        print(f"z_des = {z_des:.4f} m")

        # Pause 10 ms; actual iteration spacing also includes computation and CAN I/O.
        time.sleep(0.01)


# Main
# Create the logger before the main try/finally run.
logger = SquatLogger()
try:
    motors.enable_for_motion()

    print("CyberGear motors enabled.")

    print()
    input("Press ENTER to command a start position.")
    # These are hard-coded initial joint angles in [LK,LH,RH,RK] order.
    # They are not calculated from the sinusoid's first desired height.
    start_pos = [-0.29718542, 0.07431855, 0.41929060, 0.07520059]
    # Pair each stored angle with a motor using STATE_IDX insertion order.
    for i, motor_id in enumerate(STATE_IDX):
        motors.set_position(motor_id, start_pos[i], kp=KP,kd=KD)

    print()
    input("Press ENTER to start the squat.")

    # Start sinusoidal squat
    print("\nStarting sinusoidal squat...")

    # Run for 15 seconds at 0.2 Hz: approximately three 5-second squat cycles.
    squat_sinusoid(duration=15,frequency=0.2,amplitude=0.02, logger=logger)

    print("\nSinusoidal squat finished.")

    # Save tracking plots after a successful motion; an earlier exception skips
    # this line but still enters the finally block below.
    logger.plot()

    input("Press ENTER to disable the motors...")

finally:

    print("\nCleaning up...")
    # Disable all four motors after the final prompt or an exception in the try.
    motors.cleanup()