# LEARNING GUIDE: older single-leg inverse-kinematics squat experiment.
# Existing dependency: motor_controller.py is not present in this branch, so the
# MotorController import cannot be satisfied by the project source alone.
# This code is preserved as written; its geometry and angle convention differ
# from squat_test.py. Top-level statements run when executed or imported.
# This expects a different motor API from cybergear.CyberGear.
# Its behavior cannot be verified from the files in this repository.
from motor_controller import MotorController

# NumPy supplies trigonometry; time supplies elapsed time and short delays.
import numpy as np
import time


# Motor IDs
# Choose left hip ID 3 and left knee ID 1 (0x denotes hexadecimal).
HIP_MOTOR_ID = 0x03
KNEE_MOTOR_ID = 0x01


# Create motor controller
# Construct the external controller with a list of motor IDs.
motors = MotorController([HIP_MOTOR_ID,KNEE_MOTOR_ID])


# Link lengths
# Two link lengths, in meters. l2 here is 0.213 m, unlike the 0.221 m
# value in squat_test.py; the values have not been reconciled.
l1 = 0.183
l2 = 0.213


# Convert a desired planar endpoint (x=0, y=y_des) into two joint angles,
# then immediately send those angles to the motor controller.
def hip_inverse_kinematics(y_des):

    # This rejects negative/overextended distances, but does not check the
    # inner reach limit abs(l1-l2). Clipping below can mask unreachable inputs.
    if y_des < 0 or y_des > l1 + l2:
        raise ValueError(
            "Desired hip position out of reachable range"
        )

    # Calculate theta2 using the law of cosines
    # Law of cosines: cos(knee) = (distance^2-l1^2-l2^2)/(2*l1*l2).
    cos_theta2 = (y_des**2 - l1**2 - l2**2) / (2 * l1 * l2)
    # arccos selects one bending branch; clip keeps roundoff within [-1,1].
    theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))  # Clip to avoid errors due to rounding

    # Calculate theta1 based on theta2 and y_des
    # Subtract the triangle's internal angle from the target direction.
    # arctan2(y,x) preserves quadrant information; angles are in radians.
    theta1 = np.arctan2(y_des, 0) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))

    # Send the calculated angles to the motors
    # Send hip/knee targets with position gain 3 and damping gain 0.1.
    # The exact implementation is in the missing MotorController dependency.
    motors.set_position(HIP_MOTOR_ID, theta1, kp=3, kd=0.1)
    motors.set_position(KNEE_MOTOR_ID, theta2, kp=3, kd=0.1)


# Generate a vertical cosine motion for duration seconds. frequency is in
# Hz; amplitude and offset are meters. Defaults request a 5-second motion.
def track_sinusoid(duration=5, frequency=0.5, amplitude=0.05, offset=0.03):
    start_time = time.time()
    # Compare elapsed wall-clock time against the requested duration.
    while time.time() - start_time < duration:
        t = time.time() - start_time
        # Center the height at l1+l2-amplitude-offset and vary it by +/-amplitude.
        # For the defaults, height ranges from 0.266 to 0.366 m.
        hip_position = amplitude * np.cos(2 * np.pi * frequency * t) + (l1 + l2 - amplitude - offset)
        # Convert this time's target height to angles and command the two joints.
        hip_inverse_kinematics(hip_position)
        # Wait 10 ms between iterations; computation and communication add time.
        time.sleep(0.01)


# Begin the default motion immediately; this script has no Enter prompt.
try:
    track_sinusoid()

finally:

    # Call the external controller's cleanup method when the protected run exits.
    motors.cleanup()