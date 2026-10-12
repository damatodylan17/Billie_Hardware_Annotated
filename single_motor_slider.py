'''
single_motor_slider.py 
Allows user to control each of the one specified motor 
on BILLIE with a keyboard.
W/S = +/- angle
Q = Quit

To be able to control all four motors at once, see multi_motor_slider.py
'''

# sys provides access to standard input (the terminal keyboard stream).
import sys
# termios reads and restores terminal settings on Unix-like systems such as Linux.
import termios
# tty switches the terminal into a mode that accepts individual keypresses.
import tty
# time.sleep() pauses execution to allow time for motor feedback to arrive.
import time
# CyberGear is the motor interface defined in cybergear.py in this repository.
# It handles CAN communication; this script supplies targets and reads feedback.
from cybergear import CyberGear

# Motor
# 0x means hexadecimal: 0x3 is decimal 3, the left hip on BILLIE.
# Other IDs: 0x1 = left knee, 0x2 = right knee, 0x4 = right hip.
MOTOR_ID = 0x3  # change motor here

# The dictionary maps the selected motor ID to its motor model name.
# Construction configures the Linux can0 interface and enables this motor.
# This happens immediately, before the main try/finally block below.
motor = CyberGear({
    MOTOR_ID: "Cybergear"
})

# How much the position changes per key press
# Units are radians: 0.01 rad is approximately 0.57 degrees per keypress.
# This is a target-position increment, not a commanded angular velocity.
STEP = 0.01


# Read one keyboard character
# This helper waits for one character, then returns it to the control loop.
def get_key():

    # Get the integer file descriptor identifying standard input.
    fd = sys.stdin.fileno()
    # Save the current terminal configuration so it can be restored afterward.
    old_settings = termios.tcgetattr(fd)

    try:
        # Raw mode delivers keys immediately, without waiting for Enter.
        # It also disables normal terminal processing, including usual Ctrl+C
        # handling while this read is active; Q is the intended quit key.
        tty.setraw(fd)
        # Read exactly one character. Execution pauses here until input arrives.
        key = sys.stdin.read(1)
    finally:
        # This finally block restores the terminal even if reading raises an error.
        # TCSADRAIN waits for pending terminal output before applying the settings.
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings
        )

    # Send the character back to the statement key = get_key() below.
    return key


# The main try/finally pairs normal operation with motor cleanup.
# This script has no __main__ guard, so importing it also runs its top-level code.
# Main
try:

    # Send an enable command again; the CyberGear constructor already enables it.
    # Enabling allows motion commands but does not specify a new position target.
    motor.enable_for_motion()

    print("======================================")
    print("     CyberGear Motor Position Test")
    print("======================================")
    print()
    # hex() displays the motor ID in hexadecimal, such as 0x3.
    print("Motor ID:", hex(MOTOR_ID))
    print()
    print("Controls:")
    print("  W = increase position")
    print("  S = decrease position")
    print("  Q = quit")
    print()
    print(f"Step size: {STEP} rad")
    print()

    # Read current motor position
    # Allow 0.2 seconds for feedback; this delay does not guarantee its arrival.
    time.sleep(0.2)

    # Return the latest cached MotorData object, updated by a background CAN listener.
    # This call does not wait for a new measurement from the motor.
    data = motor.get_data(MOTOR_ID)

    # Check whether a MotorData object exists for this ID.
    # An existing object can still have current_position=None before feedback arrives;
    # the original code does not separately check for that condition.
    if data is None:
        raise RuntimeError("Could not read motor position.")

    # Start the target at the measured angle instead of assuming the motor is at zero.
    # After this initialization, position stores the commanded target; it is not
    # overwritten by later measured positions.
    position = data.current_position

    # An f-string inserts values into text; :.4f displays four decimal places.
    print(f"Current motor position: {position:.4f} rad")
    print()
    print("Starting from current position.")
    print()


    # Main control loop
    # Repeat until Q executes break, or an exception leaves the loop.
    # Since get_key() blocks, this is an input-driven loop, not a fixed-rate loop.
    while True:
        # Wait for the next keypress. Feedback is printed only after a key returns.
        key = get_key()

        # Increase position
        # lower() makes uppercase W and lowercase w behave identically.
        if key.lower() == "w":

            # Increase the previous target by STEP: position = position + STEP.
            # This script does not enforce robot-specific mechanical joint limits.
            position += STEP

            # Send the target angle and local motor feedback gains over CAN.
            # kp sets position-error correction; kd provides velocity damping.
            # Conceptually: torque = kp*(target - measured angle) - kd*velocity.
            # This wrapper sends zero target velocity and zero added torque.
            # Sending the command does not wait for the motor to reach the target.
            motor.set_position(
                MOTOR_ID,
                position=position,
                kp=3,
                kd=0.2
            )

            print()
            print("W pressed")
            print(f"Commanded: {position:.4f} rad")

        # Decrease position
        elif key.lower() == "s":

            # Decrease the previous target by STEP: position = position - STEP.
            position -= STEP

            # Send the target angle and local motor feedback gains over CAN.
            # kp sets position-error correction; kd provides velocity damping.
            # Conceptually: torque = kp*(target - measured angle) - kd*velocity.
            # This wrapper sends zero target velocity and zero added torque.
            # Sending the command does not wait for the motor to reach the target.
            motor.set_position(
                MOTOR_ID,
                position=position,
                kp=3,
                kd=0.2
            )

            print()
            print("S pressed")
            print(f"Commanded: {position:.4f} rad")

        # Quit
        elif key.lower() == "q":

            print()
            print("Q pressed.")
            print("Stopping motor...")
            # Exit the loop immediately, skip the feedback print below, and
            # continue to the outer finally block to disable the motor.
            break

        # ----------------------------------------------------
        # Read motor state
        # Read the newest cached feedback after handling this key.
        # It may still describe a time before the most recent command.
        data = motor.get_data(MOTOR_ID)

        # If a MotorData object exists, display its reported physical state.
        # Actual position can differ from the target while the motor moves or is loaded.
        # As above, this does not verify that each reported field is populated.
        if data:

            print(
                f"Actual: {data.current_position:.4f} rad | "
                f"Velocity: {data.current_velocity:.4f} rad/s | "
                f"Torque: {data.current_torque:.4f} Nm"
            )

finally:

    print()
    print("Cleaning up...")
    # Send a disable command to the selected motor. This does not command it to zero.
    # The outer finally runs on normal Q exit and exceptions within its try block;
    # it cannot handle failures before that block or a forced process termination.
    motor.cleanup()
    print("Motor disabled.")