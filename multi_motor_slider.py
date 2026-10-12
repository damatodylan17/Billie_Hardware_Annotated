# LEARNING GUIDE: manually command four joints from a terminal dashboard.
# Each key changes one target by 0.01 rad (about 0.57 degrees). All four targets
# are resent each loop. This uses curses and the Linux SocketCAN motor wrapper.
# Motor setup is at module scope and therefore also occurs if the file is imported.
"""
multi_motor_slider.py

Allows user to control each of the four motors on BILLIE with a keyboard.
W/S = Motor 1 (Left Knee)
E/D = Motor 2 (Right Knee)
R/F = Motor 3 (Left Hip)
T/G = Motor 4 (Right Hip)
Q = Quit

"""
# curses draws a terminal interface and polls keys without waiting for Enter.
# time supplies loop delays; CyberGear handles motor CAN communication.
import curses
import time
from cybergear import CyberGear


# Motor configuration
# CAN ID to label mapping: LK/RK = left/right knee; LH/RH = left/right hip.
# Dictionary insertion order also determines the display order.
MOTORS = {0x1: "LK", 0x2: "RK", 0x3: "LH", 0x4: "RH",} 

# Target increments are angles, not speeds. KP controls position correction;
# KD damps velocity through the motor's local feedback loop.
STEP = 0.01 # radians per key press
KP = 5
KD = 0.2

# This dictionary comprehension assigns the Cybergear model to each ID.
# Construction opens can0 and immediately enables the four motors.
motor = CyberGear({motor_id: "Cybergear" for motor_id in MOTORS})


# Draw slider
# Draw one joint: a text bar showing the commanded angle, plus a numeric
# actual-angle readout. stdscr is the curses screen; row sets its location.
def draw_slider(stdscr, row, name, commanded, actual):
    # These limits scale only the display. They do not limit motor commands.
    min_pos = -2.0
    max_pos = 2.0

    width = 50

    # Convert commanded position to slider location
    # Map the displayed interval [-2,2] rad onto the normalized interval [0,1].
    normalized = (commanded - min_pos) / (max_pos - min_pos)

    # Clamp only the marker location so it remains within the drawn bar.
    normalized = max(0.0, min(1.0, normalized))

    # Convert normalized position to an integer character location, from 0 to 49.
    cursor = int(normalized * (width - 1))

    # Build the bar as a list of characters and place O at the target location.
    slider = ["-"] * width
    slider[cursor] = "O"

    # Write text at a screen row/column. Formatting such as +.3f includes a
    # plus/minus sign and three decimal places; it does not affect the command.
    stdscr.addstr(
        row,
        0,
        f"{name:>2}  "
        f"[{''.join(slider)}] "
        f"{commanded:+.3f}"
    )

    # Write text at a screen row/column. Formatting such as +.3f includes a
    # plus/minus sign and three decimal places; it does not affect the command.
    stdscr.addstr(
        row + 1,
        6,
        f"Actual: {actual:+.3f} rad"
    )


# Main
# curses.wrapper supplies the initialized screen when calling this function.
def main(stdscr):

    # Don't wait for keyboard input
    # Make getch() return -1 when no key is ready, allowing continuous updates.
    stdscr.nodelay(True)

    # Hide cursor
    # Hide the blinking text cursor while drawing the dashboard.
    curses.curs_set(0)

    # Enable motors
    # Repeat the constructor's enable command, then allow time for feedback.
    motor.enable_for_motion()
    # Pause 10 ms. Work above adds time, so the loop is not exactly 100 Hz.
    time.sleep(0.2)

    # Read current positions
    # Keep separate dictionaries for target angles and measured angles.
    commanded = {}
    actual = {}

    # Resend targets for every motor, even when no new key was pressed.
    # set_position sends a zero-velocity target with the selected KP and KD.
    for motor_id in MOTORS:
        # Read the latest cached state; this does not wait for new feedback.
        data = motor.get_data(motor_id)

        # Fallback only covers a missing object. A MotorData object can exist with
        # current_position=None before feedback arrives; that case is not handled.
        if data is None:
            commanded[motor_id] = 0.0
            actual[motor_id] = 0.0

        else:
            # Initialize the target from the current angle to avoid assuming a zero pose.
            commanded[motor_id] = data.current_position
            actual[motor_id] = data.current_position

    # Keyboard mapping
    # Map each key to (motor ID, target increment). Paired keys move in opposite
    # angle directions according to that motor's installed sign convention.
    key_map = {
        # Motor 1
        "w": (0x1, +STEP),
        "s": (0x1, -STEP),

        # Motor 2
        "e": (0x2, +STEP),
        "d": (0x2, -STEP),

        # Motor 3
        "r": (0x3, +STEP),
        "f": (0x3, -STEP),

        # Motor 4
        "t": (0x4, +STEP),
        "g": (0x4, -STEP),
    }

    # Main loop
    # The finally paired with this try disables motors on loop exit.
    # Initialization earlier in main occurs before this protected block.
    try:

        while True:

            # Read keyboard
            # Poll one key code; -1 means no input, not an instruction to stop.
            key = stdscr.getch()

            if key != -1:

                # The finally paired with this try disables motors on loop exit.
                # Initialization earlier in main occurs before this protected block.
                try:
                    # Convert the integer code to a character and accept either letter case.
                    char = chr(key).lower()
                except:
                    char = ""

                # Quit
                if char == "q":
                    break

                # Move one motor
                if char in key_map:

                    # Unpack the selected joint and signed angle increment.
                    motor_id, change = key_map[char]

                    # Update the stored target. No robot-specific mechanical limit is checked.
                    commanded[motor_id] += change

                    motor.set_position(
                        motor_id,
                        position=commanded[motor_id],
                        kp=KP,
                        kd=KD
                    )

            # Continuously send position commands
            # Resend targets for every motor, even when no new key was pressed.
            # set_position sends a zero-velocity target with the selected KP and KD.
            for motor_id in MOTORS:

                motor.set_position(
                    motor_id,
                    position=commanded[motor_id],
                    kp=KP,
                    kd=KD
                )

                # Read the latest cached state; this does not wait for new feedback.
                data = motor.get_data(motor_id)

                if data is not None:
                    actual[motor_id] = data.current_position

            # Draw screen
            # Clear the drawing buffer and rebuild the interface using current values.
            stdscr.erase()

            # Write text at a screen row/column. Formatting such as +.3f includes a
            # plus/minus sign and three decimal places; it does not affect the command.
            stdscr.addstr(
                0,
                0,
                "=============================================="
            )

            # Write text at a screen row/column. Formatting such as +.3f includes a
            # plus/minus sign and three decimal places; it does not affect the command.
            stdscr.addstr(
                1,
                0,
                "       CyberGear 4-Motor Controller"
            )

            # Write text at a screen row/column. Formatting such as +.3f includes a
            # plus/minus sign and three decimal places; it does not affect the command.
            stdscr.addstr(
                2,
                0,
                "=============================================="
            )

            # Write text at a screen row/column. Formatting such as +.3f includes a
            # plus/minus sign and three decimal places; it does not affect the command.
            stdscr.addstr(
                4,
                0,
                "Controls:"
            )

            # Write text at a screen row/column. Formatting such as +.3f includes a
            # plus/minus sign and three decimal places; it does not affect the command.
            stdscr.addstr(
                5,
                0,
                "  W/S = Motor 1 (LK)"
            )

            # Write text at a screen row/column. Formatting such as +.3f includes a
            # plus/minus sign and three decimal places; it does not affect the command.
            stdscr.addstr(
                6,
                0,
                "  E/D = Motor 2 (RK)"
            )

            # Write text at a screen row/column. Formatting such as +.3f includes a
            # plus/minus sign and three decimal places; it does not affect the command.
            stdscr.addstr(
                7,
                0,
                "  R/F = Motor 3 (LH)"
            )

            # Write text at a screen row/column. Formatting such as +.3f includes a
            # plus/minus sign and three decimal places; it does not affect the command.
            stdscr.addstr(
                8,
                0,
                "  T/G = Motor 4 (RH)"
            )

            # Write text at a screen row/column. Formatting such as +.3f includes a
            # plus/minus sign and three decimal places; it does not affect the command.
            stdscr.addstr(
                9,
                0,
                "  Q   = Quit"
            )

            # Start joint displays at row 11 and leave three rows for each joint.
            row = 11

            for motor_id, name in MOTORS.items():

                draw_slider(
                    stdscr,
                    row,
                    f"{name}",
                    commanded[motor_id],
                    actual[motor_id]
                )

                row += 3

            # Write text at a screen row/column. Formatting such as +.3f includes a
            # plus/minus sign and three decimal places; it does not affect the command.
            stdscr.addstr(
                row + 1,
                0,
                f"Step: {STEP:.3f} rad    "
                f"Kp: {KP}    Kd: {KD}"
            )

            # Make the newly drawn screen visible.
            stdscr.refresh()

            # Pause 10 ms. Work above adds time, so the loop is not exactly 100 Hz.
            time.sleep(0.01)

    finally:

        # Send disable messages to all four motors; this does not move them to zero.
        motor.cleanup()


# ============================================================
# Run
# ============================================================

# Only launch the curses dashboard when executed as a script. The earlier
# module-level CyberGear construction still runs when imported.
if __name__ == "__main__":

    # wrapper initializes curses and restores terminal settings when main exits.
    curses.wrapper(main)