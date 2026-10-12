# LEARNING GUIDE: proportional-derivative feedback for four actuated joints.
# Input joint order is [left ankle, left knee, left hip, right hip, right knee].
# Positions use radians and velocities use rad/s; returned commands are torques.
# This module only computes numbers; billie_hardware.py sends them to the motors.
# NumPy provides numeric arrays and matrix-vector multiplication.
import numpy as np

# The class stores gain matrices so they can be reused at every control step.
class PDController:
    # For BILLIE, kp and kd are 4-by-4 matrices. Diagonal matrices give each
    # actuated joint its own independent proportional and derivative gain.
    def __init__(self, kp, kd, add_feedforward=True):
        # Convert the supplied gains to floating-point NumPy arrays.
        self.Kp = np.asarray(kp, dtype=float)
        self.Kd = np.asarray(kd, dtype=float)
        # Existing behavior: this option is stored but never consulted in compute().
        self.add_feedforward = add_feedforward

    # Calculate a four-element torque vector from reference and measured states.
    # uff is feedforward torque; a_ref is reference acceleration, unused here.
    def compute(self, p_ref, v_ref, p_measured, v_measured, uff, a_ref):
        # Slice [1:] removes passive ankle index 0 from all four position/velocity
        # vectors so each remaining entry corresponds to a motor.
        p_ref = p_ref[1:]
        v_ref = v_ref[1:]
        p_measured = p_measured[1:]
        v_measured = v_measured[1:]

        # Positive error means the reference angle is greater than the measured angle.
        # The velocity error is defined using the same reference-minus-measured sign.
        position_error = np.asarray(p_ref) - np.asarray(p_measured)
        velocity_error = np.asarray(v_ref) - np.asarray(v_measured)

        # print(f"position error: {position_error}")
        # print(f"velocity error: {velocity_error}")

        # The @ operator is matrix multiplication: ufb = Kp*position_error +
        # Kd*velocity_error. This is PD, with no accumulated integral-error term.
        ufb = self.Kp @ position_error + self.Kd @ velocity_error
        # This forms the feedforward-plus-feedback candidate, but it is not returned.
        u = uff + ufb

        # Actual behavior: return feedback only. The calculated u and the
        # add_feedforward flag have no effect on this returned value.
        return ufb
