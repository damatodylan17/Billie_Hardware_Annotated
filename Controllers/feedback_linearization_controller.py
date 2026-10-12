# LEARNING GUIDE: model-based acceleration tracking with a passive ankle.
# Dynamics are M(q)*q_ddot + h(q,q_dot) = [0, u], with four actuated joints.
# The controller partitions these equations to eliminate passive acceleration.
# Existing integration and dimension issues are described below, not corrected.
# NumPy handles matrices; Pinocchio computes rigid-body dynamics from the URDF.
import numpy as np
import pinocchio as pin

# TODO: use mass matrix and everything from pinnochio


# Store a robot model and feedback gains; this class does not send CAN frames.
class FeedbackLinearization:
    # model supplies geometry/inertias; kp and kd are four-joint gain matrices.
    def __init__(self, model, kp, kd):
        self.model = model
        # Allocate a Pinocchio workspace for intermediate dynamics results.
        self.data = model.createData()
        self.Kp = np.asarray(kp, dtype=float)
        self.Kd = np.asarray(kd, dtype=float)

        # Index 0 represents the passive ankle, although the slices below directly
        # use 0 and 1 rather than reading this stored attribute.
        self.passive_idx = 0

    # Inputs are reference/measured position and velocity, feedforward torque,
    # reference acceleration, and time. The time parameter is not used here.
    # The main script's warm-up call currently omits this required argument.
    def compute(self, p_ref, v_ref, p_measured, v_measured, uff, a_ref, time):
        # mass matrix from pinocchio model
        # Allocate a matrix, then immediately replace it with the CRBA result below.
        M = np.zeros((self.model.nq, self.model.nq))
        # CRBA calculates the joint-space inertia matrix at the measured pose.
        M = pin.crba(self.model, self.data, p_measured)
        # Average the matrix with its transpose to enforce symmetry.
        M = (M + M.T) / 2.0

        # nonlinear term from pinocchio model
        # nonLinearEffects supplies h: gravity plus velocity-dependent terms.
        NL = pin.nonLinearEffects(self.model, self.data, p_measured, v_measured)

        # Partition into passive (p) and actuated (a) coordinates:
        # M11 is 1x1, M12 is 1x4, M21 is 4x1, M22 is 4x4 for this model.
        # M11*qdd_p + M12*qdd_a + NL1 = 0 because no ankle torque is applied.
        M11 = M[0:1, 0:1]
        M12 = M[0:1, 1:]
        M21 = M[1:, 0:1]
        M22 = M[1:, 1:]

        # Split the bias vector into the passive entry and four actuated entries.
        NL1 = NL[0:1]
        NL2 = NL[1:]

        # Existing mismatch: p_ref[1:] has four elements, but billie_hardware.py
        # passes five-element p_measured and v_measured arrays. These subtractions
        # therefore fail with that caller; the original expressions are preserved.
        position_error = p_ref[1:] - p_measured
        velocity_error = v_ref[1:] - v_measured
        # Intended acceleration target: reference acceleration plus PD correction.
        # Here Kp and Kd act in an acceleration law, unlike direct-torque PD gains.
        desired_acceleration = (a_ref[1:] + self.Kp @ position_error + self.Kd @ velocity_error)

        # solve(A,B) computes the solution of A*X=B without explicitly forming A^-1.
        # Use the passive equation to express its acceleration in terms of the others.
        M11_inv_M12 = np.linalg.solve(M11, M12)
        M11_inv_NL1 = np.linalg.solve(M11, NL1)
        # The Schur complement M22 - M21*M11^-1*M12 is the effective actuated inertia.
        # The reduced bias removes the corresponding passive-equation contribution.
        reduced_mass = M22 - M21 @ M11_inv_M12
        reduced_bias = NL2 - M21 @ M11_inv_NL1
        # Multiply the desired actuated acceleration by effective inertia, then add
        # the reduced bias to obtain a model-based torque command.
        ufb = reduced_mass @ desired_acceleration + reduced_bias

        # Existing behavior adds uff on top of a torque that already contains model
        # acceleration and bias terms; this is not merely a PD correction. Whether
        # that extra feedforward is appropriate depends on its intended definition.
        return ufb + uff