# LEARNING GUIDE: finite-horizon time-varying linear quadratic regulation.
# TVLQR linearizes dynamics along a trajectory and computes feedback gains backward
# in time. It returns u_ff + K(t)*(x_ref - x_measured), using ten state variables
# and four actuator inputs for BILLIE. The filename spelling is unchanged.
# NumPy handles matrices, Pinocchio supplies dynamics derivatives, and
# SciPy interp1d selects a stored gain for a requested time.
import numpy as np
import pinocchio as pin
from scipy.interpolate import interp1d


# Gain computation happens during construction; later calls reuse the gains.
class TVLQR:
    # Xref rows contain [q, q_dot]; Uref rows contain four torques.
    # Q penalizes state deviations, R penalizes control deviations, and N is
    # the number of reference samples over step_duration seconds.
    def __init__(self, biped, Xref, Uref, Q, R, step_duration, N):
        self.biped = biped
        # Each controller owns a Pinocchio workspace for derivative calculations.
        self.data = biped.createData()
        self.Xref = Xref
        self.Uref = Uref
        self.Q = Q
        self.R = R
        self.step_duration = step_duration
        self.N = N
        # m is the number of actuators (4); nq is the number of joint positions (5).
        # Using nq for velocity dimensions is valid for this all-revolute model;
        # it is not a general assumption for floating-base quaternion models.
        self.m = Uref.shape[1]
        self.nq = biped.nq

        # Build the reference time grid and the interval lengths between samples.
        self.t_grid = np.linspace(0.0, step_duration, self.N)
        self.dt_grid = np.diff(self.t_grid)

        # S maps four motor torques to five generalized torques. Row 0 remains zero
        # for the passive ankle; the following rows form a 4x4 identity matrix.
        self.S = np.zeros((self.nq, self.m))
        self.S[1:1 + self.m, :] = np.eye(self.m)

        # Solve the trajectory-dependent feedback problem once at initialization.
        self.K = self._compute_K()
        # Flatten each 4x10 matrix into a row for interpolation. kind="previous"
        # holds each gain constant until the next sample (zero-order hold).
        K_zh = self.K.reshape(self.N, -1)
        self.K_reconstructed = interp1d(self.t_grid, K_zh, axis=0, kind="previous", bounds_error=False, fill_value=(K_zh[0], K_zh[-1]))

    # Backward discrete-time Riccati recursion: propagate the future cost
    # matrix P and choose the gain minimizing the local quadratic cost.
    def _compute_K(self):
        # Reserve one 4x10 gain per reference time. The last gain is left at zero
        # because the loop below starts at N-2; this is the existing terminal behavior.
        K = np.zeros((self.N, self.m, self.nq*2))
        # Use Q as the terminal state cost. copy() starts with independent storage.
        P = self.Q.copy()

        # Walk backward from the penultimate sample to index zero, inclusive.
        for k in range(self.N-2, -1, -1):
            # Split one reference row into joint angles and joint velocities.
            p_k, v_k = self.Xref[k, :self.nq], self.Xref[k, self.nq:]
            u_k = self.Uref[k, :]
            dt_k = self.dt_grid[k] if k < self.N - 1 else self.dt_grid[-1]

            # Evaluate the continuous-time state and input Jacobians at this sample.
            Ak, Bk = self._linearize(p_k, v_k, u_k)
            # Forward-Euler approximation: Ad = I + A*dt and Bd = B*dt.
            # This is a first-order discretization, not an exact matrix exponential.
            A_d = np.eye(self.nq*2) + Ak * dt_k
            B_d = Bk * dt_k

            # Compute K = (R + Bd.T*P*Bd)^-1 * Bd.T*P*Ad using a linear solve.
            # .T denotes transpose; @ denotes matrix multiplication.
            S_gain = self.R + B_d.T @ P @ B_d
            K_k = np.linalg.solve(S_gain, B_d.T @ P @ A_d)
            # Update the cost-to-go matrix for the preceding time step.
            P = self.Q + A_d.T @ P @ A_d - A_d.T @ P @ B_d @ K_k
            K[k] = K_k
            
        return K

    # For state x=[q,v], its derivative is [v, acceleration(q,v,u)].
    def _linearize(self, p, v, u):
        # gives A and B in evaluation point
        # Map actuator inputs to generalized torques before differentiating dynamics.
        tau = self.S @ u
        # Differentiate the articulated-body forward dynamics with respect to
        # position, velocity, and torque at this reference state.
        pin.computeABADerivatives(self.biped, self.data, p, v, tau)
        ddp_dp = self.data.ddq_dq
        ddp_dv = self.data.ddq_dv
        # Acceleration sensitivity to actuator torque is M^-1*S.
        ddp_dtau = self.data.Minv @ self.S

        # Build A = [[0,I],[d(acceleration)/dq,d(acceleration)/dv]].
        # The top identity expresses q_dot = v.
        A = np.zeros((self.nq*2, self.nq*2))
        A[:self.nq, self.nq:] = np.eye(self.nq)
        A[self.nq:, :self.nq] = ddp_dp
        A[self.nq:, self.nq:] = ddp_dv

        # Build B = [[0],[M^-1*S]]: input torque directly changes acceleration.
        B = np.zeros((self.nq*2, self.m))
        B[self.nq:, :] = ddp_dtau
        return A, B

    # Clamp the lookup time to the reference interval, then restore the
    # flattened gain to its 4x10 matrix shape.
    def _gain_at_t(self, t):
        t = np.clip(t, self.t_grid[0], self.t_grid[-1])
        return self.K_reconstructed(t).reshape(self.m, 2 * self.nq)

    # Use the gain at time t to calculate torque. a_ref is accepted but unused.
    # The main script's warm-up call currently omits the required t argument.
    def compute(self, p_ref, v_ref, p_measured, v_measured, uff, a_ref, t):
        # hstack joins positions and velocities into the ten-element error vector.
        # Since error is reference minus measured, feedback is added with a plus sign.
        e_x = np.hstack([p_ref, v_ref]) - np.hstack([p_measured, v_measured])
        K_t = self._gain_at_t(t)
        # Multiply the state error by the time-dependent gain to obtain four torques.
        ufb = K_t @ e_x
        # Add nominal feedforward torque to the corrective feedback torque.
        u = uff + ufb
        return u