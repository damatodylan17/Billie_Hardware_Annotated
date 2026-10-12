# LEARNING GUIDE: main saved-trajectory tracking experiment for BILLIE.
# Data flow: saved Xsol -> smooth reference -> controller -> CyberGear -> motors;
# reported motor states return through CyberGear's background CAN listener.
# Model order: [left ankle,left knee,left hip,right hip,right knee]. The ankle
# is passive, leaving four motor commands. Positions use rad, velocities rad/s,
# accelerations rad/s^2, and torques N*m. Read PDController next for the active law.
# This file performs hardware setup and prompts at module scope, even on import.
# Annotations explain existing behavior, including limitations, without fixing it.
# time provides the clock and sleep; NumPy handles numeric arrays; Pinocchio
# loads the robot model and computes rigid-body dynamics.
import time
import numpy as np
import pinocchio as pin

import matplotlib
# Select a noninteractive backend before importing pyplot so plots can be
# saved on a computer without an attached graphical display.
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# os builds output paths; datetime labels each experiment folder.
import os
from datetime import datetime

# This wrapper connects to the physical motors, rather than a simulator.
from cybergear import CyberGear
# Splines convert sampled trajectories into continuous functions of time.
from scipy.interpolate import CubicSpline, CubicHermiteSpline

# Import three alternative torque-control algorithms; only one is selected.
from Controllers.pd_controller import PDController
from Controllers.feedback_linearization_controller import FeedbackLinearization
from Controllers.linear_quadratic_regulizer_controller import TVLQR

################## 1. billie hardware #####################
# (1.a) initialize motors
# Map CAN addresses to joints: left knee=1, right knee=2, left hip=3, right hip=4.
LK_MOTOR_ID = 0x1
RK_MOTOR_ID = 0x2
LH_MOTOR_ID = 0x3
RH_MOTOR_ID = 0x4
# Construction opens/configures can0 and enables motors before the prompts.
# Subsequent model/data loading is outside main()'s try/finally block.
motors = CyberGear({LH_MOTOR_ID: "Cybergear", LK_MOTOR_ID: "Cybergear",
                RH_MOTOR_ID: "Cybergear", RK_MOTOR_ID: "Cybergear"})

print("CyberGear motors enabled.")

# Map motor ID to its position/velocity index. Index 0 is the passive ankle.
# The four torque indices are state index minus one.
STATE_IDX = {LK_MOTOR_ID: 1, LH_MOTOR_ID: 2, RH_MOTOR_ID: 3, RK_MOTOR_ID: 4}

# These are motor-local feedback gains, separate from the Python controller
# gains below. They remain active when an external torque term is also sent.
SAFETY_KP = 2.0  # prev 2.0
SAFETY_KD = 0.5
# Limit only the added torque term to +/-2 N*m. Motor-local position and
# velocity feedback can contribute additional torque beyond that term.
TORQUE_LIMIT = 2.0
# Duration parameter for _gain_ramp. Existing active commands do not apply
# that ramp; the multiplying expressions are commented out.
SOFT_START_S = 0.000001

################## 2. Controller #####################
# (2.a) intitialize pinocchio biped model and load + interpolate reference trajectories
# urdf_path = r"C:\Users\julie\OneDrive - Georgia Institute of Technology\courses\Capstone\Code\Biped_Simulation\Controllers\URDF\biped_billie_linkage_pinned-original.urdf"
# This absolute path refers to the author's machine. The repository model
# is under Controllers/URDF/, but no relative-path fallback is implemented.
urdf_path = r"/home/bliss/biped_ws/Controllers/URDF/biped_billie_linkage_pinned-original.urdf"
# Load the fixed-base model rooted at the left foot; biped stores topology
# and inertial properties. pin_d is a reusable dynamics calculation workspace.
biped = pin.buildModelFromUrdf(str(urdf_path))
pin_d = biped.createData()

# x_ref = np.load(r"C:\Users\julie\OneDrive - Georgia Institute of Technology\courses\Capstone\Code\Biped_Simulation\Data\Xsol_without-height.npy")
# u_ref = np.load(r"C:\Users\julie\OneDrive - Georgia Institute of Technology\courses\Capstone\Code\Biped_Simulation\Data\Usol_without-height.npy")

# n = number of steps
# This sets duration/trajectory selection, but does not directly control the
# repeat count: np.tile below independently hard-codes 20 repetitions.
n = 40
# Choose a single-step file for n=1; otherwise choose the full trajectory.
# These data paths are also absolute paths on the author's machine.
if n == 1:
    x_ref_path = r"/home/bliss/biped_ws/Xsol_0.2_periodic_0.2_wheight.npy"
    u_ref_path = r"/home/bliss/biped_ws/Usol_0.2_periodic_0.2_wheight.npy"
else:
    x_ref_path = r"/home/bliss/biped_ws/Xsol_full_0.2_periodic_0.2_wheight.npy"
    u_ref_path = r"/home/bliss/biped_ws/Usol_full_0.2_periodic_0.2_wheight.npy"

# Xsol rows are ten values: five positions followed by five velocities.
# The selected full trajectory has 40 rows before repetition.
x_ref = np.load(x_ref_path)
# Load four nominal torque values per row. These loaded values are later
# replaced by inverse-dynamics results, so they do not reach the controller.
u_ref = np.load(u_ref_path)

# Repeat the row sequence 20 times: 40 rows become 800. This is repetition
# of a stored trajectory, not an online gait planner or contact-switching law.
x_ref = np.tile(x_ref, (20, 1))

# Negate every state column, including velocities, for the chosen hardware
# sign convention. Negating u_ref below is superseded by recomputation.
x_ref = -x_ref
u_ref = -u_ref

# Total duration is 0.2*n seconds: 8 seconds with n=40.
step_duration = 0.2 * n
# Despite its name, dt is an array of sample times, not a scalar interval.
# It includes both endpoints, so spacing is duration/(number_of_rows-1).
dt = np.linspace(0.0, step_duration, x_ref.shape[0])
# Cubic Hermite interpolation fits positions using velocities as endpoint
# slopes. [:,:5] selects positions; [:,5:] selects velocities; axis=0
# means interpolation proceeds along the time/sample dimension.
x_reconstruct = CubicHermiteSpline(dt, x_ref[:, :5], x_ref[:, 5:], axis=0)
# The second derivative of the position spline supplies reference acceleration.
a_ref = x_reconstruct(dt, 2)

# Allocate one generalized torque per model velocity coordinate and time.
# This model has five, including the passive ankle coordinate.
tau = np.zeros((x_ref.shape[0], biped.nv))
for k in range(x_ref.shape[0]):
    # RNEA inverse dynamics computes tau=M(q)*qdd+h(q,qd) at each reference state.
    # This is a dynamics calculation, not a physical torque measurement.
    tau[k] = pin.rnea(biped, pin_d, x_ref[k, :5], x_ref[k, 5:], a_ref[k])

# Discard ankle torque and keep the four actuated components. This does
# not itself guarantee that the reference satisfies zero passive-ankle torque.
# It also overwrites the Usol data loaded earlier.
u_ref = tau[:, 1:]
# Interpolate the recalculated actuator torques for arbitrary query times.
u_reconstruct = CubicSpline(dt, u_ref, axis=0)

# (2.b) initialize pinocchio controller
# Select the active controller. PD is currently selected. The FBL and TVLQR
# paths have existing integration issues noted at the warm-up call below.
controller = "PD"

if controller == "PD":
    # Diagonal proportional gains: each actuated joint gets 5, with no direct
    # cross-joint terms. Kd below similarly assigns 0.1 to each joint.
    Kp = np.array([
            [5, 0,   0,  0],
            [0, 5, 0,  0],
            [0, 0,  5,  0],
            [0, 0,   0, 5]])
    Kd = np.array([
            [0.1, 0,  0,  0],
            [0,  0.1, 0,  0],
            [0,  0, 0.1,  0],
            [0,  0,  0, 0.1]])
    # The current PD implementation returns only Kp*position_error+Kd*velocity_error,
    # even though it calculates a feedforward-plus-feedback candidate internally.
    C = PDController(Kp, Kd)

elif controller == "FBL":
    # kp = np.diag([70,50,150,250])
    # kd = np.diag([10, 10, 10, 50])
    # Feedback-linearization gains act on an acceleration error law before
    # model-based conversion to torque; they are not direct PD torque gains.
    kp = np.diag([150,150,150,250])
    kd = np.diag([10, 10, 10, 50])
    C = FeedbackLinearization(biped, kp, kd)

elif controller == "TVLQR":
    # TVLQR weights five position errors at 900 and five velocity errors at 200.
    # R below weights four control deviations at 0.01 each.
    Q = np.diag([900, 900, 900, 900, 900,
                 200, 200, 200, 200, 200])
    R = np.eye(x_ref.shape[1]//2 - 1) * 0.01
    # Precompute feedback gains along the reference trajectory at construction.
    C = TVLQR(biped, x_ref, u_ref, Q, R, step_duration, len(x_ref))

################## 3. Helper functions #####################
# Read cached motor feedback and form five-element position/velocity arrays.
# No missing/stale-reading check or simultaneous-sampling guarantee is present.
def _get_states():
    # gets motor position and velocity
    lk_data = motors.get_data(LK_MOTOR_ID)
    rk_data = motors.get_data(RK_MOTOR_ID)
    lh_data = motors.get_data(LH_MOTOR_ID)
    rh_data = motors.get_data(RH_MOTOR_ID)

    # Estimate ankle position as the negative sum of the four motor angles
    # when building p_measured below; it is not measured by an ankle encoder.
    lankle = lk_data.current_position +  lh_data.current_position + rh_data.current_position + rk_data.current_position

    # Positions: [LK, LH, RH, RK]
    # Actual vector order includes the ankle first, despite the nearby original
    # comment listing only the four actuated joints.
    p_measured = np.array([-lankle, lk_data.current_position, lh_data.current_position,
                            rh_data.current_position, rk_data.current_position])

    # Velocities: [LK, LH, RH, RK]
    # Existing approximation sets ankle velocity to zero instead of the negative
    # sum of motor velocities, which would differentiate the position estimate.
    v_measured = np.array([0.0, lk_data.current_velocity, lh_data.current_velocity,
                            rh_data.current_velocity, rk_data.current_velocity])

    # These self-assignments have no effect; they are preserved as written.
    p_measured = p_measured
    v_measured = v_measured

    # print(f"p_measured: {p_measured}")
    # print(f"v_measured: {v_measured}")

    return p_measured, v_measured

# Return reference position, velocity, torque, and acceleration at time t.
# Spline derivative arguments 1 and 2 select first and second derivatives.
def interpolation(t):
    p_ref = x_reconstruct(t)
    v_ref = x_reconstruct(t, 1)
    a_ref = x_reconstruct(t, 2)
    u_ref = u_reconstruct(t)
    return p_ref, v_ref, u_ref, a_ref

# Compute a scalar from 0 to 1 by normalizing time and clipping endpoints.
# The active send path computes it but never uses it to scale commands.
def _gain_ramp(t):
    return float(np.clip(t / SOFT_START_S, 0.0, 1.0))

# Clip external torques and combine them with reference positions/velocities
# and motor-local gains. Return the clipped external torque vector.
def _send_command(t, u, p_ref, v_ref):
    ramp = _gain_ramp(t)
    # u = np.clip(u, -TORQUE_LIMIT, TORQUE_LIMIT) * ramp
    # Clamp each actuator's added torque, not the motor's total feedback torque.
    u = np.clip(u, -TORQUE_LIMIT, TORQUE_LIMIT)

    # Use state indices 1..4 for reference positions and 0..3 for torque values.
    # This calls the internal send method directly, bypassing the connection
    # guard decorating public set_position/set_velocity/set_torque methods.
    for motor_id, idx in STATE_IDX.items():
        motors._send_motion_command(motor_id, position=p_ref[idx], velocity=v_ref[idx], kp=SAFETY_KP, kd=SAFETY_KD, torque=u[idx -1],)
        # motors._send_motion_command(motor_id, position=p_ref[idx], velocity=v_ref[idx], kp=SAFETY_KP * ramp, kd=SAFETY_KD * ramp, torque=u[idx -1],)
    
    # motors._send_motion_command(LK_MOTOR_ID, position=p_ref[1], velocity=v_ref[1], kp=2, kd=0.5, torque=u[0])
    # motors._send_motion_command(LH_MOTOR_ID, position=p_ref[2], velocity=v_ref[2], kp=2, kd=0.5, torque=u[1])
    # motors._send_motion_command(RH_MOTOR_ID, position=p_ref[3], velocity=v_ref[3], kp=1, kd=1, torque=u[2])
    # motors._send_motion_command(RK_MOTOR_ID, position=p_ref[4], velocity=v_ref[4], kp=1, kd=1, torque=u[3])

    
    return u

# Store time histories and save PNGs. The names u_ref/u_real are historical: 
# the active logger call currently passes velocities into those slots.
class Logger:
    # Initialize lists and create a trajectory-named, timestamped plot directory.
    def __init__(self):
        self.time = []
        self.x_ref = []
        self.x_real = []
        self.u_ref = []
        self.u_real = []

        # Get trajectory name from x_ref filename
        # Take just the reference filename, then remove its .npy extension.
        trajectory_name = os.path.splitext(
            os.path.basename(x_ref_path)
        )[0]

        # Add timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        # Create folder
        folder_name = f"{trajectory_name}_{timestamp}"
        self.plot_dir = os.path.join("plots", folder_name)

        # Create the output folder, including missing parents, if it does not exist.
        os.makedirs(self.plot_dir, exist_ok=True)
        print(f"Saving plots to: {self.plot_dir}")


    # Copy each sample into the history so later array changes cannot mutate it.
    def log(self, t, x_ref, x_real, u_ref, u_real):
        self.time.append(t)
        self.x_ref.append(np.asarray(x_ref).copy())
        self.x_real.append(np.asarray(x_real).copy())
        self.u_ref.append(np.asarray(u_ref).copy())
        self.u_real.append(np.asarray(u_real).copy())

    # Reinitialize this object, clearing histories and choosing a new output path.
    def reset(self):
        self.__init__()

    # Convert histories into arrays and make separate plots for the five coordinates.
    # If there are no logged samples, the later two-dimensional indexing can fail.
    def plot(self):
        time = np.asarray(self.time)
        x_ref = np.asarray(self.x_ref)
        x_real = np.asarray(self.x_real)
        u_ref = np.asarray(self.u_ref)
        u_real = np.asarray(self.u_real)

        state_names = ["Left ankle", "Left knee", "Left hip", "Right hip", "Right knee"]

        # This four-joint label list is defined but unused by the active plotting loops.
        actuator_names = ["Left knee", "Left hip", "Right hip", "Right knee"]

        # For each modeled coordinate, select column i across all recorded time samples.
        # The ankle trace is estimated or assumed, not directly measured.
        for i, name in enumerate(state_names):
            plt.figure(figsize=(10, 4))
            plt.plot(time, x_ref[:, i], label="Reference")
            plt.plot(time, x_real[:, i], "--", label="Measured")
            plt.xlabel("Time [s]")
            plt.ylabel("Position [rad]")
            plt.title(name)
            plt.grid()
            plt.legend()

            filename = f"{name.lower().replace(' ', '_')}.png"
            filepath = os.path.join(self.plot_dir, filename)

            plt.savefig(filepath, dpi=150, bbox_inches="tight")
            plt.close()
            print(f"Saved: {filepath}")

        # for i, name in enumerate(actuator_names):
        # For each modeled coordinate, select column i across all recorded time samples.
        # The ankle trace is estimated or assumed, not directly measured.
        for i, name in enumerate(state_names):
            plt.figure(figsize=(10, 4))
            plt.plot(time, u_ref[:, i], label="reference velocity")
            plt.plot(time, u_real[:, i], "--", label="measured velocity")
            plt.xlabel("Time [s]")
            # Existing label issue: these are angular velocities, so their units are rad/s.
            plt.ylabel("Velocity [m/s]")
            plt.title(f"{name} velocity")
            plt.grid()
            plt.legend()

            # Existing filename issue: this second plot group contains velocity data,
            # despite the _torque.png suffix. The filename is preserved.
            filename = f"{name.lower().replace(' ', '_')}_torque.png"
            filepath = os.path.join(self.plot_dir, filename)

            plt.savefig(filepath,dpi=150,bbox_inches="tight")
            plt.close()
            print(f"Saved: {filepath}")

        # plt.show()


################## 4. Main control loop #####################

# input("hit enter to get the last position")
# # last_position = [-0.19531548,  0.05321584, -0.37441825,  0.11348898,  0.40302892]
# last_position = x_ref[-1]
# print(f"last_position: {last_position}")
# for i, motor_id in enumerate(STATE_IDX):
#     # motors._send_motion_command(motor_id, last_position[i+1], 0, 0, 0, 0)
#     motors.set_position(motor_id, last_position[i+1], kp=3,kd=0.2)

# (4.b) control loop
# Requested period is 0.002 seconds (500 Hz). Python scheduling, CAN traffic,
# and console output mean the achieved rate is not guaranteed.
CONTROL_DT = 0.002

# (4.a) set initial states
# The user approves sending the initial pose here; motors are already enabled.
input("Press Enter to configure robot to its initial pose.")
# Extract the first reference row; only four actuated position columns are used.
row_init = x_ref[0]
lknee_angle_init = row_init[1]
lhip_angle_init = row_init[2]
rhip_angle_init = row_init[3]
rknee_angle_init = row_init[4]

# Send initial-pose targets with local kp=3, kd=0.2 before trajectory tracking.
motors.set_position(LH_MOTOR_ID, lhip_angle_init,kp=3,kd=0.2)
motors.set_position(LK_MOTOR_ID, lknee_angle_init,kp=3,kd=0.2)
motors.set_position(RH_MOTOR_ID, rhip_angle_init,kp=3,kd=0.2)
motors.set_position(RK_MOTOR_ID, rknee_angle_init,kp=3,kd=0.2)

input("Hit Enter to start control")
# Read the latest states before evaluating the reference and feedback command.
p_measured, v_measured = _get_states()
p_ref, v_ref, uff, a_ref = interpolation(0)
# Warm-up calculation whose result is discarded. Existing issue: this call
# omits the time argument required by FBL and TVLQR, so switching to either
# currently fails here. FBL also has a measured/reference dimension mismatch.
_ = C.compute(p_ref, v_ref, p_measured, v_measured, uff, a_ref)

# Run the timed feedback loop, then command a final pose and save figures.
def main():
    logger = Logger()

    # Use a monotonic high-resolution elapsed-time clock for control scheduling.
    start_time = time.perf_counter()
    next_step = start_time

    try:
        while True:
            now = time.perf_counter()
            t = now - start_time
            # Stop when the final reference time is reached; no extra end-time command
            # is evaluated here, so p_ref retains the last sampled reference pose.
            if t >= dt[-1]:
                break

            # Read the latest states before evaluating the reference and feedback command.
            p_measured, v_measured = _get_states()
            p_ref, v_ref, uff, a_ref = interpolation(t)
            # print(f"p measured: {p_measured}")
            # print(f"p reference: {p_ref}")
            
            if controller == "PD":
                u = C.compute(p_ref, v_ref, p_measured, v_measured, uff, a_ref)

            elif controller == "TVLQR" or controller == "FBL":
                u = C.compute(p_ref, v_ref, p_measured, v_measured, uff, a_ref, t)

            u_sent = _send_command(t, u, p_ref, v_ref)
            # print(f"U sent: {u_sent}")
            # logger.log(t, p_ref, p_measured, uff, u_sent)
            # Log velocities in the u-named slots; torque logging above is commented out.
            logger.log(t, p_ref, p_measured, v_ref, v_measured)
            # Printing each iteration adds variable overhead to the requested 500 Hz loop.
            print(f"t: {t}, p_measured: {p_measured[1]}")
            
            # Advance an absolute schedule rather than sleeping a fixed period after work.
            next_step += CONTROL_DT
            
            sleep_time = next_step - time.perf_counter()
            # Sleep only if ahead of schedule. If behind, continue without sleeping;
            # there is no explicit deadline-miss handler or iteration-skipping logic.
            if sleep_time > 0:
                time.sleep(sleep_time)

    # On leaving this protected loop, request the last reference pose and plot.
    # The active code holds a pose instead of calling motors.cleanup().
    finally:
        time.sleep(0.01)
        # Read the final cached pose; it is not used by the active hold command.
        # Double underscore is just a variable name here, used to ignore velocity.
        last_position, __ = _get_states()
        print(f"p_ref final: {p_ref}")
        # Dictionary order matches [LK,LH,RH,RK], so i+1 selects the right reference.
        # The reference is the last evaluated pose, not necessarily the exact endpoint.
        for i, motor_id in enumerate(STATE_IDX):
            # motors._send_motion_command(motor_id, last_position[i+1], 0, 0, 0, 0)
            motors.set_position(motor_id, p_ref[i+1], kp=3,kd=0.2)
        # This message does not mean motors were disabled: the following cleanup
        # line is commented out and position-holding gains remain commanded.
        print("Motors stopped")
        # motors.cleanup()s
        logger.plot()
        


# Run main on direct execution. Earlier motor setup, file loads, and prompts
# are outside this guard and therefore also occur if this module is imported.
if __name__ == "__main__":
    main()
