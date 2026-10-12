# LEARNING GUIDE: high-level interface between movement scripts and CAN frames.
# Movement scripts call CyberGear methods. cybergear_driver.py packs/unpacks bytes.
# A background listener keeps MotorData objects updated with incoming feedback.
# Positions use rad, velocities rad/s, and torques N*m. Construction enables motors.
# Comments describe the existing implementation; no behavior has been repaired.
# Type hints describe expected arguments but do not enforce them at runtime.
# Callable describes a function; Optional allows a value or None.
from typing import Callable, Optional
# python-can supplies the bus, message listener, notifier, and CAN exceptions.
from can import CanOperationError
import can
# os/sys support console output and exit handling; dataclass generates
# record initialization; time provides delays; signal registers exit handlers.
import os
import sys
from dataclasses import dataclass
# Use the low-level encoder/decoder under the short alias cgd.
# motor_config supplies the model-specific range metadata.
import cybergear_driver as cgd
import motor_config
import time
import signal


# Generate initialization and representation methods from annotated fields.
# A MotorData object is a record, not a direct sensor or CAN connection.
@dataclass
class MotorData:
    """ Dataclass for storing motor state

    Contains commanded and most recent read state from the motor,
    as well as the current motor parameters and limits.    
    """
    # Required identity fields: bus address and the exact motor model string.
    motor_id: int
    motor_type: str
    # Reported fields start as None until motion feedback updates them.
    # Creating the object alone does not mean a motor has responded.
    current_position: float = None
    current_velocity: float = None
    current_torque: float = None
    current_temperature: float = None 
    # Commanded fields record requested values, distinct from physical readings.
    # They are updated before encoding, so they may differ from clamped values.
    commanded_position: float = 0
    commanded_velocity: float = 0
    commanded_torque: float = 0
    kp: float = 0
    kd: float = 0
    # These metadata tuples are replaced in __post_init__. They are not
    # actively checked by _send_motion_command; the encoder clamps separately.
    torque_limits: tuple = (0,0)
    velocity_limits: tuple = (0,0)
    position_limits: tuple = (0,0)
    kp_limits: tuple = (0,0)
    kd_limits: tuple = (0,0)
    # Several bookkeeping fields below are placeholders in this implementation.
    # For example, the listener does not update timestamp or initialized, so
    # these fields do not provide a working feedback-freshness check.
    timestamp: float = -1
    initialized: bool = False
    unique_hardware_id: int = None
    running_torque: list = None
    rms_torque: float = 0
    motor_mode: float = 0
    # Existing behavior: this unannotated mutable dictionary is a class attribute,
    # shared by MotorData instances rather than allocated separately per motor.
    internal_params = {}

    # The dataclass calls this after generated initialization to load ranges.
    def __post_init__(self):
        """Initializes the motor limits based on the motor type

        Raises:
            ValueError: Raised if the motor type is not specified
        """
        if self.motor_type is None:
            raise ValueError('motor_type must be specified')
        # Unpack ranges in the insertion order returned by motor_config: position,
        # velocity, torque, position gain, and velocity damping gain.
        self.position_limits, self.velocity_limits, self.torque_limits, self.kp_limits, self.kd_limits = motor_config.get_motor_limits(self.motor_type)

# Receive callbacks from python-can and route feedback to the correct record.
class CyberGearListener(can.Listener):
    # Create one MotorData record per expected ID. allow_strangers controls
    # whether previously unknown motor identities may be added.
    def __init__(self, motors: dict[int,str], allow_strangers: bool=False) -> None:
        # Initialize the inherited python-can Listener base class.
        super().__init__()
        self._allow_strangers = allow_strangers
        self._motors: dict[int, MotorData] = {}
        for motor_id, motor_type in motors.items():
            self._motors[motor_id] = MotorData( motor_id=motor_id, motor_type=motor_type)
        
    # The Notifier calls this for incoming frames; user scripts need not poll
    # the bus directly. Updates occur asynchronously relative to control code.
    def on_message_received(self, msg: can.Message):
        # Ignore standard-ID frames, error frames, and frames not marked received.
        if (not msg.is_extended_id) or msg.is_error_frame or (not msg.is_rx): return -1

        # Mask and shift the command/response field; & keeps selected bits and
        # >> moves them to the low end. The low byte is the target address.
        communication_type = (msg.arbitration_id & 0x3F000000) >> 24
        target_id = msg.arbitration_id & 0xFF

        # Accept messages addressed to the master, or the special identity-check byte.
        if (target_id != cgd.MASTER_CAN_ID) and (target_id != cgd.RESPONSE_IDENTITY_CHECK_FLAG): return -1
        
        if communication_type == cgd.RESPONSE_IDENTITY:
            unique_id, can_id = cgd.parse_identity_response(msg)
            # Existing issue in this optional path: MotorData requires motor_type,
            # but the construction below omits it. The default allow_strangers=False
            # keeps this path inactive in the supplied CyberGear constructor.
            if self._allow_strangers and (can_id not in self._motors.keys()):
                self._motors[can_id] = MotorData(motor_id=can_id, unique_hardware_id=unique_id)
            elif can_id in self._motors.keys():
                self._motors[can_id].unique_hardware_id=unique_id

        # Decode a returned parameter and store it under its numeric index.
        if communication_type == cgd.RESPONSE_PARAM:
            param_index, data_value, can_id = cgd.parse_param_response(msg)
            self._motors[can_id].internal_params[param_index] = data_value

        # Dedicated fault frames are ignored here; no fault-handling action is taken.
        if communication_type == cgd.RESPONSE_FAULT: return

        # Decode position, velocity, torque, temperature, mode, and fault flags.
        # The flags are unpacked below but not retained or acted on by this listener.
        if communication_type == cgd.RESPONSE_MOTION:
            pos, vel, trq, temp, can_id, motor_mode, cal_fault, hall_enc_fault, mag_enc_fault, overtemp_fault, overcurr_fault, undervolt_fault = cgd.parse_motion_response(msg)
            # Update the cached fields for this motor. These assignments do not form
            # an atomic snapshot of every motor at the same instant.
            self._motors[can_id].current_position = pos
            self._motors[can_id].current_velocity = vel
            self._motors[can_id].current_torque = trq
            self._motors[can_id].motor_mode = motor_mode
            self._motors[can_id].current_temperature = temp
            

# Public-facing wrapper owning the CAN bus and feedback listener.
class CyberGear():
    # motors maps IDs to model names; defaults select Linux SocketCAN can0.
    # Initialization configures the interface, creates a listener, installs
    # signal handlers, enables motors, and pauses briefly for responses.
    def __init__(self, motors: dict[int,str], channel:str ='can0', bustype: str='socketcan', invert:Optional[list[int]]=None, verbose: bool=False) -> None:
        # Run Linux setup commands. This helper always configures can0 even if a
        # different channel argument was supplied to this constructor.
        cgd.load_can_drivers()

        # Open the requested bus and exclude this socket's own transmitted frames.
        self._bus = can.Bus(channel=channel, bustype=bustype, receive_own_messages=False)
        self._cybergear_listener = CyberGearListener(motors)
        # Arrange background reception and callbacks to CyberGearListener.
        self._notifier = can.Notifier(self._bus, [self._cybergear_listener])
        self._enabled_for_motion = False
        # This flag starts true; it is not the result of verifying every motor.
        # Decorated send methods mark it false after CanOperationError.
        self._connection_established = True
        self.verbose = verbose
        self._priming_connection = False

        self.prev_command_time = time.perf_counter()
        
        if invert is None:
            invert = []

        # Prepare +/-1 signs for optional inversion. Existing behavior: this table
        # is not applied when encoding commands or reading feedback.
        self.invert_dict = {motor_id:-1 if motor_id in invert else 1 for motor_id in motors.keys()}

        # Install process signal handlers for interrupt and termination. They attempt
        # cleanup and then exit; this is separate from a script's own finally block.
        signal.signal(signal.SIGINT, self._exit_handler)
        signal.signal(signal.SIGTERM, self._exit_handler)

        # Enable the motors during construction, before caller prompts or motion loops.
        self.enable_for_motion()

        time.sleep(0.5)
        if verbose: print('CyberGear Motors Ready')


    # Decorator: wrap selected methods with connection-state and CAN-error checks.
    # It does not wrap every method, including direct _send_motion_command calls.
    def _guard_motor_connection(func: Callable):
        # *args and **kwargs forward positional and named arguments to the method.
        def wrapper(self, *args, **kwargs):
            if not self._connection_established:
                return -1 # Connection Lost
            try:
                res = func(self, *args, **kwargs)
            # Convert this CAN send exception into a stored connection-lost state and -1.
            # Other exception types still propagate; no reconnect is implemented here.
            except CanOperationError as e:
                # This flag starts true; it is not the result of verifying every motor.
                # Decorated send methods mark it false after CanOperationError.
                self._connection_established = False
                print(f'Connection Lost, Check Device Connections')
                return -1
            return res
        return wrapper
    
    # Attempt motor disable for SIGINT/SIGTERM, then raise SystemExit via sys.exit.
    def _exit_handler(self, sig, frame):
        if self.verbose: os.write(sys.stdout.fileno(), b'------ Shutting Down ------\n')
        if self._connection_established:
            try: self.cleanup()
            except:
                os.write(sys.stdout.fileno(), b'Error during cleanup\n')
        if self.verbose: os.write(sys.stdout.fileno(), b'------ Shutdown Complete ------\n')
        sys.exit(0)

    # Send zero added torque and zero feedback gains. Zero position/velocity
    # fields do not command motion because their gains are also zero.
    def _raw_set_zero_torque(self, motor_id: int) -> None:
        self._send_motion_command(motor_id, 0, 0, 0, 0, 0)

    # Redefine the current physical encoder angle as zero, then send zero torque.
    # This calibrates a reference; it does not move the shaft to an old zero.
    def set_zero_position(self, motor_id: int) -> None:
        self._bus.send(cgd.create_zero_position_message(target_motor_id=motor_id)) # Set the Zero Position
        self._raw_set_zero_torque(motor_id) # Set the Zero Torque

    # Request the addressed motor's identity; receiving it is handled asynchronously.
    def ping_actuator(self, act_id: int) -> None:
        self._bus.send(cgd.create_read_device_id_message(act_id))

    @_guard_motor_connection
    # Position mode through a combined motion frame: target velocity and added
    # torque are zero, while kp/kd create local position feedback and damping.
    def set_position(self, motor_id: int, position: float, kp: float, kd: float) -> None:
        return self._send_motion_command(motor_id, position, 0, 0, kp, kd)

    @_guard_motor_connection
    # Set kp=0 to remove position correction; kd controls velocity tracking.
    # The position field is therefore inactive in this command.
    def set_velocity(self, motor_id: int, velocity: float, kd: float) -> None:
        return self._send_motion_command(motor_id, 0, velocity, 0, 0, kd)

    @_guard_motor_connection
    # Set both feedback gains to zero and send only an added torque request.
    def set_torque(self, motor_id: int, torque: float) -> int:
        return self._send_motion_command(motor_id, 0, 0, torque, 0, 0)

    # Central send path for position, velocity, torque, kp, and kd.
    # A leading underscore marks an internal convention, not enforced privacy.
    def _send_motion_command(self, motor_id: int, position: float, velocity: float, torque: float, kp: float, kd: float) -> int:
        # Return 0 without sending if the ID was not registered in this interface.
        if motor_id not in self._cybergear_listener._motors.keys(): return 0

        # Record requested values. This happens before bus.send, so these fields
        # are not confirmation that transmission or motion succeeded.
        self._cybergear_listener._motors[motor_id].commanded_position = position
        self._cybergear_listener._motors[motor_id].commanded_velocity = velocity
        self._cybergear_listener._motors[motor_id].commanded_torque = torque
        self._cybergear_listener._motors[motor_id].kp = kp
        self._cybergear_listener._motors[motor_id].kd = kd

        # Convert physical values to a CAN frame, then transmit it. Encoding clamps
        # values to protocol ranges, not robot-specific mechanical joint limits.
        motion_message = cgd.create_motion_message(target_motor_id=motor_id, position=position, velocity=velocity, kp=kp, kd=kd, torque=torque)
        self._bus.send(motion_message)

        # Indicate that this send call completed, not that the target was reached.
        return 1

    # Return the existing mutable MotorData object, or None for an unknown ID.
    # There is no fresh read request, timeout, copy, or freshness validation here.
    def get_data(self, motor_id: int) -> MotorData:
        if motor_id in self._cybergear_listener._motors.keys():
            return self._cybergear_listener._motors[motor_id]
        return None

    # Convenience getters below extract one field from the cached record.
    # An unknown ID would make get_data return None and field access fail.
    def get_position(self, motor_id: int) -> float:
        return self.get_data(motor_id).current_position

    def get_velocity(self, motor_id: int) -> float:
        return self.get_data(motor_id).current_velocity
    
    def get_torque(self, motor_id: int) -> float:
        return self.get_data(motor_id).current_torque

    # Send disable frames to registered motors. This implementation does not
    # stop the Notifier or explicitly shut down the Bus object.
    def cleanup(self):
        for act_id in self._cybergear_listener._motors.keys():
            disable_message = cgd.create_disable_motion_message(act_id)
            self._bus.send(disable_message)

    # For each motor: request identity, enable motion, pause, send zero torque.
    # The routine does not wait for an identity match or verified enable reply.
    def enable_for_motion(self):
        for act_id in self._cybergear_listener._motors.keys():
            self.ping_actuator(act_id)
            enable_motion_message = cgd.create_enable_motion_message(target_motor_id=act_id)
            self._bus.send( enable_motion_message, timeout=0.1 )
            print(f'Enabling motion for Actuator {act_id}')
            time.sleep(0.1)
            self._raw_set_zero_torque(act_id)

        # This records completion of the send sequence, not confirmed hardware status.
        self._enabled_for_motion = True


# Standalone example: executing this module enables motor ID 5 and prints
# its cached record. Imports do not run this demonstration block.
if __name__ == "__main__":
    MOTOR_ID = 0x5
    motors = CyberGear({MOTOR_ID:"Cybergear"})
    print(motors.get_data(MOTOR_ID))
