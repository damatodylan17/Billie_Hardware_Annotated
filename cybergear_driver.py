# LEARNING GUIDE: CyberGear CAN protocol packing and unpacking.
# This module mostly constructs Message objects; cybergear.py sends them.
# The exception is load_can_drivers(), which executes Linux interface commands.
# Motion fields use radians, radians/second, N*m, and numeric feedback gains.
# Read from float_to_uint() through create_motion_message() to follow encoding.
# python-can provides Message containers; os executes the interface setup.
import can
import os

# Address assigned to this host; replies normally target this value.
MASTER_CAN_ID = 0

# Numeric protocol command types. These labels make bit-packed identifiers
# readable; defining a constant does not send a command.
CMD_GET_DEVICE_ID = 0
CMD_SET_MOTION = 1
CMD_ENABLE_MOTION = 3
CMD_DISABLE_MOTION = 4
CMD_SET_ZERO = 6
CMD_CHANGE_DEVICE_ID = 7
CMD_READ_PARAM = 17
CMD_WRITE_PARAM = 18


# Response types select the appropriate decoder. 0xFE is the special target
# byte checked for identity replies, rather than the normal master address.
RESPONSE_MOTION = 2
RESPONSE_IDENTITY = 0
RESPONSE_IDENTITY_CHECK_FLAG = 0xFE
RESPONSE_FAULT = 21
RESPONSE_PARAM = 17

# Motion response motor mode options
MODE_STATUS_RESET = 0
MODE_STATUS_CALIBRATE = 1
MODE_STATUS_RUN = 2

# These parameter mode values are distinct from the reported status modes
# above. Defining them does not switch a motor into any mode.
MOTOR_MODE_OPERATION = 0
MOTOR_MODE_POSITION = 1
MOTOR_MODE_VELOCITY = 2
MOTOR_MODE_CURRENT = 3

# Encoding limits: torque +/-12 N*m, position +/-12.5 rad, velocity +/-30
# rad/s, kp 0..500, kd 0..5. These are not robot mechanical travel limits.
T_MIN = -12
T_MAX = 12
P_MIN = -12.5
P_MAX = 12.5
V_MIN = -30
V_MAX = 30
KP_MIN = 0
KP_MAX = 500
KD_MIN = 0
KD_MAX = 5
# Each scaled motion field has 16 bits, spanning integer codes 0..65535.
N_BITS = 16

# --- Parameter Indices ---
# CUR - Current loop related value
# TRQ - Torque related value
# Parameter indices identify motor settings or telemetry registers.
# For example, IDX_RUN selects a mode; IDX_MECH_POS identifies position.
IDX_RUN = 0x7005
IDX_IQ_REF = 0x7006
IDX_SPD_REF = 0x700A
IDX_TRQ_LIM = 0x700B
IDX_CUR_KP = 0x7010
IDX_CUR_KI = 0x7011
IDX_CUR_FILT_GAIN = 0x7014
IDX_LOC_REF = 0x7016
IDX_SPD_LIM = 0x7017
IDX_CUR_LIM = 0x7018
IDX_MECH_POS = 0x7019
IDX_IQ_FILT_VAL = 0x701A # also references a 'f' in documentation
IDX_MECH_VEL = 0x701B
IDX_BUS_VOLT = 0x701C
IDX_NUM_ROTS = 0x701D
IDX_LOC_KP = 0x701E
IDX_SPD_KP = 0x701F
IDX_SPD_KI = 0x7020

# A set used to reject unknown parameter indices in read requests.
VALID_PARAMS = {
    IDX_RUN,
    IDX_IQ_REF,
    IDX_SPD_REF,
    IDX_TRQ_LIM,
    IDX_CUR_KP,
    IDX_CUR_KI,
    IDX_CUR_FILT_GAIN,
    IDX_LOC_REF,
    IDX_SPD_LIM,
    IDX_CUR_LIM,
    IDX_MECH_POS,
    IDX_IQ_FILT_VAL,
    IDX_MECH_VEL,
    IDX_BUS_VOLT,
    IDX_NUM_ROTS,
    IDX_LOC_KP,
    IDX_SPD_KP,
    IDX_SPD_KI
}

# Number of value bytes to extract from a parameter response. Size alone
# does not establish a parameter's numeric type or byte order.
PARAM_NUM_BYTES = {
    IDX_RUN:1,
    IDX_IQ_REF:4,
    IDX_SPD_REF:4,
    IDX_TRQ_LIM:4,
    IDX_CUR_KP:4,
    IDX_CUR_KI:4,
    IDX_CUR_FILT_GAIN:4,
    IDX_LOC_REF:4,
    IDX_SPD_LIM:4,
    IDX_CUR_LIM:4,
    IDX_MECH_POS:4,
    IDX_IQ_FILT_VAL:4,
    IDX_MECH_VEL:4,
    IDX_BUS_VOLT:4,
    IDX_NUM_ROTS:2,
    IDX_LOC_KP:4,
    IDX_SPD_KP:4,
    IDX_SPD_KI:4
}

# Metadata for read-only versus writable parameters. This table is not
# used to perform access checks by the current message helpers.
PARAM_READ_WRITE_STATUS = { # r indicates READ only, w indicates both READ and WRITE, there are no write only
    IDX_RUN:'w',
    IDX_IQ_REF:'w',
    IDX_SPD_REF:'w',
    IDX_TRQ_LIM:'w',
    IDX_CUR_KP:'w',
    IDX_CUR_KI:'w',
    IDX_CUR_FILT_GAIN:'w',
    IDX_LOC_REF:'w',
    IDX_SPD_LIM:'w',
    IDX_CUR_LIM:'w',
    IDX_MECH_POS:'r',
    IDX_IQ_FILT_VAL:'r',
    IDX_MECH_VEL:'r',
    IDX_BUS_VOLT:'r',
    IDX_NUM_ROTS:'w',
    IDX_LOC_KP:'w',
    IDX_SPD_KP:'w',
    IDX_SPD_KI:'w'
}

# BELOW ARE HELPER FUNCTIONS FOR CONSTRUCTING RELEVANT CYBERGEAR CAN MESSAGES.
# This stops short of sending the messages to allow for simulated enviornments and
# testing. Generally, the structure of a CAN message for these motors is as such.
# Arbitration ID contains three used 'fields'. The left most field (bit 28-24) contains
# the "communication type" indicating how the host or motor should respond to and unpack
# the command. This can always be extracted first to determine how to read the rest
# of the message. The middle field (bit 23-8) is a secondary data field. This can be
# used to carry additional information that the host or motor can act on. The 
# rightmost field is the 'target', i.e. which device the message is intended for.
# For messages intended for the host (coming from the motor), this is 0. Some messages use a 
# constant check value however. The data field is an 8 byte field with the majority of the
# message information. The functions below intend to preserve this notation and 
# distinciton to avoid confusion with documentation and minimize comments.

# Prepare Linux SocketCAN on can0 at 1 Mbit/s. These shell commands require
# appropriate privileges. os.system return codes are not checked here.
def load_can_drivers():
    """Loads and unload the can drivers, then reloads to ensure fresh driver initialization
    This will load the CAN drivers, but then remove them and load them again...
    Trust the process. Loading them alone will not reset the can drivers.
    If they are not reset, the buffer can fill up due to errors and the buffer
    will not properly reset.
    """
    # Ask Linux to load the CAN core module; can_raw adds raw-socket support.
    # The surrounding sequence takes can0 down, unloads, and reloads modules.
    os.system('sudo modprobe can')
    os.system('sudo modprobe can_raw')
    # os.system('sudo modprobe mttcan')

    os.system('sudo /sbin/ip link set can0 down')
    os.system('sudo rmmod can_raw')
    os.system('sudo rmmod can')
    # os.system('sudo rmmod mttcan')

    # Ask Linux to load the CAN core module; can_raw adds raw-socket support.
    # The surrounding sequence takes can0 down, unloads, and reloads modules.
    os.system('sudo modprobe can')
    os.system('sudo modprobe can_raw')
    # os.system('sudo modprobe mttcan')

    os.system('sudo /sbin/ip link set can0 down')
    # Bring can0 up with a transmit queue length of 1000 and bitrate 1,000,000.
    # This does not configure another channel passed to CyberGear.
    os.system('sudo /sbin/ip link set can0 txqueuelen 1000 up type can bitrate 1000000')

# Map a bounded physical value to an unsigned integer representation.
# (1 << n_bits)-1 gives the maximum code; int() truncates the scaled value.
def float_to_uint(x:float, x_min:float, x_max:float, n_bits:int) -> int:
    x = x_min if x < x_min else x_max if x > x_max else x # clamp x to [x_min, x_max]
    return int((x-x_min)*(((1<<n_bits)-1))/(x_max - x_min)) # scale x to [0, 2^n_bits-1]

# Inverse linear scaling: code/max_code*(x_max-x_min)+x_min.
# Quantization means a decoded number is only an approximation of its input.
def uint_to_float(x:int, x_min:float, x_max:float, n_bits:int) -> float:
    return (x / ((1<<n_bits)-1) * (x_max - x_min)) + x_min

# Pack fields with shifts and bitwise OR: command in high bits, data in
# the middle 16 bits, and the target address in the low 8 bits.
# Callers must supply valid field widths; this helper does not mask overflow.
def build_arbitration_id(target_id:int, cmd_id:int, data_field:int) -> int:
    return (cmd_id << 24) | (data_field << 8) | target_id

# Dispatcher: inspect response type/target and call the matching decoder.
# CyberGearListener performs similar routing itself instead of using this helper.
def parse_message(msg: can.Message):
    # The mask selects high response-type bits; shifting converts them to a number.
    communication_type = (msg.arbitration_id & 0x3F000000) >> 24
    # 0xFF keeps only the lowest eight bits, which hold the destination.
    target_id = msg.arbitration_id & 0xFF
    if target_id != MASTER_CAN_ID and target_id != RESPONSE_IDENTITY_CHECK_FLAG:
        return -1
    if communication_type == RESPONSE_IDENTITY:
        return parse_identity_response(msg) 
    elif communication_type == RESPONSE_PARAM:
        return parse_param_response(msg)
    # Existing behavior: dedicated fault responses return None without decoding.
    elif communication_type == RESPONSE_FAULT:
        return
    elif communication_type == RESPONSE_MOTION:
        return parse_motion_response(msg)

# Decode index and value bytes from a parameter reply, plus its motor ID.
# Existing behavior interprets 2/4-byte values as integers, with the default
# big-endian order in Python 3.11+. It does not decode IEEE floating-point
# parameter values or select byte order per parameter.
def parse_param_response(msg: can.Message) -> list:
    data = msg.data
    host_id = msg.arbitration_id & 0xFF
    cmd_id = (msg.arbitration_id & 0x3F000000) >> 24
    motor_id = (msg.arbitration_id & 0xFF00) >> 8
    # The index itself is assembled low-byte first from payload bytes 0 and 1.
    param_index = (data[1] << 8) | data[0]
    # Look up the expected value width; an unknown response index raises KeyError.
    num_param_bytes = PARAM_NUM_BYTES[param_index]
    if num_param_bytes == 1:
        return param_index, data[4], motor_id
    elif num_param_bytes == 2:
        return param_index, int.from_bytes(data[4:6]), motor_id
    elif num_param_bytes == 4:
        return param_index, int.from_bytes(data[4:]), motor_id



# Decode the four telemetry fields and status bits from a motion reply.
# The list order must agree with CyberGearListener's unpacking assignment.
def parse_motion_response(msg: can.Message) -> list:
    # Core information
    # Combine two bytes high-first, then divide by ten for the temperature value.
    temperature = (msg.data[7] | (msg.data[6] << 8))/10.0
    # Motion telemetry uses high-byte-first 16-bit fields: bytes 0/1 position,
    # 2/3 velocity, 4/5 torque. Convert each using its physical range.
    position = uint_to_float(msg.data[1] | (msg.data[0] << 8), P_MIN, P_MAX, N_BITS)
    velocity = uint_to_float(msg.data[3] | (msg.data[2] << 8), V_MIN, V_MAX, N_BITS)
    torque = uint_to_float(msg.data[5] | (msg.data[4] << 8), T_MIN, T_MAX, N_BITS)
    # Fault Information
    # Extract the sender address from identifier bits 8..15.
    motor_can_id = (msg.arbitration_id >> 8) & 0xFF
    # Extract a two-bit operating status and six one-bit fault indicators.
    # An expression (identifier >> bit) & 1 selects a single flag.
    motor_mode = (msg.arbitration_id >> 22) & 0x3
    calibration_fault = (msg.arbitration_id >> 21) & 0x1
    hall_encoder_fault = (msg.arbitration_id >> 20) & 0x1
    magnetic_encoder_fault = (msg.arbitration_id >> 19) & 0x1
    overtemp_fault = (msg.arbitration_id >> 18) & 0x1
    overcurrent_fault = (msg.arbitration_id >> 17 ) & 0x1
    undervoltage_fault = (msg.arbitration_id >> 16) & 0x1

    return [
        position,
        velocity, 
        torque, 
        temperature,
        motor_can_id,
        motor_mode, 
        calibration_fault, 
        hall_encoder_fault,
        magnetic_encoder_fault,
        overtemp_fault,
        overcurrent_fault,
        undervoltage_fault 
    ]

# Decode the unique hardware identity and CAN ID from an identity response.
def parse_identity_response(msg: can.Message) -> list:
    check_flag = msg.arbitration_id & 0xFF  
    cmd_id = (msg.arbitration_id & 0x3F000000) >> 24
    # This is the existing identity-response mask, which spans more than one
    # byte. Its behavior is preserved rather than replaced by the motion-ID mask.
    motor_id = (msg.arbitration_id & 0x7FFF80) >> 8
    # Interpret all payload bytes as one big-endian integer hardware identifier.
    unique_identity = int.from_bytes(msg.data, byteorder='big')
    print(f'{check_flag:x}')
    print(cmd_id)
    if cmd_id == RESPONSE_IDENTITY and check_flag == RESPONSE_IDENTITY_CHECK_FLAG:
        return unique_identity, motor_id
    # Use a pair of False values when the response type/check byte does not match.
    return False, False

# Construct an enable frame (type 3); all eight payload bytes are zero.
# Every create_* function returns a Message and leaves transmission to the caller.
def create_enable_motion_message(target_motor_id: int):
    enable_motion = can.Message(
        arbitration_id=build_arbitration_id(target_id=target_motor_id, cmd_id=CMD_ENABLE_MOTION, data_field=MASTER_CAN_ID),
        data=[0]*8,
        is_extended_id=True
    )
    return enable_motion

# Construct a disable frame (type 4); optionally set the first payload byte
# to request fault clearing. The default does not request fault clearing.
def create_disable_motion_message(target_motor_id: int, clear_fault:bool=False):
    data = [1,0,0,0,0,0,0,0] if clear_fault else [0]*8
    disable_motion = can.Message(
        arbitration_id=build_arbitration_id(target_id=target_motor_id, cmd_id=CMD_DISABLE_MOTION, data_field=MASTER_CAN_ID),
        data=data,
        is_extended_id=True
    )
    return disable_motion

# Construct an identity request addressed to one motor.
def create_read_device_id_message(target_motor_id: int):
    read_device_id = can.Message(
        arbitration_id=build_arbitration_id(target_id=target_motor_id, cmd_id=CMD_GET_DEVICE_ID, data_field=MASTER_CAN_ID),
        data=[0]*8,
        is_extended_id=True
    )
    return read_device_id

# Construct an address-change request with the new ID in the middle field.
# This helper is defined but not called by the supplied movement scripts.
def create_set_can_id_message(target_motor_id: int, new_motor_id: int):
    set_device_id = can.Message(
        arbitration_id=build_arbitration_id(target_id=target_motor_id, cmd_id=CMD_CHANGE_DEVICE_ID, data_field=(new_motor_id<<8 | MASTER_CAN_ID)),
        data=[0]*8,
        is_extended_id=True
    )
    return set_device_id

# Validate an index and request its value; the index is packed low-byte first
# into bytes 0 and 1. Remaining payload bytes are zero.
def create_read_param_message(target_motor_id: int, param_index: int):
    if param_index not in VALID_PARAMS:
        raise ValueError(f'{param_index} is not a valid parameter index. Please use the defined constant indices')
    read_param = can.Message(
        arbitration_id=build_arbitration_id(target_id=target_motor_id, cmd_id=CMD_READ_PARAM, data_field=MASTER_CAN_ID),
        data=[param_index & 0xFF, (param_index >> 8) & 0xFF, 0, 0, 0, 0, 0, 0],
        is_extended_id=True
    )
    return read_param

# Build the command that makes the motor's present mechanical angle zero.
# Payload byte 0 is 1; this is a reference change, not an angle target.
def create_zero_position_message(target_motor_id:int):
    zero_position = can.Message(
        arbitration_id=build_arbitration_id(target_id=target_motor_id, cmd_id=CMD_SET_ZERO, data_field=MASTER_CAN_ID),
        data=[1,0,0,0,0,0,0,0],
        is_extended_id=True
    )
    return zero_position

# Pack position, velocity, kp, kd, and torque into one extended CAN frame.
def create_motion_message(target_motor_id:int, position:float, velocity:float, kp:float, kd:float, torque:float):
    # Clamp and scale every physical field into a 16-bit integer code.
    # This can alter a requested value that exceeds the encoder's range.
    pos_field = float_to_uint(position, P_MIN, P_MAX, N_BITS)
    vel_field = float_to_uint(velocity, V_MIN, V_MAX, N_BITS)
    kp_field = float_to_uint(kp, KP_MIN, KP_MAX, N_BITS)
    kd_field = float_to_uint(kd, KD_MIN, KD_MAX, N_BITS)
    torque_field = float_to_uint(torque, T_MIN, T_MAX, N_BITS)

    # For a motion command, torque occupies the identifier's middle field.
    # The eight payload bytes hold position, velocity, kp, and kd instead.
    motion = can.Message(
        arbitration_id=build_arbitration_id(target_id=target_motor_id, cmd_id=CMD_SET_MOTION, data_field=torque_field),
        data=[
            # Split each integer into a high byte (>>8) and low byte (&0xFF).
            # Send high byte first for each of the four payload fields.
            (pos_field >> 8),
            (pos_field & 0xFF),
            (vel_field >> 8),
            (vel_field & 0xFF),
            (kp_field >> 8),
            (kp_field & 0xFF),
            (kd_field >> 8),
            (kd_field & 0xFF)
        ],
        is_extended_id=True
    )
    return motion