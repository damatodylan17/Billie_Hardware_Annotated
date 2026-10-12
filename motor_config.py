# LEARNING GUIDE: lookup table of motor ranges used by MotorData.
# These entries are metadata, not measured values or robot joint constraints.
# The active CyberGear message encoder has its own constants in cybergear_driver.py.
"""This list of parameters was last updated on: 22 January 2024
Sites from which information was sourced: 
- TMotor AK page: https://store.tmotor.com/categorys/robot-dynamics
- CubeMars AK page: https://www.cubemars.com/category-122-AK+Series+Robotic+Actuation+Module.html

Units of below limits: 
- Position [rad]
- Velocity [rad/s]
- Torque [Nm]
"""

# A nested dictionary: each motor model name maps to five (minimum, maximum)
# Tuples. Units are rad, rad/s, and N*m for position, velocity, and torque.
# kp and kd describe position feedback stiffness and velocity damping.
MOTOR_PARAMS = {
    'AK10-9-V2.0': { # 24V/48V operation 
        'position_limits': (-12.5, 12.5),
        'velocity_limits': (-50.0, 50.0),
        'torque_limits': (-48.0, 48.0),
        'kp_limits': (0.0, 500.0),
        'kd_limits': (0.0, 5.0),
    },
    'AK60-6-V1.1': { # 24V operation
        'position_limits': (-12.5, 12.5),
        'velocity_limits': (-50.0, 50.0),
        'torque_limits': (-9.0, 9.0),
        'kp_limits': (0.0, 500.0),
        'kd_limits': (0.0, 5.0),
    },
    'AK70-10': { # 24V/48V operation
        'position_limits': (-12.5, 12.5),
        'velocity_limits': (-50.0, 50.0),
        'torque_limits': (-24.8, 24.8),
        'kp_limits': (0.0, 500.0),
        'kd_limits': (0.0, 5.0),
    },
    'AK80-6': { # 48V operation
        'position_limits': (-12.5, 12.5),
        'velocity_limits': (-50.0, 50.0),
        'torque_limits': (-12.0, 12.0),
        'kp_limits': (0.0, 500.0),
        'kd_limits': (0.0, 5.0),
    },
    'AK80-8': { # 48V operation
        'position_limits': (-12.5, 12.5),
        'velocity_limits': (-50.0, 50.0),
        'torque_limits': (-25.0, 25.0),
        'kp_limits': (0.0, 500.0),
        'kd_limits': (0.0, 5.0),
    },
    'AK80-9': { # 48V operation
        'position_limits': (-12.5, 12.5),
        'velocity_limits': (-50.0, 50.0),
        'torque_limits': (-18.0, 18.0),
        'kp_limits': (0.0, 500.0),
        'kd_limits': (0.0, 5.0),
    },
    'AK80-64': { # 24V/48V operation 
        'position_limits': (-12.5, 12.5),
        'velocity_limits': (-50.0, 50.0),
        'torque_limits': (-120.0, 120.0),
        'kp_limits': (0.0, 500.0),
        'kd_limits': (0.0, 5.0),
    },
    # This is the model string used by BILLIE's scripts; spelling must match.
    # These broad encoding ranges do not establish safe mechanical knee/hip travel.
    'Cybergear': {
        'position_limits': (-12.5, 12.5),
        'velocity_limits': (-30.0, 30.0),
        'torque_limits': (-12.0, 12.0),
        'kp_limits': (0, 500.0),
        'kd_limits': (0, 5.0)
    }
}

# Return the five ranges associated with one exact model name.
# MotorData.__post_init__ unpacks them into its limit fields.
def get_motor_limits(motor_type):
    # Reject an unknown model name and show the available dictionary keys.
    if motor_type not in MOTOR_PARAMS.keys():
        raise ValueError(f'{motor_type} is not a valid motor type, must be one of {list(MOTOR_PARAMS.keys())}')
    # dict.values() follows insertion order: position, velocity, torque, kp, kd.
    # The caller depends on this order. This function does not clamp any command.
    return MOTOR_PARAMS[motor_type].values()
