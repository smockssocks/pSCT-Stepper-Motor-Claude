"""
psct_motors -- control of the pSCT focal-plane actuators.

Three JVL MIS23x integrated stepper motors, arranged in a triangle, each
pushing one ball-pin joint of the camera's inner structure along the optical
axis. Together they set the focal plane's focus, tip and tilt.

Layer map, bottom up::

    transport.py   Modbus TCP wire, pymodbus version differences absorbed
    registers.py   JVL register numbers, word order, bit-field decoding
    jvl_motor.py   one motor: position, mode, brake, errors, motion complete
    kinematics.py  three actuator heights <-> (focus, tip, tilt)
    platform.py    all three driven together, with limits and interlocks
    simulator.py   a fake motor at the transport boundary, for hardware-free runs

and on top of those::

    gui.py         desktop application
    cli.py         commissioning, calibration and scripted moves
    server.py      JSON-over-TCP bridge (this is what LabVIEW should talk to)
    labview_api.py flat function API for LabVIEW's native Python node

Start with the README; `python -m psct_motors.cli --help` is the quickest way
in once the config is filled out.
"""

from .config import (
    ActuatorConfig, BrakeConfig, PlatformConfig, PlatformLimits,
    default_config, default_config_path, load_config, save_config,
)
from .jvl_motor import BrakeState, BrakeStatus, JVLMotor, MotorFault, MotorStatus
from .kinematics import Orientation, ThreePointPlatform, platform_from_config
from .platform import FocalPlanePlatform, PlatformError, PlatformState
from .registers import MotorMode, WordOrder
from .transport import ModbusError

__version__ = "1.0.0"

__all__ = [
    "ActuatorConfig", "BrakeConfig", "PlatformConfig", "PlatformLimits",
    "default_config", "default_config_path", "load_config", "save_config",
    "BrakeState", "BrakeStatus", "JVLMotor", "MotorFault", "MotorStatus",
    "Orientation", "ThreePointPlatform", "platform_from_config",
    "FocalPlanePlatform", "PlatformError", "PlatformState",
    "MotorMode", "WordOrder", "ModbusError",
    "__version__",
]
