import wpilib
import phoenix6
import navx
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, PIDConstants, ReplanningConfig
from phoenix6.configs.cancoder_configs import *
from phoenix6.configs.talon_fx_configs import *
from phoenix6.configs.config_groups import MagnetSensorConfigs
from phoenix6.controls import *
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.controls.motion_magic_voltage import MotionMagicVoltage
from phoenix6.signals import *
from typing import Self
from wpilib import DriverStation, Field2d, RobotBase, SmartDashboard
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModulePosition, SwerveModuleState

from constants import *

class SwerveModule(wpilib.Subsystem):

    def __init__(self, module_name, drive_motor_constants: DriveMotorConstants, dir_motor_constants: DirectionMotorConstants, CAN_id: int, CAN_offset: float) -> None:

        super().__init__()
        self.turn_encoder = CANcoder(CAN_id, "rio")

        encoder_config =  CANcoderConfiguration()
        encoder_config.magnet_sensor = MagnetSensorConfigs().with_sensor_direction(SensorDirectionValue.CLOCKWISE_POSITIVE).with_magnet_offset(CAN_offset).with_absolute_sensor_range(AbsoluteSensorRangeValue.UNSIGNED_0_TO1)
        self.turn_encoder.configurator.apply(encoder_config)

        self.drive_motor = TalonFX(drive_motor_constants.motor_id, "rio")
        drive_motor_constants.apply_configuration(self.drive_motor)

        self.dir_motor = TalonFX(dir_motor_constants.motor_id, "rio")
        dir_motor_constants.apply_configuration(self.dir_motor)

        self.dirTargetPos = self.dirTargetAngle = 0.0

        self.sim = 0

    def getAngle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(rots_to_degs(self.dir_motor.get_rotor_position().value / k_direction_gear_ratio))
    
    def resetSensorPos(self) -> None:
        self.dir_motor.set_position(-self.turn_encoder.get_absolute_position().wait_for_update(0.02).value * k_direction_gear_ratio)

    def getModuleState(self) -> SwerveModuleState:

