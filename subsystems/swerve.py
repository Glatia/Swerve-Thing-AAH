import wpilib
import phoenix6
import navx
import math
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
        return SwerveModuleState(rots_to_meters(self.drive_motor.get_velocity().value), self.getAngle())
    
    def getModulePosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(rots_to_meters(self.drive_motor.get_position().value), self.getAngle())
    
    def setModuleState(self, desiredState: SwerveModuleState, override_brake_dur_neutral: bool=True) -> None:
    #   I'm not sure what the point of optimize is since we do the math farther down
    #   desiredState.optimize(desiredState, self.getAngle())
        desiredAngle = desiredState.angle.degrees() % 360

        angleDistance = math.fabs(desiredAngle - self.dirTargetAngle)

        if (angleDistance > 90 and angleDistance < 270):
            targetAngle = (desiredAngle + 180) % 360
            self.invert = -1
        else:
            targetAngle = desiredAngle
            self.invert = 1
        
        targetAngleDist = math.fabs(targetAngle - self.dirTargetAngle)

        if targetAngleDist > 180:
            targetAngleDist = abs(targetAngleDist - 360)

        rotChange = degs_to_rots(targetAngleDist)

        angleDifference = targetAngle - self.dirTargetAngle
        

