import commands2
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

# Swerve Module class
class SwerveModule(commands2.Subsystem):

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

        self.currentRot = 0.0

    # Returns the current angle based off of the rotor position of the direction motor and the gear ratio between it and the wheel
    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromRotations((self.dir_motor.get_rotor_position().value / k_direction_gear_ratio))
    
    # Resets the turn motor to 0 rots by subtracting its current position from itself
    def reset_sensor_pos(self) -> None:
        self.dir_motor.set_position(-self.turn_encoder.get_absolute_position().wait_for_update(0.02).value * k_direction_gear_ratio)

    # Returns the current state
    def get_module_state(self) -> SwerveModuleState:
        return SwerveModuleState(rots_to_meters(self.drive_motor.get_velocity().value), self.get_angle())
    
    # Returns the current position
    def get_pos(self) -> SwerveModulePosition:
        return SwerveModulePosition(rots_to_meters(self.drive_motor.get_position().value), self.get_angle())
    
    # Sets the module state
    def set_module_state(self, desiredState: SwerveModuleState, override_brake_dur_neutral: bool=True) -> None:
        
        desiredState.optimize(desiredState, self.get_angle())
        desiredAngle = desiredState.angle.rotations()

        angleDistance = math.fabs(desiredAngle - self.currentRot)

        # =====================Angle Optimization=====================

        # If the angle distance is greater than 90 and less than 270, change the target angle to across from the desired angle and invert the wheel        
        if (angleDistance > 0.25 and angleDistance < 0.75):
            targetAngle = (desiredAngle + 0.5)
            self.invert = -1
        else:
            targetAngle = desiredAngle
            self.invert = 1

        # Sets the distance to the new target angle
        targetAngleDist = math.fabs(targetAngle - self.currentRot)
        if targetAngleDist > 0.5:
            targetAngleDist = abs(targetAngleDist - 1)


        # Sets the angle difference between the targetAngle and the currentRot
        angleDifference = targetAngle - self.currentRot
        
        # If the distance is less than 0, add 360
        if angleDifference < 0:
            angleDifference += 1
        
        # If the angleDifference is greater than 180, then we subtract (because 360 - (anything greater than 180) < 180 so it's quicker to move in the other direction)
        if angleDifference > 0.5:
            self.currentRot -= targetAngleDist
        # If not, then its closer by moving in the positive direction
        else:
            self.currentRot += targetAngleDist
        
        # Sets the current angle to the target angle (because that's where we're heading)
        self.currentRot = targetAngle

        # Moves the motor
        self.dir_motor.set_control(PositionVoltage(self.currentRot * k_direction_gear_ratio))
        self.drive_motor.set_control(VelocityVoltage(meters_to_rots(self.invert * desiredState.speed, k_drive_gear_ratio), override_brake_dur_neutral=override_brake_dur_neutral))

# Drivetrain class
class Swerve(commands2.Subsystem):
    
    navx = navx.AHRS.create_spi()
    navx.enableLogging(False)

    # Creates the swerve kinematics object
    kinematics = SwerveDrive4Kinematics(Translation2d(1, 1), Translation2d(-1, 1), Translation2d(1, -1), Translation2d(-1, -1))

    # Creates the four swerve modules
    fl = SwerveModule("FL", DriveMotorConstants(MotorIDs.LEFT_FRONT_DRIVE), DirectionMotorConstants(MotorIDs.LEFT_FRONT_DIRECTION), CANConstants.FL_ID, CANConstants.FL_OFFSET)
    bl = SwerveModule("BL", DriveMotorConstants(MotorIDs.LEFT_REAR_DRIVE), DirectionMotorConstants(MotorIDs.LEFT_REAR_DIRECTION), CANConstants.BL_ID, CANConstants.BL_OFFSET)
    
    fr = SwerveModule("FR", DriveMotorConstants(MotorIDs.RIGHT_FRONT_DRIVE), DirectionMotorConstants(MotorIDs.RIGHT_FRONT_DIRECTION), CANConstants.FR_ID, CANConstants.FR_OFFSET)
    br = SwerveModule("BR", DriveMotorConstants(MotorIDs.RIGHT_REAR_DRIVE), DirectionMotorConstants(MotorIDs.RIGHT_REAR_DIRECTION), CANConstants.BR_ID, CANConstants.BR_OFFSET)

    field = Field2d()

    def __init__(self):

        super().__init__()

        SmartDashboard.putData(self.field)

        reset_yaw = commands2.InstantCommand(lambda: self.reset_yaw())
        reset_yaw.setName("Reset Yaw")
        SmartDashboard.putData("Reset Yaw", reset_yaw)

        self.max_module_speed()

        # Resets the navx yaw when the robot is restarted
        self.navx.reset()

    # Resets the yaw on the navx
    def reset_yaw(self) -> Self:
        self.navx.reset()
        return self

    # Sets the max speed that the modules can move at
    def max_module_speed(self, max_speed: float=SwerveConstants.k_max_module_speed) -> None:
        self.max_speed = max_speed

    # Returns the yaw
    def get_yaw(self) -> Rotation2d:
        return Rotation2d.fromRotations(-self.navx.getYaw())

    # Field Relative
    def drive(self, chassis_speed: ChassisSpeeds, rotation_center: Translation2d=Translation2d()) -> None:

        self.set_all_module_states(self.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(ChassisSpeeds.discretize(chassis_speed, 0.02), self.get_yaw()), centerOfRotation = rotation_center))

    # Sets the module states for each module
    def set_all_module_states(self, module_states: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:

        # Desaturates the inputs to the wheel to within the specified constraints
        desaturated_states = self.kinematics.desaturateWheelSpeeds(module_states, self.max_speed)

        # Sets the state for each module
        self.fl.set_module_state(desaturated_states[0], override_brake_dur_neutral=True)
        self.bl.set_module_state(desaturated_states[1], override_brake_dur_neutral=True)
        self.fr.set_module_state(desaturated_states[2], override_brake_dur_neutral=True)
        self.br.set_module_state(desaturated_states[3], override_brake_dur_neutral=True)

    # Resets the sensor positions once the modules are created
    def init(self):
        self.fl.reset_sensor_pos()
        self.bl.reset_sensor_pos()
        self.fr.reset_sensor_pos()
        self.br.reset_sensor_pos()