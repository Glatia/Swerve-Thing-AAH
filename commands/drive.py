from commands2 import Command
from constants import *
from enum import Enum
from math import fabs
from subsystems.swerve import Swerve
from wpilib import XboxController
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds

class Drive(commands2.Command):

    def __init__(self, swerve: Swerve, driver_controller: XboxController):
        super().__init__()

        self.swerve = swerve
        self.driver_controller = driver_controller

        self.addRequirements(self.swerve)

        speed_multis = [0.5, 1]
        speed_index = 0

    def execute(self) -> None:

        # For Chassis Speeds, X is forward, which would be the Y on the Xbox Controller.
        trans_x = (-deadband(self.driver_controller.getLeftX(), ExternalConstants.DEADBAND) ** 3) * SwerveConstants.k_max_module_speed
        trans_y = (-deadband(self.driver_controller.getLeftY(), ExternalConstants.DEADBAND) ** 3) * SwerveConstants.k_max_module_speed
        rotation = (-deadband(self.driver_controller.getRightX(), ExternalConstants.DEADBAND) ** 3) * SwerveConstants.k_max_rot_rate

        if self.driver_controller.getLeftStickButton():
            (speed_index ++ 1) % 2
        
        center_of_rotation = Translation2d()

        self.swerve.drive(ChassisSpeeds(trans_x * speed_multis[speed_index], trans_y * speed_multis[speed_index], rotation * speed_multis[speed_index]), center_of_rotation = center_of_rotation)


    def end(self, interrupted:bool):
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False

def deadband(value, deadband) -> float:
    if fabs(value) <= deadband: return 0.0
    else: return value