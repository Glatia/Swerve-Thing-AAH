from commands2 import Command
from constants import *
from enum import Enum
from math import fabs
from subsystems.swerve import Swerve
from wpilib import XboxController
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds

class Drive(Command):

    def __init__(self, swerve: Swerve, driver_controller: XboxController):
        super().__init__()

        self.swerve = swerve
        self.driver_controller = driver_controller

        self.addRequirements(self.swerve)

        self.speed_multis = [0.5, 1]
        self.speed_index = 0

        self.center_of_rotation = Translation2d()

    def execute(self) -> None:

        # For Chassis Speeds, X is forward, which would be the Y on the Xbox Controller.
        trans_x = (-deadband(self.driver_controller.getLeftX(), ExternalConstants.DEADBAND) ** 3) * SwerveConstants.k_max_module_speed
        trans_y = (-deadband(self.driver_controller.getLeftY(), ExternalConstants.DEADBAND) ** 3) * SwerveConstants.k_max_module_speed
        rotation = (-deadband(self.driver_controller.getRightX(), ExternalConstants.DEADBAND) ** 3) * SwerveConstants.k_max_rot_rate

        # Kind of goofy slowdown method
        if self.driver_controller.getLeftStickButton():
            (self.speed_index ++ 1) % 2
        
        # Sets the center of rotation to the center of the Navx (no translation)
        
        self.swerve.drive(ChassisSpeeds(trans_x * self.speed_multis[self.speed_index], trans_y * self.speed_multis[self.speed_index], rotation * self.speed_multis[self.speed_index]), center_of_rotation = self.center_of_rotation)

    def change_center_of_rotation(self, center: bool):

        self.center_of_rotation = Translation2d(center, 0.0)

    def end(self, interrupted: bool) -> bool:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False

def deadband(value, deadband) -> float:
    if fabs(value) <= deadband: return 0.0
    else: return value