import commands2
from constants import *
from enum import Enum
from math import fabs
from subsystems.swerve import Swerve
from wpilib import XboxController
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds

class Drive(commands2.Command):

    def __init__(self, swerve: Swerve, driver_controller: XboxController) -> None:
        super().__init__()

        self.swerve = swerve
        self.driver_controller = driver_controller

        self.addRequirements(self.swerve)

        self.speed_multi = 0.5

        self.center_of_rotation = Translation2d()

    def execute(self) -> None:

        # For Chassis Speeds, X is forward, which would be the Y on the Xbox Controller.
        trans_x = (-(self.driver_controller.getLeftX()) ** 3) * SwerveConstants.k_max_module_speed
        trans_y = (-(self.driver_controller.getLeftY()) ** 3) * SwerveConstants.k_max_module_speed
        rotation = (-(self.driver_controller.getRightX()) ** 3) * SwerveConstants.k_max_rot_rate

        # Kind of goofy slowdown method
        if self.driver_controller.getLeftStickButton():
            self.speed_index = (self.speed_index + 1) % 2
        
        
        self.swerve.drive(ChassisSpeeds(trans_y * self.speed_multi, trans_x * self.speed_multi, rotation * self.speed_multi), self.center_of_rotation)

    def change_center_of_rotation(self, center: float):

        self.center_of_rotation = Translation2d(center, 0.0)

    def end(self, interrupted: bool) -> bool:
        return super().end(interrupted)
    
    def isFinished(self) -> bool:
        return False