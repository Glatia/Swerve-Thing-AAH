import commands2
from commands2.button import JoystickButton
from commands.drive import Drive
from subsystems.swerve import Swerve
from wpilib import XboxController

from constants import *

# RobotContainer class
class RobotContainer:

    def __init__(self):

        # Creates the swerve object
        self.swerve: Swerve = Swerve()

        # Resets the module angles 
        self.swerve.init()

        self.driver_controller = XboxController(ExternalConstants.DRIVERCONTROLLER)

        # When no other command is scheduled for Swerve, Drive runs
        self.swerve.setDefaultCommand(Drive(self.swerve, self.driver_controller))


        JoystickButton(self.driver_controller, XboxController.Button.kRightBumper).onTrue(commands2.InstantCommand(lambda: Drive.change_center_of_rotation(5.0)))
        JoystickButton(self.driver_controller, XboxController.Button.kRightBumper).onFalse(commands2.InstantCommand(lambda: Drive.change_center_of_rotation(0.0)))

