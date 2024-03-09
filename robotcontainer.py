import commands2
from commands2.button import JoystickButton
from commands.drive import Drive
from subsystems.swerve import Swerve
from wpilib import XboxController

from constants import *

class RobotContainer:

    def __init__(self):

        self.swerve: Swerve = Swerve()

        self.swerve.init()

        self.driver_controller = XboxController(ExternalConstants.DRIVERCONTROLLER)

        self.swerve.setDefaultCommand(Drive(self.swerve, self.driver_controller))

        JoystickButton(self.driver_controller, XboxController.Button.kRightBumper).whileTrue(commands2.InstantCommand(lambda: Drive.change_center_of_rotation(5)))

