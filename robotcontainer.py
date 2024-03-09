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


