import commands2
import wpilib

from commands.drive import Drive
from subsystems.swerve import Swerve
from robotcontainer import RobotContainer
from phoenix6.signal_logger import SignalLogger
from wpilib import XboxController, SmartDashboard, DriverStation, RobotBase

# Main Robot class
class Schwervin(commands2.TimedCommandRobot):

    def __init__(self, period: float = wpilib.TimedRobot.kDefaultPeriod / 1000) -> None:

        super().__init__(period)

    def robotInit(self):

        DriverStation.silenceJoystickConnectionWarning(True)

        SignalLogger.enable_auto_logging(False)

        # Creates the RobotContainer object
        self.container = RobotContainer()
    
        