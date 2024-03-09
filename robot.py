import commands2
from commands.drive import Drive
from subsystems.swerve import Swerve
from wpilib import XboxController, SmartDashboard

class Schwervin(commands2.TimedCommandRobot):

    def __init__(self, period: float = wpilib.TimedRobot.kDefaultPeriod / 1000) -> None:

        super().__init__(period)