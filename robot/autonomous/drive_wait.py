# "feed()" is not a method of the Swerve class and will cause an error if used on a Swerve object.

from commands2 import WaitCommand
from robot.robotcontainer import RobotContainer
from subsystems.swerve import Swerve

# overload the WaitCommand so our drive does not complain
class DriveWait(WaitCommand):  # change the name for your command

    def __init__(self, container, duration) -> None:
        super().__init__(duration)
        self.setName('DriveWait')  # change this to something appropriate for this command
        self.container : RobotContainer = container
        self.drive : Swerve = self.container.drive
        # self.addRequirements(self.drive)  # commandsv2 version of requirements

    def execute(self) -> None:
        if isinstance(self.drive, Swerve):
            self.drive.feed() #"feed()" is not a method of the Swerve class. "feed()" resets the timer on the actuator that is used to do the timeouts (for the DifferentialDrive class).
        else: pass
