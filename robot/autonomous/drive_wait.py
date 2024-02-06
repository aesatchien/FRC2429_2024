from commands2 import WaitCommand
from subsystems.drivetrain import Drivetrain

# overload the WaitCommand so our drive does not complain
class DriveWait(WaitCommand):  # change the name for your command

    def __init__(self, container, duration) -> None:
        super().__init__(duration)
        self.setName('DriveWait')  # change this to something appropriate for this command
        self.container = container
        self.drive = self.container.drive
        # self.addRequirements(self.drive)  # commandsv2 version of requirements

    def execute(self) -> None:
        if isinstance(self.drive, Drivetrain):
            self.drive.feed()        
        else: pass
