from commands2 import WaitCommand

# overload the WaitCommand so our drive does not complain
class DriveWait(WaitCommand):  # change the name for your command

    def __init__(self, container, duration) -> None:
        super().__init__(duration)
        self.setName('DriveWait')  # change this to something appropriate for this command
        self.container = container
        self.drive = self.container.drive
        # self.addRequirements(self.drive)  # commandsv2 version of requirements

    def execute(self) -> None:
        self.drive.feed()
