import commands2

import constants

from autonomous.charge_station_balance import ChargeStationBalance
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity

class DriveAndBalance(commands2.SequentialCommandGroup):

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('DriveAndBalance')  # change this to something appropriate for this command
        self.container = container

        # drive onto the charge station
        # self.addCommands(DriveMove(container=self.container, drive=self.container.drive,
        #                               setpoint=1, wait_to_finish=True).withTimeout(5))
        self.addCommands(DriveSwerveAutoVelocity(self.container, self.container.drive, velocity=1.75).withTimeout(1))

        # maintain balance
        self.addCommands(ChargeStationBalance(self.container, self.container.drive).withTimeout(10))




