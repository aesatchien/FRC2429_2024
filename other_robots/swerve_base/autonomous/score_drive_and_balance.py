import commands2

import constants

from autonomous.charge_station_balance import ChargeStationBalance
from autonomous.score_hi_cone_from_stow import ScoreHiConeFromStow
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from autonomous.drive_and_balance import DriveAndBalance


class ScoreDriveAndBalance(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('DriveAndBalance')  # change this to something appropriate for this command
        self.container = container

        self.addCommands(ScoreHiConeFromStow(self.container))

        self.addCommands(DriveAndBalance(container=self.container))

        # drive onto the charge station
        # self.addCommands(DriveMove(container=self.container, drive=self.container.drive,
        #                               setpoint=1, wait_to_finish=True).withTimeout(5))
        # self.addCommands(DriveMove(self.container, self.container.drive, 1))

        # new swerve version to drive ~ 1 meter - use velocity and time for now
        #self.addCommands(DriveSwerveAutoVelocity(container=self.container, drive=self.container.drive, velocity=1).withTimeout(1.5))

        # maintain balance
        #self.addCommands(ChargeStationBalance(container=self.container, drive=self.container.drive))




