import commands2

import constants

from autonomous.drive_move import DriveMove
from autonomous.charge_station_balance import ChargeStationBalance
from autonomous.drive_climber import DriveClimber


class DriveAndBalance(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('DriveAndBalance')  # change this to something appropriate for this command
        self.container = container

        # drive onto the charge station
        # self.addCommands(DriveMove(container=self.container, drive=self.container.drive,
        #                               setpoint=1, wait_to_finish=True).withTimeout(5))
        self.addCommands(DriveClimber(container=self.container, drive=self.container.drive, setpoint_velocity=90, setpoint_distance=1.5).withTimeout(1.5))

        # maintain balance
        self.addCommands(ChargeStationBalance(container=self.container, drive=self.container.drive, velocity=12, tolerance=10, auto=True).withTimeout(7))




