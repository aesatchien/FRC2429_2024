import commands2

import constants

from subsystems.wrist import Wrist
from subsystems.elevator import Elevator
from commands.elevator_move import ElevatorMove
from commands.wrist_move import WristMove
from commands.arm_move import ArmMove
from commands.turret_move import TurretMove
from commands.manipulator_toggle import ManipulatorToggle
from autonomous.drive_wait import DriveWait


class ReleaseAndStow(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('ReleaseAndStow')  # change this to something appropriate for this command
        self.container = container

        # Open the manipulator
        self.addCommands(ManipulatorToggle(container=self.container, pneumatics=self.container.pneumatics, force='open'))

        # wait a bit for the paddles to let go
        self.addCommands(DriveWait(container=self.container, duration=0.5))

        # raise wrist to driving position
        self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['stow'], wait_to_finish=False))

        # retract the arm fully
        self.addCommands(ArmMove(container=self.container, arm=self.container.arm, setpoint=self.container.arm.min_extension, wait_to_finish=False))

        # bring the elevator down
        self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator,
                                      setpoint=Elevator.positions['lower_pickup'], wait_to_finish=False))






