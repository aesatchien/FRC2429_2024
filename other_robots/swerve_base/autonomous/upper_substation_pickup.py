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
from autonomous.turret_move_by_vision import TurretMoveByVision


class UpperSubstationPickup(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('UpperSubstationPickup')  # change this to something appropriate for this command
        self.container = container

        # Open the manipulator
        self.addCommands(ManipulatorToggle(container=self.container, pneumatics=self.container.pneumatics, force='open'))

        # raise wrist to 45 degree position
        self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['score'], wait_to_finish=False))

        # bring the elevator down to get a good view
        self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator,
                                      setpoint=850, wait_to_finish=True))

        # center on cube or cone
        self.addCommands(TurretMoveByVision(container=self.container, turret=self.container.turret, vision=self.container.vision, wait_to_finish=False))

        # bring the elevator up to the top
        self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator, setpoint=Elevator.positions['top'], wait_to_finish=False))

        # extend the arm fully
        self.addCommands(ArmMove(container=self.container, arm=self.container.arm, setpoint=self.container.arm.max_extension, wait_to_finish=True))

        # lower wrist to flat
        self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['flat'], wait_to_finish=True))

        # close the manipulator
        self.addCommands(ManipulatorToggle(container=self.container, pneumatics=self.container.pneumatics, force='close'))

        # wait a bit for the paddles to clamp
        self.addCommands(DriveWait(container=self.container, duration=0.75))

        # raise wrist to score
        self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['score'], wait_to_finish=True))

        # retract the arm fully
        self.addCommands(ArmMove(container=self.container, arm=self.container.arm, setpoint=self.container.arm.min_extension, wait_to_finish=False))

        # bring the elevator down
        self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator,
                                      setpoint=Elevator.positions['lower_pickup'], wait_to_finish=False))






