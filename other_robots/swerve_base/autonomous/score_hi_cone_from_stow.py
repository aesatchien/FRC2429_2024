import commands2

import constants
from autonomous.drive_wait import DriveWait
from commands.elevator_move import ElevatorMove
from commands.turret_move import TurretMove
from commands.manipulator_toggle import ManipulatorToggle
from commands.arm_move import ArmMove
from commands.wrist_move import WristMove
from subsystems.wrist import Wrist

class ScoreHiConeFromStow(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('ScoreHiConeFromStow')  # change this to something appropriate for this command
        self.container = container

        # Step 1.a
        # raise the elevator , don't wait to end - can go concurrently with other moves
        self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator,
                                      setpoint=self.container.elevator.max_height, wait_to_finish=False))
        # Step 2.a
        # lower / raise wrist to upper scoring position 45°
        self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['score'], wait_to_finish=False))

        # Step 2.b
        # extend the arm fully
        self.addCommands(ArmMove(container=self.container, arm=self.container.arm,
                                 setpoint=self.container.arm.max_extension, wait_to_finish=False).withTimeout(5))

        # Step 1.b
        # Get turret into position - this will take 1.25s to get there but we move to next step anyway
        self.addCommands(TurretMove(container=self.container, turret=self.container.turret, setpoint=182, wait_to_finish=True))

        # Optional - center turret on post

        # Step 4
        # Drop the wrist to level
        self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['flat'], wait_to_finish=True).withTimeout(1))

        # Step 5
        # Open the manipulator
        self.addCommands(ManipulatorToggle(container=self.container, pneumatics=self.container.pneumatics, force='open'))

        # wait a bit
        self.addCommands(DriveWait(container=self.container, duration=0.25)) # 0.5

        # Step 6 bring the wrist back up
        # Drop the wrist to level
        # self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['score'], wait_to_finish=False).withTimeout(2))
        self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['stow'], wait_to_finish=True).withTimeout(0.5))

        # step 7 - arm goes back in
        self.addCommands(ArmMove(container=self.container, arm=self.container.arm,
                                 setpoint=self.container.arm.min_extension, wait_to_finish=True).withTimeout(0.75))

        # Step 1.b
        # Get turret into position - this will take 1.25s to get there but we move to next step anyway
        # self.addCommands(TurretMove(container=self.container, turret=self.container.turret, setpoint=0, wait_to_finish=False).withTimeout(1))


         # Step 1.a
        # raise the elevator , don't wait to end - can go concurrently with other moves
        self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator,
                                      setpoint=300, wait_to_finish=False))

        self.addCommands(TurretMove(container=self.container, turret=self.container.turret, setpoint=0, wait_to_finish=True))