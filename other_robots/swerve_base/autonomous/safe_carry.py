import commands2

import constants

from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from commands.elevator_move import ElevatorMove
from commands.wrist_move import WristMove
from commands.arm_move import ArmMove
from commands.turret_move import TurretMove


class SafeCarry(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('SafeCarry')  # change this to something appropriate for this command
        self.container = container

        # Step 1.a
        # raise elevator to appropriate height
        self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator,
                                      setpoint=Elevator.positions['upper_pickup'], wait_to_finish=False))

        # Step 1.b
        # Get turret into position - get ready to score or keep CG over robot center?  TODO

        # Step 2.a
        # lower wrist to horizontal
        self.addCommands(WristMove(container=self.container, wrist=self.container.wrist,
                                   setpoint=Wrist.positions['score'], wait_to_finish=False))
        # Step 2.b
        # extend the arm fully / to the cone/cube
        self.addCommands(ArmMove(container=self.container, arm=self.container.arm,
                                 setpoint=self.container.arm.min_extension, wait_to_finish=False))

        # Step
        # move turret to 0 - our default driving position
        #self.addCommands(TurretMove(container=self.container, turret=self.container.turret, setpoint=0, wait_to_finish=False))





