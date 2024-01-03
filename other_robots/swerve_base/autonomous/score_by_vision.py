import commands2

from subsystems.elevator import Elevator
from subsystems.turret import Turret
from subsystems.arm import Arm
from subsystems.pneumatics import Pneumatics
from subsystems.vision import Vision
from commands.elevator_move import ElevatorMove
from commands.turret_move import TurretMove
from commands.wrist_move import WristMove
from autonomous.arm_move_by_vision import ArmMoveByVision
from commands.manipulator_toggle import ManipulatorToggle
from autonomous.turret_move_by_vision import TurretMoveByVision


class ScoreByVision(commands2.SequentialCommandGroup):
    def __init__(self, container, turret: Turret, elevator: Elevator, arm: Arm, pneumatics: Pneumatics, vision: Vision, position='top'):
        super().__init__()
        self.setName('ScoreByVision')

        self.addCommands(ElevatorMove(container=container, elevator=elevator, setpoint=elevator.positions[position], wait_to_finish=True))
        self.addCommands(TurretMoveByVision(container=container, turret=turret, vision=vision, color='green', wait_to_finish=True))
        self.addCommands(ArmMoveByVision(container=container, arm=arm, vision=vision, color='green', wait_to_finish=True))
        self.addCommands(ManipulatorToggle(container=container, pneumatics=pneumatics, force='open'))