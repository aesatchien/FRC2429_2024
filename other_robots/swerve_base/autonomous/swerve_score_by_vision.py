import commands2

from subsystems.turret import Turret
from subsystems.swerve import Swerve
from subsystems.arm import Arm
from subsystems.vision import Vision
from subsystems.elevator import Elevator
from subsystems.wrist import Wrist
from subsystems.pneumatics import Pneumatics
from commands.elevator_move import ElevatorMove
from commands.arm_move import ArmMove
from commands.wrist_move import WristMove
from commands.manipulator_toggle import ManipulatorToggle
from .elevator_move_by_turret import ElevatorMoveByTurret
from .arm_move_by_turret import ArmMoveByTurret
from .auto_aim_swerve import AutoAimSwerve

class SwerveScoreByVision(commands2.SequentialCommandGroup):
    # LHACK, lines up and scores piece. Decides whether to score on hi or mid based on turret position. Untested.
    def __init__(self, container):
        super().__init__()
        self.setName("Swerve Score by Vision")

        self.addCommands(ElevatorMoveByTurret(container, container.elevator, container.turret, wait_to_finish=False))

        self.addCommands(ArmMoveByTurret(container, container.arm, container.turret, wait_to_finish=False))

        self.addCommands(AutoAimSwerve(container, container.drive, container.vision, auto=False).withTimeout(2)) # Should change this to CJH auto_strafe_swerve

        self.addCommands(WristMove(container, container.wrist, setpoint=container.wrist.positions['flat'], wait_to_finish=True))

        self.addCommands(ManipulatorToggle(container, container.pneumatics, force='open'))
