import commands2

from subsystems.wrist import Wrist
from subsystems.arm import Arm

from autonomous.score_hi_cone_from_stow import ScoreHiConeFromStow
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity

from commands.manipulator_toggle import ManipulatorToggle
from commands.wrist_move import WristMove
from commands.elevator_move import ElevatorMove
from commands.arm_move import ArmMove
from commands.turret_move import TurretMove

class ScoreTwice(commands2.SequentialCommandGroup):
    # LHACK, scores twice. Commented out everything after backing up from grid bc I don't want to tip bot hueneme-style
    def __init__(self, container) -> None:
        super().__init__()
        self.setName('Score Twice')
        self.container = container

        # Drop first cone
        self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator,
                                      setpoint=self.container.elevator.max_height, wait_to_finish=False))

        self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['score'], wait_to_finish=False))

        self.addCommands(ArmMove(container=self.container, arm=self.container.arm,
                                 setpoint=self.container.arm.max_extension, wait_to_finish=False).withTimeout(5))

        self.addCommands(TurretMove(container=self.container, turret=self.container.turret, setpoint=182, wait_to_finish=True))

        self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['flat'], wait_to_finish=True).withTimeout(1))

        self.addCommands(ManipulatorToggle(container=self.container, pneumatics=self.container.pneumatics, force='open'))

        # drive backwards a bit before starting superstructure stuff
        self.addCommands(DriveSwerveAutoVelocity(container=self.container, drive=self.container.drive, velocity=0.5).withTimeout(0.25))

        if False:
            self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['floor'], wait_to_finish=False))

            self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator, setpoint=self.container.elevator.positions['bottom'], wait_to_finish=False))

            self.addCommands(TurretMove(container=self.container, turret=self.container.turret, setpoint=2, wait_to_finish=False))

            # Drive to cone
            self.addCommands(DriveSwerveAutoVelocity(container=self.container, drive=self.container.drive, velocity=2).withTimeout(1.75))

            # Grab it
            self.addCommands(ManipulatorToggle(container=self.container, pneumatics=self.container.pneumatics, force='close'))

            # Drive back and score on mid

            # Start moving superstructure
            self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['stow'], wait_to_finish=False))

            self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator, setpoint=self.container.elevator.positions['low'], wait_to_finish=False))

            self.addCommands(ArmMove(container=self.container, arm=self.container.arm,
                                     setpoint=self.container.arm.positions['middle'], wait_to_finish=False).withTimeout(5))

            self.addCommands(TurretMove(container=self.container, turret=self.container.turret, setpoint=182, wait_to_finish=False))

            # Drive to grid
            self.addCommands(DriveSwerveAutoVelocity(container=self.container, drive=self.container.drive, velocity=-2).withTimeout(2))

            # Drop
            self.addCommands(WristMove(container=self.container, wrist=self.container.wrist, setpoint=Wrist.positions['flat'], wait_to_finish=True).withTimeout(1))

            self.addCommands(ManipulatorToggle(container=self.container, pneumatics=self.container.pneumatics, force='open'))

