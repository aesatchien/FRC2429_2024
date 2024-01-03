import commands2

from autonomous.score_hi_cone_from_stow import ScoreHiConeFromStow
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity

class ScoreHiAndMove(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container, distance=2) -> None:
        super().__init__()
        self.setName('ScoreHiAndMove')  # change this to something appropriate for this command
        self.container = container

        # Step 1.a
        # score hi from stow
        self.addCommands(ScoreHiConeFromStow(container=self.container))

        # Step 2.a
        # drive forward
        # note: hi score doesn't reset rotation anymore (due to break during auto)
        # self.addCommands(DriveMove(container=self.container, drive=self.container.drive, setpoint=1.5))

        # new swerve version to drive ~ 2 meter - use velocity and time for now
        self.addCommands(DriveSwerveAutoVelocity(container=self.container, drive=self.container.drive, velocity=1).withTimeout(distance))

