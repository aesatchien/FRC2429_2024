import commands2
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from subsystems.swerve import Swerve
from subsystems.swerve_constants import DriveConstants as dc


class DriveSwerveAutoVelocity(commands2.Command):  # change the name for your command

    def __init__(self, container, drive: Swerve, velocity, direction='forwards', decide_by_turret=False) -> None:
        super().__init__()
        self.setName('DriveSwerveAutoVelocity')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.addRequirements(self.drive)  # commandsv2 version of requirements
        self.setpoint_velocity = velocity  # in m/s, gets normalized when sent to drive
        self.decide_by_turret = decide_by_turret  # use this to determine direction for auto scoring
        self.direction = direction

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:

        sign = 1
        # drive at the velocity passed to the function
        if self.direction == 'forwards':
            self.drive.drive(sign * self.setpoint_velocity / dc.kMaxSpeedMetersPerSecond, 0, 0, False, False)
        elif self.direction == 'strafe':
            self.drive.drive(0, self.setpoint_velocity / dc.kMaxSpeedMetersPerSecond, 0, False, False)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.drive.drive(0,0,0,True,True)  # what should we do here?
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")