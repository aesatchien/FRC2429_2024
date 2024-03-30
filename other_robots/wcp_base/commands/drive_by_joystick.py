import typing
import commands2
from subsystems.drivetrain import Drivetrain  # allows us to access the definitions
from wpilib import SmartDashboard

class DriveByJoystick(commands2.CommandBase):
    def __init__(
        self, container,
        drive: Drivetrain,
        forward: typing.Callable[[], float],
        rotation: typing.Callable[[], float],
    ) -> None:

        super().__init__()
        self.setName('drive_by_joystick')
        self.container = container
        self.drive = drive
        self.forward = forward
        self.rotation = rotation

        self.addRequirements([self.drive])

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

    def execute(self) -> None:
        self.drive.arcade_drive(self.forward(), self.rotation())

    def end(self, interrupted: bool) -> None:
        self.drive.arcade_drive(0,0)

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")