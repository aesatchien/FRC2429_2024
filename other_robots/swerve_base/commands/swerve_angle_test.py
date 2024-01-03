import commands2
from wpilib import SmartDashboard
from subsystems.swerve import Swerve
from wpimath.filter import Debouncer
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d
import math
from wpimath.filter import SlewRateLimiter


class SwerveAngleTest(commands2.CommandBase):

    def __init__(self, container, swerve: Swerve) -> None:
        super().__init__()
        self.setName('SwerveAngleTest')
        self.container = container
        self.swerve = swerve
        self.counter = 0

        self.debouncer = Debouncer(debounceTime=0.1)
        self.addRequirements(self.swerve)  # commandsv2 version of requirements
        self.magLimiter = SlewRateLimiter(6.28)  # rate is units per second

    def runsWhenDisabled(self):  # ok to run when disabled - override the base method
        return False

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")


    def execute(self) -> None:
        angle = - math.pi * self.container.driver_controller.getRawAxis(1)
        angle = self.magLimiter.calculate(angle)   # test a rate limiter
        for module in self.swerve.swerve_modules:
            module.setDesiredState(SwerveModuleState(speed=0, angle=Rotation2d(angle)))

    def isFinished(self) -> bool:
        return not self.debouncer.calculate(self.container.driver_controller.getRawButton(3))

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")