import commands2
from wpilib import SmartDashboard
from subsystems.swerve import Swerve
from wpimath.geometry import Pose2d
from wpimath.filter import Debouncer


class GyroReset(commands2.Command):
    def __init__(self, container, swerve: Swerve, angle=None, from_pose=False) -> None:
        super().__init__()
        self.setName('GyroReset')
        self.container = container
        self.swerve = swerve
        self.counter = 0
        self.angle = angle
        self.from_pose = from_pose  # determine if we want to set the angle from the pose
        self.addRequirements(self.swerve)  # commandsv2 version of requirements

    def runsWhenDisabled(self):  # ok to run when disabled - override the base method
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        # All this command does for now is reset the gyro, although we may need to add more
        # self.swerve.gyro.reset()
        if self.from_pose:
            angle = self.swerve.get_pose().rotation().degrees()  # get this from the swerve's current pose - maybe the yaw
            self.swerve.reset_gyro(adjustment=angle)
        elif self.angle is None:
            self.swerve.reset_gyro()  # also update the keep_angle
        else:
            self.swerve.reset_gyro(adjustment=self.angle)


    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        # return not self.debouncer.calculate(self.container.driver_controller.getRawButton(1))
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")