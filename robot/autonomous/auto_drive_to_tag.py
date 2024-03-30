import commands2
from wpilib import SmartDashboard

from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from subsystems.swerve import Swerve
from subsystems.led import Led
from autonomous.pathplannermaker import PathPlannerConfiguration
from pathplannerlib.auto import PathPlannerPath, PathPlannerAuto
from pathplannerlib.path import PathConstraints, GoalEndState
from subsystems.swerve_constants import AutoConstants as ac
import wpimath.geometry as geo


class CommandTemplate(commands2.Command):  # change the name for your command

    def __init__(self, container, destination='stage') -> None:
        super().__init__()
        self.setName('AutoDriveToTag')  # change this to something appropriate for this command
        self.container = container
        self.drive: Swerve = self.container.drive
        self.destination = destination
        self.addRequirements(self.container.drive)  # commands v2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        # copying template from Josh
        # figure out where we are going
        target_pose = self.drive.get_nearest_tag(location=self.destination)
        x = target_pose.translation().x
        y = target_pose.translation().y
        rot = target_pose.rotation().degrees()

        current_pose = self.drive.get_pose()

        if current_pose.translation().distance(
                target_pose.translation()) < 0.1:  # bezier curve generation requires the robot to move a non-zero distance.
            command = DriveSwerveAutoVelocity(self.drive, 0).withTimeout(0.1)  # stop

        position_list = {"x": x - current_pose.X(), "y": y - current_pose.Y(), "rotation": rot - self.drive.get_angle()}
        # create a Transform2d object that contains the position matrix and rotation matrix of the desired position.
        delta_pose = geo.Transform2d(geo.Translation2d(position_list["x"], position_list["y"]), geo.Rotation2d.fromDegrees(0))

        start_pose = geo.Pose2d(current_pose.translation(), current_pose.rotation())
        end_pose = start_pose.transformBy(delta_pose)

        linear_speed_factor = 1
        angular_speed_factor = 1
        fast_turn = True
        final_velocity = 0
        bezier_points = PathPlannerPath.bezierFromPoses([start_pose, end_pose])
        path = PathPlannerPath(  # assumes holonomic drivetrain.
            bezier_points,
            PathConstraints(ac.kMaxSpeedMetersPerSecond * linear_speed_factor,
                            ac.kMaxAccelerationMetersPerSecondSquared * linear_speed_factor,
                            ac.kMaxAngularSpeedRadiansPerSecond * angular_speed_factor,
                            ac.kMaxAngularSpeedRadiansPerSecondSquared * angular_speed_factor),
            GoalEndState(final_velocity, geo.Rotation2d.fromDegrees(position_list["rotation"]), rotateFast=fast_turn)
        )
        cmd = self.drive.AutoBuilder.followPath(path).andThen(
            self.container.led.set_indicator_with_timeout(Led.Indicator.CALIBRATION_SUCCESS, 1))

        cmd.scedule()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:

        self.drive.reset_keep_angle()

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = False
        if print_end_message:
            print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")