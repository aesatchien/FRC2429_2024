#JS 3/11/24
#
#Problems: This does NOT WORK. DO NOT USE.
# - Here's what's happening:
    # - Command is called by button
    # - Command builds the PathPlannerPath and tells the robot to follow it via AutoBuilder.
    # - Command ends immediately after calling AutoBuilder (such that THIS command ends, while the AutoBuilder command is still running)
    # The problems are:
        #For some reason, because of this configuration (of calling a command within a command), I can't take back control over the robot with the joystick after it starts its path, AND it doesn't travel to the right place either.
#Current solution: sticking with the old code.
    # - The one thing I changed was the "pose_supplier" in the AutoBuilder config in Swerve.py, where I fed it the "get_pose_no_tag" method instead of the "get_pose" method.
    # I'm assuming that we don't need to use apriltags for odometry while following a PathPlanner path, and the simplest way to achieve this is by passing in a different pose_supplier that doesn't use apriltags pose estimation.
#Possible solutions:
    # - Using decorators to "modify" the FollowPathCommand command such that it'll execute "set_apriltag_use(True)" on initialize() and "set_apriltag_use(False)" on end(). 
        # - However, this is tricky, because FollowPathCommand is the parent class of the FollowPathHolomonic, which is called by AutoBuilder() in configuration in Swerve.py.  To "modify" FollowPathCommand's init() and execute() functions would require a series of nested decorators, which complicates this way more than it needs to be.
    # - Use SequentialCommandGroup:
        # self.addCommands(TurnOffAprilTags)
        # self.addCommands(CreatePath) <-- stores PathPlannerPath path in a variable in Swerve.py
        # self.addCommands(AutoBuilder.followPath(drive.get_path()))
        # self.addCommands(TurnOnAprilTags)

import commands2
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d
from pathplannerlib.auto import PathPlannerPath
from pathplannerlib.path import PathConstraints, GoalEndState

import typing
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from subsystems.swerve import Swerve, AutoBuilder
from subsystems.swerve_constants import DriveConstants as dc
from subsystems.swerve_constants import AutoConstants as ac

class PathPlannerConfigurationCommand(commands2.Command):  # change the name for your command

    def __init__(self, container, swerve:Swerve, position_list:typing.Dict[str, float], final_velocity:float, speed_factor=1, fast_turn=False) -> None:
        super().__init__()
        self.setName('PathPlannerConfigurationCommand')  # change this to something appropriate for this command
        self.container = container
        self.drive = swerve
        self.position_list = position_list
        self.final_velocity = final_velocity
        self.speed_factor = speed_factor
        self.fast_turn = fast_turn
        self.addRequirements(self.drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""

        self.drive.set_use_apriltags(False)

        self.position_list = {"x": self.position_list["x"] - self.drive.get_pose().X(), "y": self.position_list["y"] - self.drive.get_pose().Y(), "rotation": self.position_list["rotation"] - self.drive.get_angle()}

        if self.position_list["x"] == 0 and self.position_list["y"] == 0: #bezier curve generation requires the robot to move a non-zero distance.
            return DriveSwerveAutoVelocity(self.drive, 0).withTimeout(0.1) #stop
        
        current_pose = self.drive.get_pose()

        #create a Transform2d object that contains the position matrix and rotation matrix of the desired position.
        delta_pose = Transform2d(Translation2d(self.position_list["x"], self.position_list["y"]), Rotation2d.fromDegrees(0))

        start_pose = Pose2d(current_pose.translation(), current_pose.rotation())
        end_pose = start_pose.transformBy(delta_pose)

        bezier_points = PathPlannerPath.bezierFromPoses([start_pose, end_pose])
        self.path = PathPlannerPath( #assumes holonomic drivetrain.
            bezier_points,
            PathConstraints(ac.kMaxSpeedMetersPerSecond * self.speed_factor, ac.kMaxAccelerationMetersPerSecondSquared * self.speed_factor, ac.kMaxAngularSpeedRadiansPerSecond * self.speed_factor, ac.kMaxAngularSpeedRadiansPerSecondSquared * self.speed_factor),
            GoalEndState(self.final_velocity, Rotation2d.fromDegrees(self.position_list["rotation"]))
        )
        self.path.preventFlipping = True

        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        pass
    
    def end(self, interrupted: bool) -> None:
        AutoBuilder.followPath(self.path).schedule()

        self.drive.set_use_apriltags(True)

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")