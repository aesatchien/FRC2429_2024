import math
import commands2
from wpilib import SmartDashboard
from wpilib import Timer
from commands2 import SwerveControllerCommand
from wpimath.trajectory import TrajectoryConfig, Trajectory, TrajectoryGenerator
import wpimath.geometry as geo
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController

from subsystems.swerve import Swerve
from subsystems.swerve_constants import DriveConstants as dc
from subsystems.swerve_constants import AutoConstants as ac

class DriveSwervePointTrajectory(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, drive: Swerve, pointlist=None, velocity=None, acceleration=None) -> None:
        super().__init__()
        self.setName('DriveSwervePointTrajectory')  # change this to something appropriate for this command
        self.container = container
        self.drive: Swerve = drive
        self.addRequirements(self.drive)  # commandsv2 version of requirements

        self.pointlist = [geo.Translation2d(1,0), geo.Translation2d(1,-1), geo.Translation2d(0,-1)] if pointlist is None else pointlist
        self.velocity = dc.kMaxSpeedMetersPerSecond if velocity is None else velocity
        self.acceleration = ac.kMaxAccelerationMetersPerSecondSquared if acceleration is None else acceleration
        # configure and create a trajectory - note this is overloaded with many ways to call
        self.trajectory_config = TrajectoryConfig(self.velocity, self.acceleration)
        self.trajectory_config.setKinematics(dc.kDriveKinematics)
        self.trajectory = TrajectoryGenerator.generateTrajectory(
            start=geo.Pose2d(0,0, geo.Rotation2d(0)),
            interiorWaypoints=self.pointlist,
            end=geo.Pose2d(0,0, geo.Rotation2d(0)),
            config=self.trajectory_config
        )
        print(self.trajectory)
        # set up PID controllers for forward, strafe and rotation
        self.x_controller = PIDController(0.0,0,0)
        self.y_controller = PIDController(0.0, 0, 0)
        self.theta_controller = ProfiledPIDControllerRadians(Kp=0.3,Ki=0,Kd=0,constraints=ac.kThetaControllerConstraints)
        self.theta_controller.enableContinuousInput(-math.pi, math.pi)
        self.controller = HolonomicDriveController(self.x_controller, self.y_controller, self.theta_controller)
        self.timer = Timer()
        self.pose = self.drive.get_pose  # function returning the pose of the swerve drive
        self.kinematics = dc.kDriveKinematics
        self.outputModuleStates = self.drive.setModuleStates

        # make the swerve controller command - eventually we want to use this but right now we are in learning mode
        swerve_controller_command = SwerveControllerCommand(
            trajectory=self.trajectory,
            pose=self.drive.get_pose,
            kinematics=dc.kDriveKinematics,
            controller=self.controller,
            outputModuleStates=self.drive.setModuleStates,
            requirements=[self.drive]
        )

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.timer.restart()

    def execute(self) -> None:
        # NOTE - THIS CAN ALL BE DONE BY THE SwerveControllerCommand - I'm just copying it for now to test it
        current_time = self.timer.get()
        desiredState = self.trajectory.sample(current_time)

        targetChassisSpeeds = self.controller.calculate(
            self.pose(), desiredState, self.trajectory.states()[-1].pose.rotation()
        )
        targetModuleStates = self.kinematics.toSwerveModuleStates(targetChassisSpeeds)

        self.outputModuleStates(targetModuleStates)


    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.trajectory.totalTime())

    def end(self, interrupted: bool) -> None:
        self.timer.stop()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")