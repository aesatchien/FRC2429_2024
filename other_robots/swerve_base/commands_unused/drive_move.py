'''returns a command for a SequentialCommandGroup to run'''
import commands2
import math
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from subsystems.swerve import Swerve
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from subsystems.swerve_constants import DriveConstants as dc, AutoConstants

class DriveMove(commands2.SequentialCommandGroup):
    def __init__(self, container, drive: Swerve, setpoint=0) -> None:
        # Tank drive + smartmotion method
        super().__init__()

        self.addCommands(DriveSwerveAutoVelocity(container, drive, setpoint))

        # Trajectory method
        if False:
            self.container = container

            # setup trajectory
            config = TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared,
            )
            # Add kinematics to ensure max speed is actually obeyed
            config.setKinematics(dc.kDriveKinematics)

            # An example trajectory to follow. All units in meters.
            trajectory = TrajectoryGenerator.generateTrajectory(
                # Start at the origin facing the +X direction and end setpoint meters straight ahead of where we started, facing forward
                [Pose2d(0, 0, Rotation2d(0)), Pose2d(setpoint, 0, Rotation2d(0))],
                config,
            )

            thetaController = ProfiledPIDControllerRadians(
                AutoConstants.kPThetaController,
                0,
                0,
                AutoConstants.kThetaControllerConstraints,
            )
            thetaController.enableContinuousInput(-math.pi, math.pi)

            swerveControllerCommand = commands2.Swerve4ControllerCommand(
                trajectory,
                drive.get_pose,  # Functional interface to feed supplier
                dc.kDriveKinematics,
                # Position controllers
                PIDController(AutoConstants.kPXController, 0, 0),
                PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                drive.setModuleStates,
                [drive],
            )

            # Reset odometry to the starting pose of the trajectory.
            drive.resetOdometry(trajectory.initialPose())

            # Run path following command, then stop at the end.
            self.addCommands(swerveControllerCommand.andThen(
                lambda: drive.drive(0, 0, 0, False, False)
            ))
