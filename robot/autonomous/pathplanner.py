import math
import commands2
import wpilib
from wpilib import SmartDashboard
from wpilib import Timer
from commands2 import SwerveControllerCommand
from wpimath.trajectory import TrajectoryConfig, Trajectory, TrajectoryGenerator
import wpimath.geometry as geo
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from pathlib import Path
import pickle
from datetime import datetime
import pathplannerlib

from subsystems.swerve import Swerve
from subsystems.swerve_constants import DriveConstants as dc
from subsystems.swerve_constants import AutoConstants as ac
import constants

class DriveSwervePointTrajectory(commands2.Command):  # change the name for your command

    def __init__(self, container, drive: Swerve, pointlist=None, velocity=None, acceleration=None) -> None:
        super().__init__()
        self.setName('DriveSwervePointTrajectory')  # change this to something appropriate for this command
        self.container = container
        self.drive: Swerve = drive
        self.addRequirements(self.drive)  # commandsv2 version of requirements

        x, y = constants.k_start_x, constants.k_start_y
        # x, y = 0, 0
        self.pointlist = []  # [geo.Translation2d(x+2,y+0), geo.Translation2d(x+2,y+2), geo.Translation2d(x+0,y+2)] if pointlist is None else pointlist
        self.velocity = dc.kMaxSpeedMetersPerSecond if velocity is None else velocity
        self.acceleration = ac.kMaxAccelerationMetersPerSecondSquared if acceleration is None else acceleration
        # configure and create a trajectory - note this is overloaded with many ways to call
        self.trajectory_config = TrajectoryConfig(self.velocity, self.acceleration)
        self.trajectory_config.setKinematics(dc.kDriveKinematics)
        self.trajectory = TrajectoryGenerator.generateTrajectory(
            start=geo.Pose2d(x, y, geo.Rotation2d(0)),
            interiorWaypoints=self.pointlist,
            end=geo.Pose2d(x, y+2, geo.Rotation2d(3.14)),
            config=self.trajectory_config
        )
        print(self.trajectory)

        # put in some tools for tracking telemetry
        self.counter = 0
        self.telemetry = []
        self.write_telemetry = True  # save telemetry data

        # set up PID controllers for forward, strafe and rotation
        self.x_controller = PIDController(1,0,0)
        self.y_controller = PIDController(1, 0, 0)
        self.theta_controller = ProfiledPIDControllerRadians(Kp=1,Ki=0,Kd=0,constraints=ac.kThetaControllerConstraints)
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
        self.counter = 0
        self.telemetry = []

    def execute(self) -> None:
        # NOTE - THIS CAN ALL BE DONE BY THE SwerveControllerCommand - I'm just copying it for now to test it
        current_time = self.timer.get()
        desiredState = self.trajectory.sample(current_time)

        targetChassisSpeeds = self.controller.calculate(
            self.pose(), desiredState, self.trajectory.states()[-1].pose.rotation()
        )
        targetModuleStates = self.kinematics.toSwerveModuleStates(targetChassisSpeeds)

        self.outputModuleStates(targetModuleStates)

        self.counter += 1
        if self.counter % 10 == 0:  # ten times per second update the telemetry array
            if wpilib.RobotBase.isReal():
                pose = self.drive.get_pose()
            else:
                dash_pose = wpilib.SmartDashboard.getNumberArray('drive_pose', [])
                pose = geo.Pose2d(dash_pose[0], dash_pose[1], geo.Rotation2d.fromDegrees(dash_pose[2]))

            telemetry_data = {'TIME': current_time, 'RBT_X': pose.X(), 'RBT_Y': pose.Y(),
                              'RBT_TH': pose.rotation().radians(),
                              #'RBT_VEL': self.container.robot_drive.get_average_encoder_rate(),
                              #'RBT_RVEL': ws_right, 'RBT_LVEL': ws_left,
                              'TRAJ_X': desiredState.pose.X(), 'TRAJ_Y': desiredState.pose.Y(),
                              'TRAJ_TH': desiredState.pose.rotation().radians(), 'TRAJ_VEL': desiredState.velocity,
                              #'RAM_VELX': ramsete.vx, 'RAM_LVEL_SP': left_speed_setpoint,
                              #'RAM_RVEL_SP': right_speed_setpoint,
                              #'RAM_OM': ramsete.omega, 'LFF': left_feed_forward, 'RFF': right_feed_forward,
                              #'LPID': left_output_pid, 'RPID': right_output_pid
                              }
            self.telemetry.append(telemetry_data)


    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.trajectory.totalTime())

    def end(self, interrupted: bool) -> None:
        self.timer.stop()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        # print(self.telemetry)

        if self.write_telemetry:
            location = Path.cwd() if wpilib.RobotBase.isSimulation() else Path('/home/lvuser/py/')  # it's not called robot on the robot
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            file_name = timestamp + '_' + self.getName()  + '.pkl'
            pickle_file = location / 'sim' / 'data' / file_name
            with open(pickle_file.absolute(), 'wb') as fp:
                out_dict = {'TIMESTAMP': timestamp, 'DATA': self.telemetry, 'COURSE': 'NONE',
                            'VELOCITY': 1, 'KP_VEL': 1, 'KD_VEL': 1, 'BETA': 1, 'ZETA': 1}
                pickle.dump(out_dict, fp)
            print(f'*** Wrote telemetry data to {file_name} ***')
        else:
            print(f'*** Skipping saving of telemetry to disk ***')