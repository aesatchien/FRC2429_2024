# 2024 0331 CJH - AutoDriveToTag
# this is a command for driving to a location using odometry
# JS did a lot of the legwork, CJH made it run-time configurable
# basically we drive to apriltags but with custom offsets based on the type of tag (stage, amp, etc)
# there is a bit of a kludge here because we build a command in the init and then fire it off, so we have to track
# it in the scheduler

import commands2
from wpilib import SmartDashboard

from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from subsystems.swerve import Swerve, AutoBuilder
from subsystems.led import Led
from autonomous.pathplannermaker import PathPlannerConfiguration
from pathplannerlib.auto import PathPlannerPath, PathPlannerAuto
from pathplannerlib.path import PathConstraints, GoalEndState
from subsystems.swerve_constants import AutoConstants as ac
import wpimath.geometry as geo


class AutoDriveToTag(commands2.Command):  # change the name for your command

    def __init__(self, container, drive, destination='stage') -> None:
        super().__init__()
        self.setName('AutoDriveToTag')  # change this to something appropriate for this command
        self.container = container
        self.drive: Swerve = drive
        self.led: Led = self.container.led
        self.destination = destination
        # self.addRequirements(self.container.drive)  # if you add this the holonomic path follower will end this command
        self.cmd = None  # we will build a path-following command in initialize
        self.scheduler = None  # we will grab the scheduler in init
        self.counter = 0  # use this for some periodic debugging messages

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")
        # copying template from Josh
        # figure out where we are going - swerve currently is managing this since it keeps the odometry... could live anywhere
        target_pose = self.drive.get_nearest_tag(destination=self.destination)
        x = target_pose.translation().x
        y = target_pose.translation().y
        rot = target_pose.rotation().degrees()

        current_pose = self.drive.get_pose()
        start_pose = current_pose
        end_pose = target_pose
        print(f'  Driving from {current_pose} to {target_pose}\n   via {current_pose-target_pose}')

        # see if it's worth moving - prevent the bezier from failing on us
        if current_pose.translation().distance(target_pose.translation()) < 0.1:  # bezier curve generation requires the robot to move a non-zero distance.
            print("  No point in driving there, already here!")
            self.cmd = DriveSwerveAutoVelocity(container=self.container, drive=self.drive, velocity=0).withTimeout(0.1)
                        # .andThen(self.container.led.set_indicator_with_timeout(Led.Indicator.AMP, 0.75)))  # stop, flash red

        else:  # looks good to move
            linear_speed_factor = 0.6
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
                GoalEndState(velocity=final_velocity, rotation=target_pose.rotation(), rotateFast=fast_turn)
            )
            path.preventFlipping = True
            self.cmd = AutoBuilder.followPath(path)  # .andThen(commands2.InstantCommand(self.drive.reset_keep_angle))
            #.andThen(self.container.led.set_indicator_with_timeout(Led.Indicator.CALIBRATION_SUCCESS, 0.75)))

        self.drive.set_use_apriltags(True)  # dont update when moving
        self.cmd.schedule()  # this kills the current command if we required the drive, so don't!
        print(f'  scheduling {self.cmd.getName()} ...')

    def execute(self) -> None:
        pass
        # self.counter += 1
        # # monitor our command
        # if self.counter % 100 == 0:
        #     print(f' {self.cmd.getName()} is scheduled: {self.scheduler.isScheduled(self.cmd)}')

    def isFinished(self) -> bool:
        # finish when the command we scheduled ends! - interrupt if we let go of the button
        return not self.scheduler.isScheduled(self.cmd)

    def end(self, interrupted: bool) -> None:

        if interrupted:
            # cancel the command we launched in the init phase if we let go of the button early, show failure
           self.scheduler.cancel(self.cmd)
           print(f'  CANCELING {self.cmd.getName()}', flush=True)
           self.led.set_indicator_with_timeout(Led.Indicator.AMP, 0.75).schedule()  # RED

        else:
            # show success
            self.container.led.set_indicator_with_timeout(Led.Indicator.CALIBRATION_SUCCESS, 0.75).schedule()  # GREEN

        self.drive.reset_keep_angle()  # either way, reset the keep angle on the drive
        self.drive.set_use_apriltags(True)  # update again

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")