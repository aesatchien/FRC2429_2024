#JS, LC, 3/7/24
#Purpose: a file for path making functions (PathPlanner).
#Problems:
    # - on the sim, the robot seems to move to the right location using the "on_the_fly" bezier curve maker. However, changing the "rotation" parameter doesn't seem to be reflected in the sim.
    # - a bezier curve has an n+1 degree, so it will always be some sort of curved path. Not efficient for straight paths.

import math
import commands2
import wpilib
from wpilib import SmartDashboard
from wpilib import Timer
from pathlib import Path
import pickle
from datetime import datetime
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d
from pathplannerlib.auto import PathPlannerPath, PathPlannerAuto
from pathplannerlib.path import PathConstraints, GoalEndState

import os
import typing
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from subsystems.swerve import Swerve, AutoBuilder
from subsystems.led import Led
from subsystems.swerve_constants import DriveConstants as dc
from subsystems.swerve_constants import AutoConstants as ac
import constants

class PathPlannerConfiguration():

    def __init__(self) -> None:
        pass

    # This is a method that will configure the paths for the robot to follow, based on the .path files in the deploy/pathplanner/paths directory.
    def configure_paths(self, autonomous_chooser:wpilib.SendableChooser):
        allowed_autos = ['1+1 amp', '1+1 middle', '1+1 source', '1+0', "1+1 amp with midline", "1+1 source with midline", "1+2 middle source", "1+2 middle amp"]
        only_use_allowed_autos = True # Change to false for testing, true for comp.  Does not allow paths, only autos, when true.


        if wpilib.RobotBase.isReal():
            path_to_pathplanner_trajectories = '/home/lvuser/py/deploy/pathplanner/paths'
            path_to_pathplanner_autos = '/home/lvuser/py/deploy/pathplanner/autos'
        else:
            path_to_pathplanner_trajectories = os.path.abspath(constants.k_path_from_robot_to_pathplanner_files)
            path_to_pathplanner_autos = os.path.abspath(constants.k_path_from_robot_to_pathplanner_autos)

        file_names = os.listdir(path_to_pathplanner_trajectories) + os.listdir(path_to_pathplanner_autos)

        # No more setting default command here because default command should be drivewait which is not pathplanner auto 3/21/24 LHACK
        for file_name in file_names:
            pure_name = os.path.splitext(file_name)[0]
            extension = os.path.splitext(file_name)[1]
            # We don't want paths, only autos
            if extension == '.auto':
                if only_use_allowed_autos and pure_name in allowed_autos:
                    autonomous_chooser.addOption(pure_name, PathPlannerAuto(pure_name))

                elif not only_use_allowed_autos:
                    print(f'Adding {pure_name} since we\'re adding all autos')

            elif extension == '.path' and not only_use_allowed_autos:
                autonomous_chooser.addOption(pure_name, AutoBuilder.followPath(PathPlannerPath.fromPathFile(pure_name)))

    # This is a method that will create a path from (0,0) to the desired position.
    def configure_path_manual(position_list:typing.Dict[str, float], final_velocity:float, distance_to_rotate:float) -> commands2.Command:
        return AutoBuilder.pathfindToPose(
            Pose2d(position_list['x'], position_list['y'], Rotation2d.fromDegrees(position_list['rotation'])),
            PathConstraints(ac.kMaxSpeedMetersPerSecond, ac.kMaxAccelerationMetersPerSecondSquared, ac.kMaxAngularSpeedRadiansPerSecond, ac.kMaxAngularSpeedRadiansPerSecondSquared),
            final_velocity,
            distance_to_rotate
        )
    
    # This is a method that will be used to create a path on the fly from the "current position" (x,y) of the robot.
    def on_the_fly_path(swerve:Swerve, led:Led, position_list:typing.Dict[str, float], final_velocity:float, linear_speed_factor=1, angular_speed_factor=1, fast_turn=True) -> commands2.Command:
        if position_list["x"] == 0 and position_list["y"] == 0: #bezier curve generation requires the robot to move a non-zero distance.
            return DriveSwerveAutoVelocity(swerve, 0).withTimeout(0.1) #stop
        current_pose = swerve.get_pose()

        position_list = {"x": position_list["x"] - current_pose.X(), "y": position_list["y"] - current_pose.Y(), "rotation": position_list["rotation"] - swerve.get_angle()}
        #create a Transform2d object that contains the position matrix and rotation matrix of the desired position.
        delta_pose = Transform2d(Translation2d(position_list["x"], position_list["y"]), Rotation2d.fromDegrees(0))

        start_pose = Pose2d(current_pose.translation(), current_pose.rotation())
        end_pose = start_pose.transformBy(delta_pose)

        bezier_points = PathPlannerPath.bezierFromPoses([start_pose, end_pose])
        path = PathPlannerPath( #assumes holonomic drivetrain.
            bezier_points,
            PathConstraints(ac.kMaxSpeedMetersPerSecond * linear_speed_factor, ac.kMaxAccelerationMetersPerSecondSquared * linear_speed_factor, ac.kMaxAngularSpeedRadiansPerSecond * angular_speed_factor, ac.kMaxAngularSpeedRadiansPerSecondSquared * angular_speed_factor),
            GoalEndState(final_velocity, Rotation2d.fromDegrees(position_list["rotation"]), rotateFast=fast_turn)
        )
        return AutoBuilder.followPath(path).andThen(led.set_indicator_with_timeout(Led.Indicator.CALIBRATION_SUCCESS, 1))

    def on_the_fly_path_by_pose(self, swerve: Swerve, led: Led, destination:str, final_velocity: float, linear_speed_factor=1, angular_speed_factor=1, fast_turn=True) -> commands2.Command:
        # CJH changed function to take a pose - simpler this way  20240330
        if destination == 'stage':
            target_pose = swerve.get_nearest_tag(destination='stage')  # return a pose2d of where the nearest stage is
        elif destination == 'amp':  # not tested yet
            target_pose = swerve.get_nearest_tag(destination='amp')  # return a pose2d of where the nearest amp is
        elif destination == 'speaker': # not tested yet
            pass
        else:
            target_pose = swerve.get_pose()  # just stay where we are

        x = target_pose.translation().x
        y = target_pose.translation().y
        rot = target_pose.rotation().degrees() - 180

        current_pose = swerve.get_pose()

        if current_pose.translation().distance(target_pose.translation()) < 0.1:  # bezier curve generation requires the robot to move a non-zero distance.
            return DriveSwerveAutoVelocity(swerve, 0).withTimeout(0.1)  # stop

        position_list = {"x": x - current_pose.X(), "y": y - current_pose.Y(), "rotation": rot - swerve.get_angle()}
        # create a Transform2d object that contains the position matrix and rotation matrix of the desired position.
        delta_pose = Transform2d(Translation2d(position_list["x"], position_list["y"]), Rotation2d.fromDegrees(0))

        start_pose = Pose2d(current_pose.translation(), current_pose.rotation())
        end_pose = start_pose.transformBy(delta_pose)

        bezier_points = PathPlannerPath.bezierFromPoses([start_pose, end_pose])
        path = PathPlannerPath( #assumes holonomic drivetrain.
            bezier_points,
            PathConstraints(ac.kMaxSpeedMetersPerSecond * linear_speed_factor, ac.kMaxAccelerationMetersPerSecondSquared * linear_speed_factor, ac.kMaxAngularSpeedRadiansPerSecond * angular_speed_factor, ac.kMaxAngularSpeedRadiansPerSecondSquared * angular_speed_factor),
            GoalEndState(final_velocity, Rotation2d.fromDegrees(position_list["rotation"]), rotateFast=fast_turn)
        )
        return AutoBuilder.followPath(path).andThen(led.set_indicator_with_timeout(Led.Indicator.CALIBRATION_SUCCESS, 1))
