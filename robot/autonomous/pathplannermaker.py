import math
import commands2
import wpilib
from wpilib import SmartDashboard
from wpilib import Timer
from pathlib import Path
import pickle
from datetime import datetime
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d
from pathplannerlib.auto import AutoBuilder, PathPlannerPath
from pathplannerlib.path import PathConstraints, GoalEndState

import os
import typing
from subsystems.swerve import Swerve
from subsystems.swerve_constants import DriveConstants as dc
from subsystems.swerve_constants import AutoConstants as ac
import constants
from subsystems.swerve_constants import AutoConstants as ac

class PathPlannerConfiguration():

    def __init__(self) -> None:
        pass

    # This is a method that will configure the paths for the robot to follow, based on the .path files in the deploy/pathplanner/paths directory.
    def configure_paths(self, autonomous_chooser:wpilib.SendableChooser):
        if wpilib.RobotBase.isReal():
            path_to_pathplanner_trajectories = '/home/lvuser/py/deploy/pathplanner/paths'
        else:
            path_to_pathplanner_trajectories = os.path.abspath(constants.k_path_from_robot_to_pathplanner_files)

        file_names = os.listdir(path_to_pathplanner_trajectories)
        file_names = [file_name for file_name in file_names if '.path' in file_name]  # in case non-path files exist
        for ix, file_name in enumerate(file_names):
            file_name = os.path.splitext(file_name)[0] # Get the name of the trajectory, not the .path extension
            if ix == 0:
                print("FILE NAME", file_name)
                autonomous_chooser.setDefaultOption(file_name, AutoBuilder.followPath(PathPlannerPath.fromPathFile(file_name)).withTimeout(10))
            else:
                print("FILE NAME", file_name)
                autonomous_chooser.addOption(file_name, AutoBuilder.followPath(PathPlannerPath.fromPathFile(file_name)).withTimeout(10))


    # This is a method that will create a path from (0,0) to the desired position.
    def configure_path_manual(position_list:typing.Dict[str, float], final_velocity:float, distance_to_rotate:float) -> commands2.Command:
        return AutoBuilder.pathfindToPose(
            Pose2d(position_list['x'], position_list['y'], Rotation2d.fromDegrees(position_list['rotation'])),
            PathConstraints(ac.kMaxSpeedMetersPerSecond, ac.kMaxAccelerationMetersPerSecondSquared, ac.kMaxAngularSpeedRadiansPerSecond, ac.kMaxAngularSpeedRadiansPerSecondSquared),
            final_velocity,
            distance_to_rotate
        )
    
    # This is a method that will be used to create a path on the fly from the "current position" (x,y) of the robot.
    def on_the_fly_path(robot:Swerve, position_list:typing.Dict[str, float], final_velocity:float) -> commands2.Command:
        current_pose = robot.get_pose()
        #create a Transform2d object that contains the position matrix and rotation matrix of the desired position.
        delta_pose = Transform2d(Translation2d(position_list["x"], position_list["y"]), Rotation2d.fromDegrees(position_list["rotation"]))

        start_pose = Pose2d(current_pose.translation(), current_pose.rotation())
        end_pose = start_pose.transformBy(delta_pose)

        bezier_points = PathPlannerPath.bezierFromPoses([start_pose, end_pose])
        path = PathPlannerPath(
            bezier_points,
            PathConstraints(ac.kMaxSpeedMetersPerSecond, ac.kMaxAccelerationMetersPerSecondSquared, ac.kMaxAngularSpeedRadiansPerSecond, ac.kMaxAngularSpeedRadiansPerSecondSquared),
            GoalEndState(final_velocity, Rotation2d.fromDegrees(position_list["rotation"]))
        )
        return AutoBuilder.followPath(path)
    
    #Not quite sure how to get the sendable chooser to update.
    # def on_the_fly_path(robot:Swerve, position_chooser:wpilib.SendableChooser, final_velocity:float) -> commands2.Command:
    #     desired_pos = position_chooser.getSelected()
        
    #     current_pose = robot.get_pose()
    #     #create a Transform2d object that contains the position matrix and rotation matrix of the desired position.
    #     delta_pose = Transform2d(Translation2d(desired_pos["x"], desired_pos["y"]), Rotation2d.fromDegrees(desired_pos["rotation"]))

    #     start_pose = Pose2d(current_pose.translation(), current_pose.rotation())
    #     end_pose = start_pose.transformBy(delta_pose)

    #     bezier_points = PathPlannerPath.bezierFromPoses([start_pose, end_pose])
    #     path = PathPlannerPath(
    #         bezier_points,
    #         PathConstraints(ac.kMaxSpeedMetersPerSecond, ac.kMaxAccelerationMetersPerSecondSquared, ac.kMaxAngularSpeedRadiansPerSecond, ac.kMaxAngularSpeedRadiansPerSecondSquared),
    #         GoalEndState(final_velocity, Rotation2d.fromDegrees(desired_pos["rotation"]))
    #     )
    #     return AutoBuilder.followPath(path)