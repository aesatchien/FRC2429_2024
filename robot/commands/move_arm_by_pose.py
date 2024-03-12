import commands2
import robotpy_apriltag
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Translation2d

import constants

class MoveArmByPose(commands2.CommandBase):  # change the name for your command
    # LHACK 3/11/2024 a command that uses linear interpolation to move the lower crank arm according to our distance to the speaker

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('Move arm by pose')  # change this to something appropriate for this command
        self.container = container
        # Meters and degrees- todo: move to constants?
        self.distance_angle_lookup_table = {
            0: 45,
            1: 40,
            2: 30
        }
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.speaker_translation = Translation2d(constants.k_red_speaker[0], constants.k_red_speaker[1])
        elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.speaker_translation = Translation2d(constants.k_blue_speaker[0], constants.k_blue_speaker[1])

    def execute(self) -> None:
        # Get distance to speaker
        robot_pose = self.container.drive.get_pose()
        robot_translation = Translation2d(robot_pose.X(), robot_pose.Y())
        distance_to_speaker = self.speaker_translation.distance(robot_translation)

        # Find the points to interpolate between
        distances = list(self.distance_angle_lookup_table.keys())
        distances = sorted(distances)

        if len(distances) == 0:
            raise ValueError('No known distances!')
        elif len(distances) == 1:
            greater_distance = lesser_distance = distances[0]
        else:
            greater_distance = distances[-1]  # If we're further than the furthest known distance, keep the slope between the last 2 distances
            lesser_distance = distances[-2]

            for idx, known_distance in enumerate(distances):
                if known_distance > distance_to_speaker:
                    if idx > 0:
                        greater_distance = distances[idx]
                        lesser_distance = distances[idx - 1]
                    else:
                        greater_distance = distances[1]
                        lesser_distance = distances[0]  # If we're closer than the closest known distance, keep the slope between the first 2 distances
                    break

        # We have the 2 points to interpolate between, so interpolate!
        m = ((self.distance_angle_lookup_table[greater_distance] - self.distance_angle_lookup_table[lesser_distance]) /
                                                (greater_distance - lesser_distance))

        interpolated = self.distance_angle_lookup_table[lesser_distance] + m * (distance_to_speaker - lesser_distance)

        self.container.crank_arm.set_goal(interpolated)

        return

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
