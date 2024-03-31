import commands2
import math
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Translation2d

import constants

class MoveArmByPose(commands2.CommandBase):  # change the name for your command
    # LHACK 3/11/2024 a command that uses linear interpolation to move the lower crank arm according to our distance to the speaker

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('Move arm by pose')  # change this to something appropriate for this command
        self.container = container
        # Meters and degrees. todo: tune
        # 50" 52 deg
        # 56" 51 deg
        # 61" 48 deg
        # 87" 41 deg
        # 77" 45 deg
        # 59" 54 deg
        self.distance_angle_lookup_table = {
            1.27: 54,
            1.42: 54,
            1.50: 53,
            1.55: 48,
            1.96: 45,
            2.21: 41
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

        #robot_translation = Translation2d(robot_pose.X(), robot_pose.Y())
        #distance_to_speaker = self.speaker_translation.distance(robot_translation)
        
        x = robot_pose.X()
        y = robot_pose.Y()
        self.distance_to_speaker = math.sqrt(math.pow((x - self.speaker_translation[0]), 2) + math.pow((y - self.speaker_translation[1]), 2))

        

        ''' interpolation isn't necessary, I will try not using it for now - Arshan

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

        '''

        angle_difference = 4 #difference value to fix errors (initially shots were too high, so we can eperiment with different values for this variable and see how it affects accuracy)

        self.container.crank_arm.set_goal(math.pi / 2)
        if self.container.crank_arm.angle > abs(constants.k_max_upper_crank_where_retracting_lower_crank_safe_rad):
            self.container.shooter_arm.set_goal(math.radians(math.degrees(-1 * math.atan((constants.k_speaker_opening_height - 0.8382) / self.distance_to_speaker))+angle_difference))

        return

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
