import commands2
import math
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Translation2d

import constants

class MoveArmByPose(commands2.CommandBase):
    # LHACK 3/11/2024 a command that uses linear interpolation to move the lower crank arm according to our distance to the speaker

    def __init__(self, container, shooting_backwards=True) -> None:
        super().__init__()
        self.setName('Move arm by pose')  # change this to something appropriate for this command
        self.container = container
        self.shooting_backwards = shooting_backwards
        # Meters and degrees. todo: tune
        # 50" 52 deg
        # 56" 51 deg
        # 61" 48 deg
        # 87" 41 deg
        # 77" 45 deg
        # 59" 54 deg

        #add two interpolation data point lookup tables, one for shooting backwards in low position, and one for shooting forwards in old shooter position

        self.close_range_distance_angle_lookup_table = {
            1.27: 54,
            1.42: 54,
            1.50: 53,
            1.55: 48,
            1.96: 45,
            2.21: 41
        } #put lower crank arm angle values in this table ; upper crank arm is at -84 degrees and fixated when lower shooting mode
        
        self.far_range_distance_angle_offset_lookup_table = {
            #1: 4,
            #2: 7,
            #3: 7,
            1.4: 5,
            3.92: 7
        } #put upper crank arm (shooter arm) angle offset values in this table ; lower crank arm is 90 degrees, and upper crank arm is rotating in forward high shooting mode

        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""

        self.container.set_arm_configuration('auto aim')
        SmartDashboard.putString('arm_config', self.container.get_arm_configuration())

        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.speaker_translation = Translation2d(constants.k_speaker_tags_poses["red"][0], constants.k_speaker_tags_poses["red"][1])
        elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.speaker_translation = Translation2d(constants.k_speaker_tags_poses["blue"][0], constants.k_speaker_tags_poses["blue"][1])


    def execute(self) -> None:
        # Get distance to speaker
        robot_pose = self.container.drive.get_pose()
        x = robot_pose.X()
        y = robot_pose.Y()
        self.distance_to_speaker = math.sqrt(math.pow((x - self.speaker_translation[0]), 2) + math.pow((y - self.speaker_translation[1]), 2))

        #robot_pose = self.container.drive.get_pose()
        #robot_translation = Translation2d(robot_pose.X(), robot_pose.Y())
        #distance_to_speaker = self.speaker_translation.distance(robot_translation)


        #if shooting forwards use forward shooting
        if self.distance_to_speaker <= 2.21 and self.shooting_backwards:
            # Find the points to interpolate between
            distances = list(self.close_range_distance_angle_lookup_table.keys())
            distances = sorted(distances)

            if len(distances) == 0:
                raise ValueError('No known distances!')
            elif len(distances) == 1:
                greater_distance = lesser_distance = distances[0]
            else:
                greater_distance = distances[-1]  # If we're further than the furthest known distance, keep the slope between the last 2 distances
                lesser_distance = distances[-2]

                for idx, known_distance in enumerate(distances):
                    if known_distance > self.distance_to_speaker:
                        if idx > 0:
                            greater_distance = distances[idx]
                            lesser_distance = distances[idx - 1]
                        else:
                            greater_distance = distances[1]
                            lesser_distance = distances[0]  # If we're closer than the closest known distance, keep the slope between the first 2 distances
                        break

            #find slope and interpolate
            m = ((self.close_range_distance_angle_lookup_table[greater_distance] - self.close_range_distance_angle_lookup_table[lesser_distance]) /
                                                    (greater_distance - lesser_distance))

            interpolated = self.close_range_distance_angle_lookup_table[lesser_distance] + m * (self.distance_to_speaker - lesser_distance)

            #self.container.shooter_arm.set_goal( wherever the default old position was for upper crank)

            self.container.crank_arm.set_goal(math.radians(interpolated))

            self.container.shooter_arm.set_goal(math.radians(-84))

        
        #else if shooting backwards
        elif not self.shooting_backwards:
           # Find the points to interpolate between
            distances = list(self.far_range_distance_angle_offset_lookup_table.keys())
            distances = sorted(distances)

            if len(distances) == 0:
                raise ValueError('No known distances; table empty.')
            elif len(distances) == 1:
                greater_distance = lesser_distance = distances[0]
            else:
                greater_distance = distances[-1]  # If we're further than the furthest known distance, keep the slope between the last 2 distances
                lesser_distance = distances[-2]

                for i, known_distance in enumerate(distances):
                    if known_distance > self.distance_to_speaker:
                        if i > 0:
                            greater_distance = distances[i]
                            lesser_distance = distances[i - 1]
                        else:
                            greater_distance = distances[1]
                            lesser_distance = distances[0]  # If we're closer than the closest known distance, keep the slope between the first 2 distances
                        break

            # find slope and interpolate
            m = ((self.far_range_distance_angle_offset_lookup_table[greater_distance] - self.far_range_distance_angle_offset_lookup_table[lesser_distance]) / (greater_distance - lesser_distance))

            interpolated_offset = self.far_range_distance_angle_offset_lookup_table[lesser_distance] + m * (self.distance_to_speaker - lesser_distance)

            #IF TESTING, change this to experiment
            interpolated_offset = interpolated_offset

            self.container.crank_arm.set_goal(math.pi / 2)

            used_height = constants.k_speaker_opening_height - (constants.k_bumper_height + constants.k_crank_arm_dict['arm_length'])

            if self.container.crank_arm.angle > abs(constants.k_max_upper_crank_where_retracting_lower_crank_safe_rad):

                self.container.shooter_arm.set_goal(math.radians(math.degrees(-math.atan(used_height / self.distance_to_speaker))+interpolated_offset))

        return

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
