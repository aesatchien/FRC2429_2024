import commands2
import math
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Translation2d

import constants

class SetFlywheelLobByPose(commands2.Command):
    # LHACK 3/11/2024 a command that uses linear interpolation to move the lower crank arm according to our distance to the corner

    def __init__(self, container, stop_flywheels_on_end=True) -> None:
        super().__init__()
        self.setName('Move arm by pose')  # change this to something appropriate for this command
        self.container = container
        self.stop_flywheels_on_end=stop_flywheels_on_end
        #add two interpolation data point lookup tables, one for shooting backwards in low position, and one for shooting forwards in old shooter position

        self.distance_velocity_lookup_table = {
            100: 0
        } #put lower crank arm angle values in this table ; upper crank arm is at -84 degrees and fixated when lower shooting mode

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
            self.corner_translation = Translation2d(constants.k_red_amp[0], constants.k_red_amp[1])
        elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self.corner_translation = Translation2d(constants.k_blue_amp[0], constants.k_blue_amp[1])


    def execute(self) -> None:
        # Get distance to corner
        robot_pose = self.container.drive.get_pose()
        x = robot_pose.X()
        y = robot_pose.Y()
        self.distance_to_corner = math.sqrt(math.pow((x - self.corner_translation[0]), 2) + math.pow((y - self.corner_translation[1]), 2))


        #if shooting backwards use backwards low shooting, and vice versa if shooting forwards:

        #interpolation
        distances = list(self.distance_velocity_lookup_table.keys())
        distances = sorted(distances)

        if len(distances) == 0:
            raise ValueError('No known distances!')
        elif len(distances) == 1:
            greater_distance = lesser_distance = distances[0]
        else:
            greater_distance = distances[-1]  # If we're further than the furthest known distance, keep the slope between the last 2 distances
            lesser_distance = distances[-2]

            for idx, known_distance in enumerate(distances):
                if known_distance > self.distance_to_corner:
                    if idx > 0:
                        greater_distance = distances[idx]
                        lesser_distance = distances[idx - 1]
                    else:
                        greater_distance = distances[1]
                        lesser_distance = distances[0]  # If we're closer than the closest known distance, keep the slope between the first 2 distances
                    break

            #find slope and interpolate
        m = ((self.distance_velocity_lookup_table[greater_distance] - self.distance_velocity_lookup_table[lesser_distance]) /
                                                (greater_distance - lesser_distance))
        
        interpolated = self.distance_velocity_lookup_table[lesser_distance] + m * (self.distance_to_corner - lesser_distance)

        self.container.shooter.set_flywheel(interpolated)

        return

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        if self.stop_flywheels_on_end:
            self.container.shooter.stop_shooter()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
