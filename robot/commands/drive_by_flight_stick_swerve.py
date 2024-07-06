#  copying 1706's default swerve drive control

import math
import typing
import commands2
import wpilib

from subsystems.swerve import Swerve  # allows us to access the definitions
from wpilib import SmartDashboard
from commands2.button import CommandJoystick
from wpimath.geometry import Translation2d
from wpimath.filter import Debouncer
import constants
from subsystems.swerve_constants import DriveConstants as dc

class DriveByFlightStickSwerve(commands2.Command):
    def __init__(self, container, flight_stick: CommandJoystick, swerve: Swerve, field_oriented=True, rate_limited=False) -> None:
        # LHACK 5/27/24 A command to drive a swerve with the Thrustmaster T.Flight HOTAS X joystick
        # TODO: let the dpad hat thingy snap to headings
        super().__init__()
        self.setName('drive_by_joystick_swerve')
        self.container = container
        self.swerve = swerve
        self.field_oriented = field_oriented  # Sanjith wants this on a button instead
        self.rate_limited = rate_limited
        self.addRequirements(*[self.swerve])
        self.flight_stick = flight_stick
        # self.slow_mode_trigger = self.controller.rightBumper()
        # self.robot_oriented_debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

    def execute(self) -> None:

        # Weird math is because this stick's throttle goes from 1 to -1 as it's pushed forwards
        slowmode_multiplier = 0.1 + 0.9 * (1 - self.flight_stick.getThrottle())/2
        angular_slowmode_multiplier = 0.1 + 0.9 * (1 - self.flight_stick.getThrottle())/2

        max_linear = 1 * slowmode_multiplier  # stick values  - actual rates are in the constants files
        max_angular = 1 * angular_slowmode_multiplier

        # note that swerve's x direction is up/down on the left stick.  (normally think of this as y)
        # according to the templates, these are all multiplied by -1
        # SO IF IT DOES NOT DRIVE CORRECTLY THAT WAY, CHECK KINEMATICS, THEN INVERSION OF DRIVE/ TURNING MOTORS
        # not all swerves are the same - some require inversion of drive and or turn motors

        # TODO: experiment with which transforms feel nicest at the shop


        equispeed_squares = False
        circular_cap = True

        joystick_translation = Translation2d(self.flight_stick.getX(), -self.flight_stick.getY())

        if equispeed_squares:
            # If we plug X and Y into input_transform directly, the commanded speed is constant along any circle centered on the
            # joystick's origin.  Since the range of motion of the joystick is square, not circular, we want the commanded speed
            # to be constant along any square centered on the origin.
            # This code divides the joystick's translation vector by the length of the line from the origin to the square
            # which bounds the joystick's range of motion

            # success!!!

            theta = abs(joystick_translation.angle().radians())
            while theta > math.pi/2:
                theta -= math.pi/2
            if theta > math.pi/4:
                theta = math.pi/2 - theta

            length_from_origin_to_square_side = 1/math.cos(theta)
            transformed_joystick_translation = joystick_translation/length_from_origin_to_square_side

        elif circular_cap:
            # Capping the magnitude of the joystick translation ensures the max diagonal isn't faster than the max along cardinal directions
            transformed_joystick_translation = joystick_translation / joystick_translation.norm() if joystick_translation.norm() > 1 else joystick_translation

        else:
            transformed_joystick_translation = joystick_translation


        desired_fwd = self.input_transform_linear(1.0 * transformed_joystick_translation.Y()) * max_linear
        desired_strafe = -self.input_transform_linear(1.0 * transformed_joystick_translation.X()) * max_linear
        desired_rot = -self.input_transform(1 * self.flight_stick.getTwist()) * max_angular

        if wpilib.RobotBase.isSimulation():
            SmartDashboard.putNumberArray('joystick', [desired_fwd, desired_strafe, desired_rot])

        correct_like_1706 = False  # this is what 1706 does, but Rev put all that in the swerve module's drive
        if correct_like_1706:
            desired_translation = Translation2d(desired_fwd, desired_strafe)
            desired_magnitude = desired_translation.norm()
            if desired_magnitude > max_linear:
                desired_translation = desired_translation * max_linear / desired_magnitude
            self.swerve.drive(desired_translation.X(), desired_translation.Y(), desired_rot,
                          fieldRelative= self.field_oriented, rate_limited=self.rate_limited)
        else:
            self.swerve.drive(xSpeed=desired_fwd,ySpeed=desired_strafe, rot=desired_rot,
                              fieldRelative= self.field_oriented, rate_limited=self.rate_limited, keep_angle=True)

    def end(self, interrupted: bool) -> None:
        # probably should leave the wheels where they are?
        self.swerve.drive(0, 0, 0, fieldRelative=self.field_oriented, rate_limited=True)
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def apply_deadband(self, value, db_low=dc.k_inner_deadband, db_high=dc.k_outer_deadband):
        # put a deadband on the joystick input values here
        if abs(value) < db_low:
            return 0
        elif abs(value) > db_high:
            return 1 * math.copysign(1, value)
        else:
            return value

    def input_transform(self, value, a=0.9, b=0.1):
        # 1706 uses a nice transform
        db_value = self.apply_deadband(value)
        return a * math.copysign(db_value**2, value) + b * db_value

    def input_transform_linear(self, value, a=0.9, b=0.1):
        # 1706 uses a nice transform
        db_value = self.apply_deadband(value)
        return db_value