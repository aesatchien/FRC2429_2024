#  copying 1706's default swerve drive control

import math
import typing
import commands2
import wpilib

from subsystems.swerve import Swerve  # allows us to access the definitions
from wpilib import SmartDashboard
from commands2.button import CommandXboxController
from wpimath.geometry import Translation2d
from wpimath.filter import Debouncer
from wpimath.controller import PIDController
import constants
from subsystems.swerve_constants import DriveConstants as dc

class DriveAndAutoAimChassis(commands2.Command):
    def __init__(self, container, swerve: Swerve, field_oriented=True, rate_limited=False,) -> None:
        super().__init__()
        self.setName('drive_and_auto_aim_chassis')
        self.container = container
        self.swerve = swerve
        self.field_oriented = field_oriented  # Sanjith wants this on a button instead
        self.rate_limited = rate_limited
        self.addRequirements(*[self.swerve])
        # can't import container and don't want to pass lambdas just yet
        self.controller: typing.Optional[CommandXboxController] = self.container.driver_command_controller
        self.slow_mode_trigger = self.controller.leftBumper()
        self.robot_oriented_trigger = self.controller.rightBumper()
        self.debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)
        self.robot_oriented_debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)
        self.rotation_pid = PIDController(4, 0, 0) # Values taken from pathplanner's in the swerve subsystem
        self.rotation_pid.enableContinuousInput(-math.pi, math.pi)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

        self.slowmode_history = [1 for i in range(20)]

    def execute(self) -> None:
        slowmode_multiplier = 0.2 + 0.8 * self.controller.getRightTriggerAxis()

        if self.robot_oriented_debouncer.calculate(self.robot_oriented_trigger.getAsBoolean()):
            self.field_oriented = False
        else:
            self.field_oriented = True

        max_linear = 1 * slowmode_multiplier  # stick values  - actual rates are in the constants files
        max_angular = 0.5 # * slowmode_multiplier

        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            translation_origin_to_speaker = Translation2d(constants.k_red_speaker[0], constants.k_red_speaker[1])
        else:
            translation_origin_to_speaker = Translation2d(constants.k_blue_speaker[0], constants.k_blue_speaker[1])

        translation_origin_to_robot = self.swerve.get_pose().translation()
        translation_robot_to_speaker = translation_origin_to_speaker - translation_origin_to_robot
        desired_angle = translation_robot_to_speaker.angle()
        self.rotation_pid.setSetpoint(desired_angle.radians())
        raw_rot = self.rotation_pid.calculate(self.swerve.get_pose().rotation().radians())

        # note that serve's x direction is up/down on the left stick.  (normally think of this as y)
        # according to the templates, these are all multiplied by -1
        # SO IF IT DOES NOT DRIVE CORRECTLY THAT WAY, CHECK KINEMATICS, THEN INVERSION OF DRIVE/ TURNING MOTORS
        # not all swerves are the same - some require inversion of drive and or turn motors
        desired_fwd = -self.input_transform(1.0 * self.controller.getLeftY()) * max_linear
        desired_strafe = -self.input_transform(1.0 * self.controller.getLeftX()) * max_linear
        desired_rot = self.input_transform(1.0 * raw_rot) * max_angular

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
                              fieldRelative= self.field_oriented, rate_limited=True, keep_angle=True)

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
        return a * db_value**3 + b * db_value