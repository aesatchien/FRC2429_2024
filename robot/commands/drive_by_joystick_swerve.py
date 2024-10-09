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
import constants
from subsystems.swerve_constants import DriveConstants as dc

class DriveByJoystickSwerve(commands2.Command):
    def __init__(self, container, swerve: Swerve, field_oriented=True, rate_limited=False,) -> None:
        # TODO: sanjith would like his joystick to directly adjust heading so if he puts his stick at 45 deg the robot will go to 45 deg
        super().__init__()
        self.setName('drive_by_joystick_swerve')
        self.container = container
        self.swerve = swerve
        self.field_oriented = field_oriented  # Sanjith wants this on a button instead
        self.rate_limited = rate_limited
        self.addRequirements(*[self.swerve])
        # can't import container and don't want to pass lambdas just yet
        self.controller: typing.Optional[CommandXboxController] = self.container.driver_command_controller
        # self.slow_mode_trigger = self.controller.rightBumper()
        self.robot_oriented_trigger = self.controller.leftBumper()
        self.debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)
        self.robot_oriented_debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

        self.slowmode_history = [1 for i in range(20)]

    def execute(self) -> None:
        # setting a slow mode here - not sure if it's the best way
        # if self.debouncer.calculate(self.slow_mode_trigger.getAsBoolean()):  # holding down button 5 - LB
        #     self.slowmode_history.append(constants.k_slowmode_multiplier)
        #     slowmode_multiplier = constants.k_slowmode_multiplier
        # elif self.controller.getLeftTriggerAxis() > 0.5:  # squeezing the left axisbutton
        #     self.slowmode_history.append(constants.k_slowmode_multiplier * 1.5)
        #     slowmode_multiplier = constants.k_slowmode_multiplier * 1.5
        # else:
        #     self.slowmode_history.append(1)
        #     slowmode_multiplier = sum(self.slowmode_history) / len(self.slowmode_history)
        # self.slowmode_history.pop(0)

        slowmode_multiplier = 0.2 + 0.8 * self.controller.getRightTriggerAxis()
        angular_slowmode_multiplier = 0.5 + 0.5 * self.controller.getRightTriggerAxis()

        if self.robot_oriented_debouncer.calculate(self.robot_oriented_trigger.getAsBoolean()):
            self.field_oriented = False
        else:
            self.field_oriented = True

        max_linear = 1 * slowmode_multiplier  # stick values  - actual rates are in the constants files
        max_angular = 1 * angular_slowmode_multiplier

        # note that swerve's x direction is up/down on the left stick.  (normally think of this as y)
        # according to the templates, these are all multiplied by -1
        # SO IF IT DOES NOT DRIVE CORRECTLY THAT WAY, CHECK KINEMATICS, THEN INVERSION OF DRIVE/ TURNING MOTORS
        # not all swerves are the same - some require inversion of drive and or turn motors

        joystick_fwd = self.controller.getLeftY()
        joystick_strafe = self.controller.getLeftX()

        linear_mapping = True  # two ways to make sure diagonal is at "full speed"
        if linear_mapping:
            # this lets you go full speed diagonal at the cost of sensitivity on the low end
            desired_fwd = -self.input_transform_linear(1.0 * joystick_fwd) * max_linear
            desired_strafe = -self.input_transform_linear(1.0 * joystick_strafe) * max_linear
            desired_rot = -self.input_transform_linear(1.0 * self.controller.getRightX()) * max_angular
        else:
            # correcting for the x^2 + y^2 = 1 of the joysticks so we don't have to use a linear mapping
            angle = math.atan2(joystick_fwd, joystick_strafe)
            correction = math.fabs(math.cos(angle) * (math.sin(angle)))  # peaks at 45 degrees
            fwd = joystick_fwd * (1 + 0.3 * math.copysign(correction, joystick_fwd))  # they max out at .77, so add the remainder at 45 degrees to make 1
            strafe = joystick_strafe * (1 + 0.3 * math.copysign(correction, joystick_strafe))
            if wpilib.RobotBase.isSimulation():
                wpilib.SmartDashboard.putNumberArray('joysticks',[joystick_fwd, joystick_strafe, correction, fwd, strafe])

            desired_fwd = -self.input_transform(1.0 * fwd) * max_linear
            desired_strafe = -self.input_transform(1.0 * strafe) * max_linear
            desired_rot = -self.input_transform(1.0 * self.controller.getRightX()) * max_angular

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
        return a * db_value**3 + b * db_value

    def input_transform_linear(self, value, a=0.9, b=0.1):
        # 1706 uses a nice transform
        db_value = self.apply_deadband(value)
        return db_value