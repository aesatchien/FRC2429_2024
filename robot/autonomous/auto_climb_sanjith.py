import math

import commands2
from wpilib import SmartDashboard
import navx
from subsystems.climber import Climber
from subsystems.swerve import Swerve
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.led import Led

class AutoClimbSanjith(commands2.CommandBase):

    def __init__(self, container, climber:Climber, left_volts=2, right_volts=2) -> None:
        super().__init__()
        self.setName('AutoClimbSanjith')
        self.climber = climber
        self.container = container
        self.led: Led = container.led
        self.shooter_arm: UpperCrankArmTrapezoidal = self.container.shooter_arm
        self.crank_arm: LowerCrankArmTrapezoidal = self.container.crank_arm
        self.swerve: Swerve = self.container.drive
        self.navx = self.swerve.navx
        self.left_volts = left_volts
        self.right_volts = right_volts
        self.addRequirements(climber)
        self.count = 0

        # make some milestones
        self.climb_started = False
        self.servos_toggled = False
        self.initial_slack_taken = False
        self.shooter_half_down = False
        self.toggle_servo_fired = False
        self.at_top_of_climb = False

    # TODO: have coast by default but brake when climbing
    def initialize(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)

        if self.at_top_of_climb:
            self.led.set_indicator(Led.Indicator.CALIBRATION_SUCCESS)
        else:
            self.led.set_indicator(Led.Indicator.CLIMB)

        # do NOT reset the climber encoders - want to be able to use this multiple times!

        # if self.navx is None:
        #     self.climber.set_climber(self.left_volts, self.right_volts)

    def execute(self) -> None:
        self.count += 1

        # figure out how to balance and whatnot with the climber motors
        roll = self.swerve.get_pitch()  # it's the navx's pitch axis, and left high is positive
        roll_correction_max_volts = 1  # how many volts to change by to correct for roll - cap the correction at this magnitude
        full_correction_degree_limit = 10  # max degrees before we apply full correction
        roll_correction = roll_correction_max_volts * roll / full_correction_degree_limit  # add or subtract up to a volt based on our tilt
        roll_correction = roll_correction if math.fabs(roll_correction) < roll_correction_max_volts else math.copysign(roll_correction_max_volts, roll_correction)
        left_volts = max(0, self.left_volts - roll_correction)  # don't let them go negative with the correction
        right_volts = max(0, self.right_volts + roll_correction)
        verbose = True if self.count % 50 == 0 else False  # report once in a while
        self.climber.set_climber(left_volts, right_volts, verbose=verbose)

        # lower the shooter box while we go up - each rotation is probably pi*1.375 = 4.3" or so then 25x gearbox
        right_encoder, left_encoder = self.climber.get_encoders()
        encoder_average = (right_encoder + left_encoder) / 2

        # would be nice if we could only do this once instead of spamming the goal setter
        if encoder_average > 145 and not self.at_top_of_climb:  # 24.2 inches of rope
            self.led.set_indicator(Led.Indicator.CALIBRATION_SUCCESS)  # flash green
            self.at_top_of_climb = True
        elif encoder_average > 125 and not self.toggle_servo_fired:  # 24.2 inches of rope
            self.shooter_arm.set_goal(math.radians(-90))
            self.climber.open_trap_servo()
            print(f"Moved Shooter to -90 and fired trap servo at {self.container.get_enabled_time()}s")
            self.toggle_servo_fired = True
        elif encoder_average > 110 and not self.shooter_half_down:  # 19 inches
            self.shooter_arm.set_goal(math.radians(-45))
            self.shooter_half_down = True
            print(f"Moved shooter to -45 at {self.container.get_enabled_time()}s")
        elif encoder_average > 100 and not self.servos_toggled:
            self.servos_toggled = True
            self.climber.close_servos()
            print(f"Closing servos at {self.container.get_enabled_time()}s")
        elif encoder_average > 84 and not self.initial_slack_taken:  # just getting taut - 14.5"
            print(f"Initial slack taken up {self.container.get_enabled_time()}s")
            self.crank_arm.set_goal(math.radians(109))
            self.shooter_arm.set_goal(math.radians(0))
            self.initial_slack_taken = True
        elif encoder_average > 10 and not self.climb_started:  # just getting taut - 14.5"
            print(f"Starting climb at {self.container.get_enabled_time()}s")
            self.shooter_arm.set_goal(math.radians(22))
            self.climb_started = True

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        if self.at_top_of_climb:
            self.led.set_indicator_with_timeout(Led.Indicator.CALIBRATION_SUCCESS, 0.5).schedule()
        else:
            self.led.set_indicator(Led.Indicator.NONE)
        self.climber.stop_climber()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")




