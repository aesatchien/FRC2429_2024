import math

import commands2
from wpilib import SmartDashboard
import navx
from subsystems.climber import Climber
from subsystems.swerve import Swerve
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from subsystems.led import Led

class AutoRunClimber(commands2.CommandBase):

    def __init__(self, container, climber:Climber, left_volts=2, right_volts=2) -> None:
        super().__init__()
        self.setName('Run climber')
        self.climber = climber
        self.container = container
        self.led: Led = container.led
        self.shooter_arm: UpperCrankArmTrapezoidal = self.container.shooter_arm
        self.swerve: Swerve = self.container.drive
        self.navx = self.swerve.navx
        self.left_volts = left_volts
        self.right_volts = right_volts
        self.addRequirements(climber)
        self.count = 0

    # TODO: have coast by default but brake when climbing
    def initialize(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)

        self.led.set_indicator(Led.Indicator.CLIMB)

        # do NOT reset the climber encoders - want to be able to use this multiple times!

        # if self.navx is None:
        #     self.climber.set_climber(self.left_volts, self.right_volts)

    def execute(self) -> None:
        self.count += 1

        # figure out how to balance and whatnot with the climber motors
        roll = self.swerve.get_roll()
        roll_correction_max_volts = 1  # how many volts to change by to correct for roll - cap the correction at this magnitude
        full_correction_degree_limit = 15  # max degrees before we apply full correction
        roll_correction = roll_correction_max_volts * roll / full_correction_degree_limit  # add or subtract up to a volt based on our tilt
        roll_correction = roll_correction if math.fabs(roll_correction) < roll_correction_max_volts else math.copysign(roll_correction_max_volts, roll_correction)
        left_volts = max(0, self.left_volts + roll_correction)  # don't let them go negative with the correction
        right_volts = max(0, self.right_volts - roll_correction)
        self.climber.set_climber(left_volts, right_volts)

        # lower the shooter box while we go up - each rotation is probably pi*1.375 = 4.3" or so
        right_encoder, left_encoder = self.climber.get_encoders()
        encoder_average_inches = 4.3 * (right_encoder + left_encoder) / 2  # 4.3" per rotation
        if encoder_average_inches > 15:
            self.shooter_arm.set_goal(math.radians(-90))
        elif encoder_average_inches > 12:
            self.shooter_arm.set_goal(math.radians(-70))
        elif encoder_average_inches > 9:
            self.shooter_arm.set_goal(math.radians(-50))
        elif encoder_average_inches > 6:
            self.shooter_arm.set_goal(math.radians(-30))
        elif encoder_average_inches > 3:
            self.shooter_arm.set_goal(math.radians(-10))

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.led.set_indicator(Led.Indicator.NONE)
        self.climber.stop_climber()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")




