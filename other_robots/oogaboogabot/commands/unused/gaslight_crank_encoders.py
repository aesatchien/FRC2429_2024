import math
import commands2
from wpilib import SmartDashboard

import constants
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal

class GaslightCrankEncoders(commands2.CommandBase):  # change the name for your command
    # Gaslights the crank encoder.  Cycles the position to gaslight between the minimum position and our shoot position.
    def __init__(self, container, lower_crank: LowerCrankArmTrapezoidal) -> None:
        super().__init__()
        self.setName('Gaslight crank encoders')  # change this to something appropriate for this command
        self.container = container
        self.lower_crank = lower_crank
        self.positions_to_gaslight_to = [constants.k_crank_arm_dict['min_angle'], constants.k_crank_presets['shoot']['lower']]
        self.counter = 0
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.position_to_gaslight_to = constants.k_crank_arm_dict['min_angle']
        # We gaslight it to be somewhere, then tell it to go there; somehow it ungaslights itself
        self.lower_crank.disable_arm()
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        self.position_to_gaslight_to += 0.2
        print("position to gaslight to: " + str(self.position_to_gaslight_to))
        self.lower_crank.set_encoder_position(math.radians(self.position_to_gaslight_to))

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        self.lower_crank.set_encoder_position(math.radians(self.position_to_gaslight_to))
        self.lower_crank.enable_arm()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s gaslighting to {self.position_to_gaslight_to} **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s gaslighting to {self.position_to_gaslight_to} **")