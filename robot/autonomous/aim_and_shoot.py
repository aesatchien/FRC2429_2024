import commands2
import constants

from commands.arm_smart_go_to import ArmSmartGoTo
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from autonomous.auto_shoot_cycle import AutoShootCycle

class AimAndShoot(commands2.SequentialCommandGroup):
    def __init__(self, container, lower_arm: LowerCrankArmTrapezoidal, upper_arm: UpperCrankArmTrapezoidal) -> None:
        super().__init__()
        self.setName('AimAndShoot')  # change this to something appropriate for this command
        self.container = container
        self.addCommands(ArmSmartGoTo(self.container, upper_arm, lower_arm, 'shoot')) # Go to first position, specifically "shoot"
        self.addCommands(AutoShootCycle(self.container))
