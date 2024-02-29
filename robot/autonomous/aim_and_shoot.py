import commands2
import constants

from commands.arm_move import ArmMove
from commands.arm_cycle import ArmCycle
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from autonomous.auto_shoot_cycle import AutoShootCycle

class AimAndShoot(commands2.SequentialCommandGroup):
    def __init__(self, container, lower_arm: LowerCrankArmTrapezoidal, upper_arm: UpperCrankArmTrapezoidal) -> None:
        super().__init__()
        self.setName('AimAndShoot')  # change this to something appropriate for this command
        self.container = container
        self.addCommands(ArmCycle(container=self.container, upper_crank=upper_arm, lower_crank=lower_arm)) # Go to first position, specifically "shoot"
        self.addCommands(AutoShootCycle(self.container))
