import commands2
import constants


from autonomous.auto_shoot_cycle import AutoShootCycle
from commands.arm_cycle import ArmCycle

from commands.arm_smart_go_to import ArmSmartGoTo
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from commands.arm_smart_go_to import ArmSmartGoTo
class AimAndShoot(commands2.SequentialCommandGroup):
    def __init__(self, container, lower_arm: LowerCrankArmTrapezoidal, upper_arm: UpperCrankArmTrapezoidal) -> None:
        super().__init__()
        self.setName('AimAndShoot')  # change this to something appropriate for this command
        self.container = container
        self.addCommands(ArmSmartGoTo(self.container, upper_crank=upper_arm, lower_crank=lower_arm, desired_position='shoot', wait_for_finish=True))# Go to first position, specifically "shoot"
        self.addCommands(commands2.WaitCommand(3))
        self.addCommands(AutoShootCycle(self.container))
