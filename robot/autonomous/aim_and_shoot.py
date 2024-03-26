import commands2
import constants


from autonomous.auto_shoot_cycle import AutoShootCycle
from commands.arm_cycle import ArmCycle

from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from autonomous.auto_commands import GoToShoot
class AimAndShoot(commands2.SequentialCommandGroup):
    def __init__(self, container, lower_arm: LowerCrankArmTrapezoidal, upper_arm: UpperCrankArmTrapezoidal) -> None:
        super().__init__()
        self.setName('AimAndShoot')  # change this to something appropriate for this command
        self.container = container
        self.addCommands(ArmSmartGoTo(self.container, desired_position='low_shoot', wait_for_finish=True))# Go to first position, specifically "shoot"
        self.addCommands(commands2.WaitCommand(3))
        self.addCommands(AutoShootCycle(self.container))
