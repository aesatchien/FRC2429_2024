import commands2
import constants

<<<<<<< HEAD
from autonomous.auto_shoot_cycle import AutoShootCycle
from commands.arm_cycle import ArmCycle
=======
from commands.arm_smart_go_to import ArmSmartGoTo
>>>>>>> a5c416d0e2c7be0d010fd296fb8698b1ea7f0a97
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from commands.arm_smart_go_to import ArmSmartGoTo
class AimAndShoot(commands2.SequentialCommandGroup):
    def __init__(self, container, lower_arm: LowerCrankArmTrapezoidal, upper_arm: UpperCrankArmTrapezoidal) -> None:
        super().__init__()
        self.setName('AimAndShoot')  # change this to something appropriate for this command
        self.container = container
<<<<<<< HEAD
        self.addCommands(ArmSmartGoTo(self.container, upper_crank=upper_arm, lower_crank=lower_arm, desired_position='shoot')) # Go to first position, specifically "shoot"
=======
        self.addCommands(ArmSmartGoTo(self.container, upper_arm, lower_arm, 'shoot')) # Go to first position, specifically "shoot"
>>>>>>> a5c416d0e2c7be0d010fd296fb8698b1ea7f0a97
        self.addCommands(AutoShootCycle(self.container))
