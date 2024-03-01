import commands2
import constants

from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from subsystems.swerve import Swerve

from autonomous.aim_and_shoot import AimAndShoot
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity

class ShootAndDriveOut(commands2.SequentialCommandGroup):
    def __init__(self, container, lower_arm: LowerCrankArmTrapezoidal, upper_arm: UpperCrankArmTrapezoidal, drive: Swerve) -> None:
        super().__init__()
        self.setName('AimAndShoot')  # change this to something appropriate for this command
        self.container = container
        self.addCommands(AimAndShoot(self.container, lower_arm=lower_arm, upper_arm=upper_arm))
        self.addCommands(DriveSwerveAutoVelocity(self.container, drive, -1, 'forwards').withTimeout(1.5))
