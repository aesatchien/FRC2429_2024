import commands2
from wpilib import SmartDashboard
from typing import Union

from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal


class CrankArmCoast(commands2.Command):  # change the name for your command

    def __init__(self, container, crank_arm: Union[UpperCrankArmTrapezoidal,LowerCrankArmTrapezoidal]) -> None:
        super().__init__()
        self.setName(f'{crank_arm.getName()} Coast')  # change this to something appropriate for this command
        self.container = container
        self.crank_arm = crank_arm
        self.addRequirements(self.crank_arm)  # commandsv2 version of requirements

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")
        self.crank_arm.set_brake_mode(mode='coast')

        # keep track of if we were enabled or not
        self.initial_state = self.crank_arm.is_enabled()
        self.crank_arm.disable_arm()

    def execute(self) -> None:
        # update the arm with the current angle so it doesn't jerk when we start
        self.crank_arm.set_goal(self.crank_arm.get_angle())

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        if self.initial_state:  # turn the arm back on if it was already on when we started
            self.crank_arm.enable_arm()
        self.crank_arm.set_brake_mode(mode='brake')
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")