# both arms move with the same commands, so no reason to write two different commands for this
# this command moves the arm subsystem to a specific location, using the trapezoidal subsystem's default rates
import commands2
from wpilib import SmartDashboard
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from typing import Union
import math

class ArmMove(commands2.Command):

    def __init__(self, container, arm: Union[UpperCrankArmTrapezoidal,LowerCrankArmTrapezoidal], degrees=5, direction=None, absolute=False, wait_to_finish=True) -> None:
        super().__init__()
        self.setName(f'{arm.getName()}_Move')
        self.container = container
        self.arm = arm  # should work with either type of arm ?
        self.degrees = degrees
        self.direction = direction
        self.wait_to_finish = wait_to_finish  # determine how long we wait to end
        self.absolute = absolute
        self.addRequirements(self.arm)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        self.print_start_message()
        if self.direction is None:
            if self.absolute:
                self.arm.set_goal(math.radians(self.degrees))
            else:
                self.arm.move_degrees(self.degrees)
        else:
            self.arm.set_next_position(direction=self.direction)

        self.arm.is_moving = True  # try to raise a flag that lets us know we're in motion
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            print(f"Are we at goal? {self.arm.get_at_goal()}")
            return self.arm.get_at_goal()
        else:
            print("Called as fire-and-forget, returning true")
            return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s at {self.arm.get_angle()} after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s at {self.arm.get_angle()} after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")