import commands2
from wpilib import SmartDashboard
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.shooter_crank_trapezoid import ShooterCrankArmTrapezoidal
from typing import Union
import math

class ArmMove(commands2.Command):

    def __init__(self, container, crank_arm: Union[ShooterCrankArmTrapezoidal,LowerCrankArmTrapezoidal], degrees=0, wait_to_finish=True) -> None:
        super().__init__()
        self.setName(f'{crank_arm.getName()}_Move')
        self.container = container
        self.crank_arm = crank_arm  # should work with either type of arm ?
        self.degrees = degrees
        self.wait_to_finish = wait_to_finish  # determine how long we wait to end

        self.addRequirements(self.crank_arm)  # commandsv2 version of requirements

    def initialize(self) -> None:
        position = self.crank_arm.get_angle()
        goal = position + self.degrees * math.pi / 180
        self.crank_arm.move_degrees(self.degrees)
        self.print_start_message()
        self.crank_arm.is_moving = True  # try to raise a flag that lets us know we're in motion

        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        message = f'setting {self.getName()} from {position * 180 / math.pi:.1f} to {goal * 180 / math.pi:.1f}'
        print(message)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        # print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")