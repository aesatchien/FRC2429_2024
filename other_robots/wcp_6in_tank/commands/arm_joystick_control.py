# both arms move with the same commands, so no reason to write two different commands for this
# this command moves the arm subsystem up and down via joystick, using the trapezoidal subsystem's default rates
import math

import commands2
import wpilib
from wpilib import SmartDashboard
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
import typing

class ArmJoystickControl(commands2.Command):

    def __init__(self, container, arm: typing.Union[UpperCrankArmTrapezoidal,LowerCrankArmTrapezoidal], degrees=1, controller:wpilib.XboxController=None, wait_to_finish=True) -> None:
        super().__init__()
        self.setName(f'{arm.getName()}_JoystickControl')
        self.container = container
        self.arm = arm  # should work with either type of arm ?
        self.degrees = degrees
        self.wait_to_finish = wait_to_finish  # determine how long we wait to end
        self.controller = controller
        self.addRequirements(self.arm)  # commandsv2 version of requirements
        self.counter = 0

    def initialize(self) -> None:

        self.start_time = round(self.container.get_enabled_time(), 2)
        self.print_start_message()
        self.arm.is_moving = True  # try to raise a flag that lets us know we're in motion
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        self.counter += 1
        if self.counter % 10 == 0:
            if self.arm.getName() == 'crank_arm':
                joystick_value = self.controller.getLeftX()
            else:
                joystick_value = -1 * self.controller.getLeftY()

            joystick_value = 0 if math.fabs(joystick_value) < 0.1 else joystick_value  # give it a deadband
            # print(f'{self.arm.getName()} joystick: {joystick_value}')
            self.arm.move_degrees(joystick_value * self.degrees, silent=True)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        # print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")