import math

import commands2
from wpilib import SmartDashboard

import constants

from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal

from commands.arm_move import ArmMove
from commands2 import WaitCommand

class ArmSmartGoTo(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, upper_crank: UpperCrankArmTrapezoidal, lower_crank: LowerCrankArmTrapezoidal, desired_position: str) -> None:
        super().__init__()
        self.setName('Arm smart go to')  # change this to something appropriate for this command
        self.container = container
        self.upper_crank = upper_crank
        self.lower_crank = lower_crank
        self.desired_position = desired_position
        if not self.desired_position in ['intake', 'shoot', 'amp']: raise ValueError
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        command = None
        if self.desired_position == 'shoot':
            self.container.led.set_indicator('VISION_TARGET_FAILURE')
            if self.lower_crank.get_angle() < math.radians(80):
                crank_wait_time = 1.5
            else:
                crank_wait_time = 0
            command = (ArmMove(container=self.container, arm=self.lower_crank, degrees=constants.k_crank_presets['shoot']['lower'], absolute=True)
                        .andThen(WaitCommand(crank_wait_time))
                        .andThen(ArmMove(container=self.container, arm=self.upper_crank, degrees=constants.k_crank_presets['shoot']['upper'], absolute=True)))

        elif self.desired_position == 'intake':
            self.container.led.set_indicator('VISION_TARGET_SUCCESS')
            if self.upper_crank.get_angle() > math.radians(-50):
                shooter_wait_time = 1.5
            else:
                shooter_wait_time = 0
            command = (ArmMove(container=self.container, arm=self.upper_crank, degrees=constants.k_crank_presets['intake']['upper'], absolute=True)
                        .andThen(WaitCommand(shooter_wait_time)
                        .andThen(ArmMove(self.container, self.lower_crank, degrees=constants.k_crank_presets['intake']['lower'], absolute=True))))

        elif self.desired_position == 'amp':
            self.container.led.set_mode('cube')
            if self.lower_crank.get_angle() < math.radians(80):
                crank_wait_time = 1.5
            else:
                print()
                crank_wait_time = 0
            command = (ArmMove(container=self.container, arm=self.lower_crank, degrees=constants.k_crank_presets['amp']['lower'], absolute=True)
                       .andThen(WaitCommand(crank_wait_time))
                       .andThen(ArmMove(container=self.container, arm=self.upper_crank, degrees=constants.k_crank_presets['amp']['upper'], absolute=True)))

        commands2.CommandScheduler.getInstance().schedule(command)


        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s to {self.desired_position} **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        pass
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")