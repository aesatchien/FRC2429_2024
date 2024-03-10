import math

import commands2
from wpilib import SmartDashboard

import constants

from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.intake import Intake
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from subsystems.led import Led

from commands.arm_move import ArmMove
from commands.acquire_note_toggle import AcquireNoteToggle
from commands2 import WaitCommand

class ArmSmartGoTo(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, desired_position: str, wait_for_finish=False) -> None:
        super().__init__()
        self.setName('ArmSmartGoTo')  # change this to something appropriate for this command
        self.container = container
        self.upper_crank: UpperCrankArmTrapezoidal = self.container.shooter_arm
        self.lower_crank: LowerCrankArmTrapezoidal = self.container.crank_arm
        self.intake: Intake = self.container.intake
        self.shooter: Shooter = self.container.shooter
        self.indexer: Indexer = self.container.indexer
        self.desired_position = desired_position
        self.wait_for_finish = wait_for_finish
        self.start_time = 0   # having some problems with this crashing...
        if not self.desired_position in ['intake', 'shoot', 'amp', 'low_amp']: raise ValueError
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        command = None
        if self.desired_position == 'shoot':
            self.container.led.set_indicator(Led.Indicator.READY_SHOOT)

            # add with timeouts on these wait until commands
            command = (AcquireNoteToggle(container=self.container, force='off')
                       .andThen(ArmMove(container=self.container, arm=self.lower_crank, degrees=constants.k_crank_presets['shoot']['lower'], absolute=True, wait_to_finish=True))
                       .andThen(ArmMove(container=self.container, arm=self.upper_crank, degrees=constants.k_crank_presets['shoot']['upper'], absolute=True)))

        elif self.desired_position == 'intake':
            self.container.led.set_indicator(Led.Indicator.INTAKE)

            command = (ArmMove(container=self.container, arm=self.upper_crank, degrees=constants.k_crank_presets['intake']['upper'], absolute=True, wait_to_finish=True)
                       .andThen(ArmMove(self.container, self.lower_crank, degrees=constants.k_crank_presets['intake']['lower'], absolute=True)))

        elif self.desired_position == 'amp':
            self.container.led.set_indicator(Led.Indicator.AMP)

            command = (AcquireNoteToggle(container=self.container, force='off')
                       .andThen(ArmMove(container=self.container, arm=self.lower_crank, degrees=constants.k_crank_presets['amp']['lower'], absolute=True, wait_to_finish=True))
                       .andThen(ArmMove(container=self.container, arm=self.upper_crank, degrees=constants.k_crank_presets['amp']['upper'], absolute=True)))

        elif self.desired_position == 'low_amp':
            self.container.led.set_indicator(Led.Indicator.AMP)

            command = (AcquireNoteToggle(container=self.container, force='off')
                       .andThen(ArmMove(container=self.container, arm=self.lower_crank, degrees=constants.k_crank_presets['low_amp']['lower'], absolute=True, wait_to_finish=True))
                       .andThen(ArmMove(container=self.container, arm=self.upper_crank, degrees=constants.k_crank_presets['low_amp']['upper'], absolute=True)))
        else:
            pass  # should throw an error

        commands2.CommandScheduler.getInstance().schedule(command)


        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s to {self.desired_position} **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.wait_for_finish:
            return self.lower_crank.get_at_goal() and self.upper_crank.get_at_goal()
        else:
            return True

    def end(self, interrupted: bool) -> None:
        pass
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")