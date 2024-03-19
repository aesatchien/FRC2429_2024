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

    # note: wait_for_finish doesn't work since the subsystem's goal doesn't seem to update immediately.
    # for certain pairs of setpoints, it works...

    def __init__(self, container, desired_position: str, wait_for_finish=False) -> None:
        super().__init__()
        self.setName('ArmSmartGoTo')  # change this to something appropriate for this command
        self.container = container
        self.shooter_arm: UpperCrankArmTrapezoidal = self.container.shooter_arm
        self.crank_arm: LowerCrankArmTrapezoidal = self.container.crank_arm
        self.intake: Intake = self.container.intake
        self.shooter: Shooter = self.container.shooter
        self.indexer: Indexer = self.container.indexer
        self.led: Led = self.container.led
        self.desired_position = desired_position
        self.wait_for_finish = wait_for_finish
        self.start_time = 0   # having some problems with this crashing...
        if not self.desired_position in ['intake', 'shoot', 'low_shoot','amp', 'low_amp', 'bottom']: raise ValueError
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:

        # let container remember where we are for shooter rpm - 20240319 CJH
        self.container.set_arm_configuration(self.desired_position)
        # get the current arm positions
        shooter_angle = math.degrees(self.shooter_arm.get_angle())
        crank_angle = math.degrees(self.crank_arm.get_angle())

        command = None

        if self.desired_position == 'shoot':
            self.led.set_indicator_with_timeout(Led.Indicator.READY_SHOOT, 5).schedule()
            shooter_goal = constants.k_crank_presets['shoot']['lower']
            crank_goal = constants.k_crank_presets['shoot']['upper']
            command = (AcquireNoteToggle(container=self.container, force='off')
                       .andThen(ArmMove(container=self.container, arm=self.crank_arm, degrees=constants.k_crank_presets['shoot']['lower'], absolute=True, wait_to_finish=True))
                       .andThen(ArmMove(container=self.container, arm=self.shooter_arm, degrees=constants.k_crank_presets['shoot']['upper'], absolute=True)))

        elif self.desired_position == 'low_shoot':
            self.led.set_indicator_with_timeout(Led.Indicator.READY_SHOOT, 5).schedule()
            shooter_goal = constants.k_crank_presets['low_shoot']['lower']
            crank_goal = constants.k_crank_presets['low_shoot']['upper']
            command = (AcquireNoteToggle(container=self.container, force='off')
                       .andThen(ArmMove(container=self.container, arm=self.shooter_arm, degrees=constants.k_crank_presets['low_shoot']['upper'], absolute=True, wait_to_finish=True))
                       .andThen(ArmMove(container=self.container, arm=self.crank_arm, degrees=constants.k_crank_presets['low_shoot']['lower'], absolute=True)))

        elif self.desired_position == 'intake':
            self.led.set_indicator_with_timeout(Led.Indicator.INTAKE, 5).schedule()
            shooter_goal = constants.k_crank_presets['intake']['lower']
            crank_goal = constants.k_crank_presets['intake']['upper']
            command = (ArmMove(container=self.container, arm=self.shooter_arm, degrees=constants.k_crank_presets['intake']['upper'], absolute=True, wait_to_finish=True)
                       .andThen(ArmMove(self.container, self.crank_arm, degrees=constants.k_crank_presets['intake']['lower'], absolute=True)))

        elif self.desired_position == 'amp':
            self.led.set_indicator_with_timeout(Led.Indicator.AMP, 5).schedule()
            shooter_goal = constants.k_crank_presets['amp']['lower']
            crank_goal = constants.k_crank_presets['amp']['upper']
            command = (AcquireNoteToggle(container=self.container, force='off')
                       .andThen(ArmMove(container=self.container, arm=self.crank_arm, degrees=constants.k_crank_presets['amp']['lower'], absolute=True, wait_to_finish=True))
                       .andThen(ArmMove(container=self.container, arm=self.shooter_arm, degrees=constants.k_crank_presets['amp']['upper'], absolute=True)))

        elif self.desired_position == 'low_amp':
            self.led.set_indicator_with_timeout(Led.Indicator.AMP, 5).schedule()
            shooter_goal = constants.k_crank_presets['low_amp']['lower']
            crank_goal = constants.k_crank_presets['low_amp']['upper']
            command = (AcquireNoteToggle(container=self.container, force='off')
                       .andThen(ArmMove(container=self.container, arm=self.crank_arm, degrees=constants.k_crank_presets['low_amp']['lower'], absolute=True, wait_to_finish=True))
                       .andThen(ArmMove(container=self.container, arm=self.shooter_arm, degrees=constants.k_crank_presets['low_amp']['upper'], absolute=True)))

        elif self.desired_position == 'bottom':
            self.led.set_indicator_with_timeout(Led.Indicator.INTAKE, 5).schedule()
            shooter_goal = constants.k_crank_presets['bottom']['lower']
            crank_goal = constants.k_crank_presets['bottom']['upper']
            command = (ArmMove(container=self.container, arm=self.shooter_arm12, degrees=constants.k_crank_presets['bottom']['upper'], absolute=True, wait_to_finish=True)
                       .andThen(ArmMove(self.container, self.crank_arm, degrees=constants.k_crank_presets['bottom']['lower'], absolute=True)))

        else:
            pass  # should throw an error

        # logic to determine which arm moves first - four possibilities
        if shooter_goal > shooter_angle and crank_goal > crank_angle:  # both moving up
            pass
        elif shooter_goal < shooter_angle and crank_goal < crank_angle:  # both moving down
            pass
        elif shooter_goal > shooter_angle and crank_goal < crank_angle:  # shooter up, crank down
            pass
        elif shooter_goal < shooter_angle and crank_goal > crank_angle:  # shooter down, crank up
            pass
        else:  # someone isn't moving, so whichever is fine?
            pass


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
            return self.crank_arm.get_at_goal() and self.upper_crank.get_at_goal()
        else:
            return True

    def end(self, interrupted: bool) -> None:
        pass
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = False
        if print_end_message:
            print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")