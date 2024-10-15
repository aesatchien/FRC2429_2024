import math
import commands2
import wpilib
from wpilib import SmartDashboard
from subsystems.led import Led
from subsystems.intake import Intake
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from commands.arm_smart_go_to import ArmSmartGoTo
from commands.arm_move import ArmMove
import constants


class SmartIntake(commands2.Command):

    def __init__(self, container, auto=False, timeout=15, wait_to_finish=True) -> None:
        super().__init__()
        self.setName('SmartIntake')
        self.container = container
        self.wait_to_finish = wait_to_finish
        self.auto = auto  # do things differently if called from autonomous
        self.timeout = timeout
        self.timer = wpilib.Timer()

        self.led: Led = container.led
        self.shooter: Shooter = container.shooter
        self.intake: Intake = container.intake
        self.indexer: Indexer = container.indexer

        self.intake_power = constants.k_intake_output
        self.indexer_power = constants.k_indexer_output

        self.addRequirements(self.intake)
        self.count = 0

    def initialize(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)

        self.timer.restart()

        # always stop the shooter no matter what you do here
        self.shooter.stop_shooter()

        # turn on intake and indexer
        self.intake.set_intake(power=self.intake_power)
        self.indexer.set_indexer(power=self.indexer_power)

        # indicate we have started
        self.container.led.set_indicator(Led.Indicator.INTAKE_ON)

    def execute(self) -> None:
        self.count += 1

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return self.shooter.is_ring_loaded() or self.timer.hasElapsed(self.timeout)
        else:
            return True

    def end(self, interrupted: bool) -> None:
        if interrupted:
            self.led.set_indicator_with_timeout(Led.Indicator.CALIBRATION_FAIL, 4).schedule()
            print(f'Failed to intake ring from {self.getName()}')
            self.intake.stop_intake()
            self.indexer.stop_indexer()
        else:
            self.led.set_indicator_with_timeout(Led.Indicator.CALIBRATION_SUCCESS, 4).schedule()
            self.intake.stop_intake()
            self.indexer.stop_indexer()

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")




