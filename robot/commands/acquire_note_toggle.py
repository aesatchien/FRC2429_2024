import commands2
import wpilib
from wpilib import SmartDashboard
from subsystems.intake import Intake
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from subsystems.led import Led

class AcquireNoteToggle(commands2.CommandBase):

    def __init__(self, container, force=None, timeout=None) -> None:
        super().__init__()
        self.setName('AcquireNoteToggle')
        self.intake: Intake = container.intake
        self.indexer: Indexer = container.indexer
        self.shooter: Shooter = container.shooter
        self.container = container
        self.force = force
        self.timeout = timeout
        self.timer = wpilib.Timer()
        self.addRequirements(self.intake, self.indexer, self.shooter)

        self.intake_power = 0.5
        self.indexer_power = 1

    def initialize(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s with force {self.force}**", flush=True)

        self.timer.restart()

        if self.force == 'on':
            self.container.led.set_indicator(Led.Indicator.INTAKE_ON)
            self.intake.set_intake(power=self.intake_power)
            self.indexer.set_indexer(power=self.indexer_power)
        elif self.force == 'off':
            # self.container.led.set_indicator(Led.Indicator.KILL)
            self.intake.stop_intake()
            self.indexer.stop_indexer()
        else:
            if self.intake.intake_on:
                self.container.led.set_indicator(Led.Indicator.NONE)
            else:
                self.container.led.set_indicator(Led.Indicator.INTAKE_ON)
            self.intake.toggle_intake(power=self.intake_power)
            self.indexer.toggle_indexer(power=self.indexer_power)

        # always stop the shooter no matter what you do here
        self.shooter.stop_shooter()


    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.timeout is None:
            return True
        else:
            return self.timer.get() > self.timeout

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")




