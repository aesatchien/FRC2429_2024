import commands2
import wpilib
from wpilib import SmartDashboard
from subsystems.intake import Intake
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from subsystems.led import Led

class IntakeToggle(commands2.CommandBase):

    def __init__(self, container, intake:Intake, rpm=1000, force=None, timeout=None) -> None:
        super().__init__()
        self.setName('IntakeToggle')
        self.intake = intake
        self.indexer : Indexer = container.indexer
        self.shooter : Shooter = container.shooter
        self.container = container
        self.rpm = rpm
        self.force = force
        self.timeout = timeout
        self.timer = wpilib.Timer()
        self.addRequirements(intake)

    def initialize(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s with force {self.force}**", flush=True)

        self.timer.restart()

        if self.force == 'on':
            self.container.led.set_indicator(Led.Indicator.INTAKE_ON)
            self.intake.set_intake_motor(self.rpm)
            self.indexer.set_indexer(1)
        elif self.force == 'off':
            self.container.led.set_indicator(Led.Indicator.NONE)
            self.intake.stop_intake()
            self.indexer.stop_indexer()
            self.shooter.stop_shooter()
        else:
            if self.intake.intake_enabled:
                self.container.led.set_indicator(Led.Indicator.NONE)
            else:
                self.container.led.set_indicator(Led.Indicator.INTAKE_ON)
            self.intake.toggle_intake(self.rpm)
            self.indexer.toggle_indexer(power=1)
            self.shooter.toggle_shooter()


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




