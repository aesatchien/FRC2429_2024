import commands2
import wpilib
from wpilib import SmartDashboard
from subsystems.intake import Intake

class IntakeToggle(commands2.CommandBase):

    def __init__(self, container, intake:Intake, rpm=1000, force=None, timeout=None) -> None:
        super().__init__()
        self.setName('IntakeToggle')
        self.intake = intake
        self.container = container
        self.rpm = rpm
        self.force = force
        self.timeout = timeout
        self.timer = wpilib.Timer()
        self.addRequirements(intake)

    def initialize(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)

        self.timer.restart()

        if self.force == 'on':
            self.intake.set_intake_motor(self.rpm)
        elif self.force == 'off':
            self.intake.stop_intake()
        else:
            self.intake.toggle_intake(self.rpm)

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




