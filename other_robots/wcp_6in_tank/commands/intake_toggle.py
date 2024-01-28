import commands2
from wpilib import SmartDashboard

class IntakeToggle(commands2.CommandBase):

    SmartDashboard.putNumber('set intake rpm', 2500)

    def __init__(self, container, intake, rpm=3000, force=None, ) -> None:
        super().__init__()
        self.setName('IntakeToggle')
        self.intake = intake
        self.container = container
        self.rpm = rpm
        self.force = force
        self.addRequirements(intake)

    def initialize(self) -> None:

        rpm = SmartDashboard.getNumber ('set intake rpm', 2500)

        if self.force == 'on':
            self.intake.set_intake(self.rpm)
        elif self.force == 'off':
            self.intake.stop_intake()
        else:
            self.intake.toggle_intake(rpm)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")




