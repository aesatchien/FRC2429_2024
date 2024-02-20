import commands2
from wpilib import SmartDashboard


class ClimberToggle(commands2.Command):

    SmartDashboard.putNumber('set climber rpm', 2500)

    def __init__(self, container, climber, rpm=3000, force=None) -> None:
        super().__init__()
        self.setName('ClimberToggle')
        self.climber = climber
        self.container = container
        self.rpm = rpm
        self.force = force
        self.addRequirements(climber)

    def initialize(self) -> None:
        rpm = SmartDashboard.getNumber('set shooter rpm', 2500)

        if self.force == 'on':
            self.climber.set_climber(self.rpm)
        elif self.force == 'off':
            self.climber.stop_climber()
        else:
            self.climber.toggle_climber(rpm)

        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Firing {self.getName()} with force={self.force} at {self.start_time} s **", flush=True)
        # SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted:bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **",flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")