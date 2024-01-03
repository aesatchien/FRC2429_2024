import commands2
from wpilib import SmartDashboard
from subsystems.turret import Turret
from wpimath.filter import Debouncer


class TurretReset(commands2.CommandBase):

    def __init__(self, container, turret: Turret, new_angle=None) -> None:
        super().__init__()
        self.setName('TurretReset')
        self.container = container
        self.turret = turret
        self.counter = 0
        self.new_angle = new_angle

        self.addRequirements(self.turret)  # commandsv2 version of requirements

    def runsWhenDisabled(self):  # ok to run when disabled - override the base method
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.turret.reset_turret_encoder(angle=self.new_angle)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")