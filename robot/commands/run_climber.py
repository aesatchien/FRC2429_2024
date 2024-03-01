import commands2
import wpilib
from wpilib import SmartDashboard
from subsystems.climber import Climber

class RunClimber(commands2.CommandBase):

    def __init__(self, container, climber:Climber, power=5, timeout=None) -> None:
        super().__init__()
        self.setName('IndexerToggle')
        self.climber = climber
        self.container = container
        self.power = power
        self.timeout = timeout
        self.timer = wpilib.Timer()
        self.addRequirements(climber)

    # TODO: have coast by default but brake when climbing
    def initialize(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)

        self.timer.restart()
        self.climber.set_climber(self.power)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.timeout is None:
            return False
        else:
            return self.timer.get() > self.timeout

    def end(self, interrupted: bool) -> None:
        self.climber.stop_climber()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")




