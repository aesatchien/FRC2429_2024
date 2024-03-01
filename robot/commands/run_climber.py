import commands2
from wpilib import SmartDashboard
import navx
from subsystems.climber import Climber

class RunClimber(commands2.CommandBase):

    def __init__(self, container, climber:Climber, navx : navx.AHRS=None, left_volts=2, right_volts=2) -> None:
        super().__init__()
        self.setName('IndexerToggle')
        self.climber = climber
        self.container = container
        self.navx = navx
        self.left_volts = left_volts
        self.right_volts = right_volts
        self.addRequirements(climber)

    # TODO: have coast by default but brake when climbing
    def initialize(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)

        self.timer.restart()
        if self.navx is None:
            self.climber.set_climber(self.left_volts, self.right_volts)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.navx is not None:
            self.climber.stop_climber()
        return False

    def end(self, interrupted: bool) -> None:
        self.climber.stop_climber()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")




