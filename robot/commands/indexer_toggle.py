import commands2
import wpilib
from wpilib import SmartDashboard
from subsystems.indexer import Indexer

class IndexerToggle(commands2.CommandBase):

    def __init__(self, container, indexer:Indexer, power=1, force=None, timeout=None) -> None:
        super().__init__()
        self.setName('IndexerToggle')
        self.indexer = indexer
        self.container = container
        self.force = force
        self.power = power
        self.timeout = timeout
        self.timer = wpilib.Timer()
        self.addRequirements(indexer)

    def initialize(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"** Started {self.getName()} at {self.start_time} s  with power {self.power}, force {self.force}, and timeout {self.timeout} **", flush=True)

        self.timer.restart()

        if self.force == 'on':
            self.indexer.set_indexer(self.power)
        elif self.force == 'off':
            self.indexer.stop_indexer()
        else:
            self.indexer.toggle_indexer(self.power)

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
        print_end_message = False
        if print_end_message:
            print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
            SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")




