import commands2
from wpilib import SmartDashboard
from subsystems.indexer import Indexer

class IndexerToggle(commands2.CommandBase):

    def __init__(self, container, indexer:Indexer, power=5, force=None, ) -> None:
        super().__init__()
        self.setName('IndexerToggle')
        self.indexer = indexer
        self.container = container
        self.force = force
        self.power = power
        self.addRequirements(indexer)

    def initialize(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)

        if self.force == 'on':
            self.indexer.set_indexer(self.power)
        elif self.force == 'off':
            self.indexer.stop_indexer()
        else:
            self.indexer.toggle_indexer(self.power)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")




