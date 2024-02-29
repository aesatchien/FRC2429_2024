import commands2
import constants

from commands.indexer_toggle import IndexerToggle
from commands.shooter_toggle import ShooterToggle

class AutoShootCycle(commands2.SequentialCommandGroup):
    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutoShootCycle')  # change this to something appropriate for this command
        self.container = container

        # back up indexer, turn on shooter, wait, fire indexer full speed into
        self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=-2, force='on', timeout=0.1))
        self.addCommands(ShooterToggle(container=self.container, shooter=self.container.shooter, force='on', rpm=2000))
        self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=0, force='off', timeout=1))
        self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=5, force='on', timeout=None))
        # self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=0, force='off', timeout=None))
        # self.addCommands(ShooterToggle(container=self.container, shooter=self.container.shooter, force='off'))

