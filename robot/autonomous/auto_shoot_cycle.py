import commands2
import constants

from commands.indexer_toggle import IndexerToggle
from commands.shooter_toggle import ShooterToggle
from commands.arm_smart_go_to import ArmSmartGoTo

class AutoShootCycle(commands2.SequentialCommandGroup):
    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutoShootCycle')  # change this to something appropriate for this command
        self.container = container

        # back up indexer, turn on shooter, wait, fire indexer full speed into,
        self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=-1, force='on', timeout=0.1))
        self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=0, force='off', timeout=None))
        self.addCommands(ShooterToggle(container=self.container, shooter=self.container.shooter, force='on', rpm=3500, amp_rpm=1500, # usually rpm=3500, amp_rpm=2000
                                       auto_amp_slowdown=True, wait_for_spinup=True).withTimeout(1))
        self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=1, force='on', timeout=1))  # this is a bad timeout
        self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=0, force='off', timeout=None))
        self.addCommands(ShooterToggle(container=self.container, shooter=self.container.shooter, force='off'))
        self.addCommands(ArmSmartGoTo(container=self.container, desired_position='intake'))
