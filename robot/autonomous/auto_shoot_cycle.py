import commands2
import constants

from commands.indexer_toggle import IndexerToggle
from commands.shooter_toggle import ShooterToggle
from commands.arm_smart_go_to import ArmSmartGoTo

class AutoShootCycle(commands2.SequentialCommandGroup):
    def __init__(self, container, go_to_shoot=True) -> None:
        super().__init__()
        self.setName('AutoShootCycle')  # change this to something appropriate for this command
        self.container = container
        self.addCommands(commands2.PrintCommand('** BEGIN AUTO SHOOT CYCLE)'))
        # back up indexer, turn on shooter, wait, fire indexer full speed into,
        self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=-0.33, force='on', timeout=0.1
                                       ))
        self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=0, force='off', timeout=None))
        # in AV speaker shots keep bouncing out - so we lowered from 3500 to 3200 rmp  20240404 CJH
        self.addCommands(ShooterToggle(container=self.container, shooter=self.container.shooter, force='on', rpm=3000, amp_rpm=1500, # usually rpm=3500, amp_rpm=2000;
                                       auto_amp_slowdown=True, wait_for_spinup=True).withTimeout(0.66))
        self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=1, force='on', timeout=0.65))  # this is a bad timeout
        self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=0, force='off', timeout=None))
        self.addCommands(ShooterToggle(container=self.container, shooter=self.container.shooter, force='off'))

        if go_to_shoot:  # This only works in teleop.  3/13/14 lhack we don't use this anymore since only our shooter position (and not intake position) lets us go under the stage
            self.addCommands(ArmSmartGoTo(container=self.container, desired_position='low_shoot'))
