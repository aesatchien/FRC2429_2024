import commands2
import constants

from commands.indexer_toggle import IndexerToggle
from commands.shooter_toggle import ShooterToggle
from commands.arm_move import ArmMove
from commands.arm_smart_go_to import ArmSmartGoTo
from subsystems.led import Led

class AutoClimbArm(commands2.SequentialCommandGroup):
    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutoClimbArm')  # change this to something appropriate for this command
        self.container = container

        # back up indexer, turn on shooter, wait, fire indexer full speed into
        # self.addCommands(commands2.ParallelCommandGroup(
        #     self.addCommands(self.self.container.led.set_indicator_with_timeout(Led.Indicator.CLIMB, 5))
        #
        # ))
        # self.addCommands(IndexerToggle(container=self.container, indexer=self.container.indexer, power=1, force='on', timeout=None))
        # self.addCommands(ArmMove(container=self.container, arm=self.container.crank_arm, degrees=90, absolute=True, wait_to_finish=True))
        # self.addCommands(ArmMove(container=self.container, arm=self.container.shooter_arm, degrees=30, absolute=True, wait_to_finish=True))
        self.addCommands(
            commands2.ParallelCommandGroup(
            self.container.led.set_indicator_with_timeout(Led.Indicator.CLIMB, 5),
            commands2.SequentialCommandGroup(
                IndexerToggle(container=self.container, indexer=self.container.indexer, power=1, force='on',
                              timeout=None),
                # ArmSmartGoTo(container=self.container, desired_position='shoot'),
                # IndexerToggle(container=self.container, indexer=self.container.indexer, power=1, force='on',
                #               timeout=2),
                # ArmSmartGoTo(container=self.container, desired_position='climb_second')
                ArmMove(container=self.container, arm=self.container.crank_arm,
                        degrees=constants.k_crank_presets['shoot']['lower'], absolute=True, wait_to_finish=True),
                ArmMove(container=self.container, arm=self.container.shooter_arm,
                                 degrees=constants.k_crank_presets['shoot']['upper'], absolute=True),
                IndexerToggle(container=self.container, indexer=self.container.indexer, power=1, force='on', timeout=2),
                ArmMove(container=self.container, arm=self.container.shooter_arm,
                        degrees=constants.k_crank_presets['climb_second']['upper'], absolute=True),
            ),
            )
        )




