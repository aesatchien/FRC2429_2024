import commands2

import constants
from commands.arm_move import ArmMove

class GoToShoot(commands2.SequentialCommandGroup):
    def __init__(self, container) -> None:
        super().__init__()
        self.setName('Go to shoot')  # change this to something appropriate for this command
        self.container = container

        self.addCommands(ArmMove(self.container, self.container.crank_arm, constants.k_crank_presets['shoot']['lower'],
                                 absolute=True, wait_to_finish=True))
        self.addCommands(ArmMove(self.container, self.container.shooter_arm, constants.k_crank_presets['shoot']['upper'],
                                 absolute=True, wait_to_finish=True))

class GoToIntake(commands2.SequentialCommandGroup):
    def __init__(self, container) -> None:
        super().__init__()
        self.setName('Go to intake')  # change this to something appropriate for this command
        self.container = container

        self.addCommands(ArmMove(self.container, self.container.shooter_arm, constants.k_crank_presets['intake']['upper'],
                                 absolute=True, wait_to_finish=True))
        self.addCommands(ArmMove(self.container, self.container.crank_arm, constants.k_crank_presets['intake']['lower'],
                                 absolute=True, wait_to_finish=True))

class GoToAmp(commands2.SequentialCommandGroup):
    def __init__(self, container) -> None:
        super().__init__()
        self.setName('Go to amp')  # change this to something appropriate for this command
        self.container = container

        self.addCommands(ArmMove(self.container, self.container.crank_arm, constants.k_crank_presets['amp']['lower'],
                                 absolute=True, wait_to_finish=True))
        self.addCommands(ArmMove(self.container, self.container.shooter_arm, constants.k_crank_presets['amp']['upper'],
                                 absolute=True, wait_to_finish=True))
