import commands2

import constants
from pathplannerlib.auto import AutoBuilder, PathPlannerPath
from autonomous.auto_shoot_cycle import AutoShootCycle
from commands.arm_move import ArmMove
from commands.acquire_note_toggle import AcquireNoteToggle
from commands.move_arm_by_pose import MoveArmByPose

# LHACK 3/18/24- a bunch of commands used for auto only that don't really deserve their own files IMO

class GetRing(commands2.SequentialCommandGroup):
    def __init__(self, container, ring_num: int, near_or_far: str) -> None:
        super().__init__()
        self.setName(f'Get {near_or_far} ring {ring_num}')  # change this to something appropriate for this command
        self.container = container

        if not near_or_far in ['near', 'far']: raise(ValueError('Don\'t know whether to go for near or far ring!'))
        if near_or_far == 'near' and ring_num > 3: raise(ValueError(f'There is no near ring {ring_num}!'))

        self.addCommands(GoToIntake(self.container))
        self.addCommands(AcquireNoteToggle(self.container, force='on', timeout=None))
        # Note that this requires certain paths to be present
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(f'Get {near_or_far} ring {ring_num}')))
        self.addCommands(AcquireNoteToggle(self.container, force='off', timeout=None))

class GoToSpeakerAndShoot(commands2.SequentialCommandGroup):
    def __init__(self, container, side: str) -> None:
        super().__init__()
        self.setName(f'Shoot ring on side {side}')
        self.container = container

        if not side in ['ampside', 'middle', 'sourceside']: raise ValueError('Invalid side of speaker!')

        self.addCommands(
            commands2.ParallelRaceGroup(
                MoveArmByPose(self.container),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile(f'Go to {side} speaker side'))
            )
        )

        self.addCommands(AutoShootCycle(self.container, go_to_intake=False))

class ShootPreload(commands2.SequentialCommandGroup):
    def __init__(self, container, time_to_aim) -> None:
        super().__init__()
        self.setName(f'Shoot preload after waiting {time_to_aim} seconds')
        self.container = container

        self.addCommands(
            commands2.ParallelRaceGroup(
                MoveArmByPose(self.container),
                commands2.SequentialCommandGroup(
                    commands2.WaitCommand(time_to_aim),
                    AutoShootCycle(self.container)
                )
            )
        )

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
