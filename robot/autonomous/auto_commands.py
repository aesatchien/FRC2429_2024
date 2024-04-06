import commands2

import constants
from pathplannerlib.auto import AutoBuilder, PathPlannerPath
from autonomous.auto_shoot_cycle import AutoShootCycle
from commands.arm_move import ArmMove
from commands.acquire_note_toggle import AcquireNoteToggle
from commands.smart_intake import SmartIntake
from commands.move_arm_by_pose import MoveArmByPose
from commands.drive_and_auto_aim_chassis import DriveAndAutoAimChassis
from commands.unused.change_shooting_direction import ChangeShootingDirection

# LHACK 3/18/24- a bunch of commands used for auto only that don't really deserve their own files IMO

class GetRing(commands2.SequentialCommandGroup):
    def __init__(self, container, ring_num: int, near_or_far: str) -> None:
        super().__init__()
        self.setName(f'Get {near_or_far} ring {ring_num}')  # change this to something appropriate for this command
        self.container = container

        if not near_or_far in ['near', 'far']: raise(ValueError('Don\'t know whether to go for near or far ring!'))
        if near_or_far == 'near' and ring_num > 3: raise(ValueError(f'There is no near ring {ring_num}!'))
        use_smart_intake=False
        if use_smart_intake:
                self.addCommands(
                commands2.ParallelCommandGroup(
                    SmartIntake(container=self.container, wait_to_finish=True, auto=True),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile(f'Get {near_or_far} ring {ring_num}'))
                )
            )
            # Note that this requires certain paths to be present
            # self.addCommands(AcquireNoteToggle(self.container, force='off', timeout=None))
        else:

            self.addCommands(
                commands2.ParallelCommandGroup(
                    self.addCommands(AcquireNoteToggle(self.container, force='on', timeout=None)),
                    GoToIntake(self.container),
                    commands2.WaitCommand(0.5).andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile(f'Get {near_or_far} ring {ring_num}')))
                )
            )
            # Note that this requires certain paths to be present
            self.addCommands(AcquireNoteToggle(self.container, force='on', timeout=0.25))
            self.addCommands(AcquireNoteToggle(self.container, force='off', timeout=None))


class GoToSpeakerAndShoot(commands2.SequentialCommandGroup): #runs up to speaker and shoots backwards
    def __init__(self, container, side: str) -> None:
        super().__init__()
        self.setName(f'Shoot ring on side {side}')
        self.container = container
        shooting_in_intake = True
        if not side in ['ampside', 'middle', 'sourceside']: raise ValueError('Invalid side of speaker!')

        if shooting_in_intake:
            GoToIntake(self.container)
            self.addCommands(
                commands2.ParallelCommandGroup(
                    GoToIntake(self.container),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile(f'Go to {side} speaker side'))
                )
            )
            self.addCommands(AutoShootCycle(self.container, go_to_shoot=False))

        else:
            self.addCommands(ArmMove(container=self.container, arm=self.container.shooter_arm, degrees=constants.k_crank_presets['low_shoot']['upper'], absolute=True, wait_to_finish=True))
            self.addCommands(
                commands2.ParallelRaceGroup(
                    MoveArmByPose(self.container, shooting_backwards=True),
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile(f'Go to {side} speaker side'))
                )
            )
            self.addCommands(
                commands2.ParallelRaceGroup(
                    AutoShootCycle(self.container, go_to_shoot=False),
                    DriveAndAutoAimChassis(self.container, self.container.drive, constants.k_field_centric, constants.k_rate_limited, shooting_backwards=True)
                )
            )

class ShootFromAnywhere(commands2.SequentialCommandGroup): #
    def __init__(self, container) -> None:
        super().__init__()
        self.setName(f'Shoot from anywhere')
        self.container = container

        self.addCommands(
            commands2.ParallelRaceGroup(
                MoveArmByPose(self.container, shooting_backwards=False),
                DriveAndAutoAimChassis(self.container, self.container.drive, constants.k_field_centric, constants.k_rate_limited, shooting_backwards=False),
                commands2.WaitCommand(2.5)
            )
        )
        self.addCommands(AutoShootCycle(self.container, go_to_shoot=False))
        self.addCommands(ArmMove(container=self.container, arm=self.container.shooter_arm, degrees=constants.k_crank_presets['low_shoot']['upper'], absolute=True, wait_to_finish=True))


class ShootPreload(commands2.SequentialCommandGroup):
    def __init__(self, container, time_to_aim) -> None:
        super().__init__()
        self.setName(f'Shoot preload after waiting {time_to_aim} seconds')
        self.container = container

        self.addCommands(GoToIntake(self.container))
        self.addCommands(commands2.WaitCommand(0.3))
        self.addCommands(AutoShootCycle(self.container, go_to_shoot=False))

class GoToShoot(commands2.SequentialCommandGroup):
    def __init__(self, container) -> None:
        super().__init__()
        self.setName('Go to shoot')  # change this to something appropriate for this command
        self.container = container

        self.addCommands(ArmMove(self.container, self.container.crank_arm, constants.k_crank_presets['low_shoot']['lower'],
                                 absolute=True, wait_to_finish=True))
        self.addCommands(ArmMove(self.container, self.container.shooter_arm, constants.k_crank_presets['low_shoot']['upper'],
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
