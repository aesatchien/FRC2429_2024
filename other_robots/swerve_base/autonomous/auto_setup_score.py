# command group to servo to parallel to wall, detect tag, strafe, drive forward, set elevator, extend arm

import commands2

from autonomous.auto_rotate_swerve import AutoRotateSwerve
from autonomous.auto_strafe_swerve import AutoStrafeSwerve
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from commands.elevator_move import ElevatorMove
from commands.arm_move import ArmMove
from commands.wrist_move import WristMove
from commands2 import WaitCommand

class AutoSetupScore(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container, distance=2) -> None:
        super().__init__()
        self.setName('AutoSetupScore')  # change this to something appropriate for this command
        self.container = container

        # align robot with wall
        self.addCommands(AutoRotateSwerve(container=self.container, drive=self.container.drive).withTimeout(1))

        # pause to give time for an april tag detection
        self.addCommands(WaitCommand(0.25))

        # strafe to target
        self.addCommands(AutoStrafeSwerve(container=self.container, drive=self.container.drive,
                         vision=self.container.vision, target_type='tag', auto=True).withTimeout(3))

        # set elevator to correct height - decide based on turret position
        self.addCommands(ElevatorMove(container=self.container, elevator=self.container.elevator, wait_to_finish=True,
                                      decide_by_turret=True))

        # extend arm - decide on distance based on turret position - but what if strafe failed?
        self.addCommands(ArmMove(container=self.container, arm=self.container.arm, wait_to_finish=False,
                                      decide_by_turret=True))
        # send the wrist to whatever
        self.addCommands(WristMove(container=self.container, wrist=self.container.wrist,
                                   setpoint=55, wait_to_finish=False))

        # drive forward - should get this from the tag if we can / make it optional
        # added a get direction from turret to velocity as well
        # self.addCommands(DriveSwerveAutoVelocity(container=self.container, drive=self.container.drive,
        #                                         velocity=0.5, decide_by_turret=True).withTimeout(0.25))

