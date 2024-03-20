import commands2
from wpilib import SmartDashboard

import constants
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from autonomous.auto_shoot_cycle import AutoShootCycle
from commands.indexer_toggle import IndexerToggle
from commands.shooter_toggle import ShooterToggle
from commands.arm_move import ArmMove
from commands.arm_smart_go_to import ArmSmartGoTo
from commands.toggle_climb_servos import ToggleClimbServos
from commands.run_climber import RunClimber
from subsystems.led import Led


class AutoClimbArm2(commands2.Command):  # try to auto climb

    stage = 0  # keep track of how many times we have climbed

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutoClimbArm2')  # change this to something appropriate for this command
        self.container = container
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.container.set_arm_configuration('trap')  # keep track of config for shot speeds and folding up
        self.stage += 1
        print("\n" + f"** Started {self.getName()} at at stage < {self.stage} > {self.start_time} s **", flush=True)

        if self.stage == 1:  # initial part of climb - raise and move chain
            command = commands2.ParallelCommandGroup(
                self.container.led.set_indicator_with_timeout(Led.Indicator.CLIMB, 5),
                          commands2.SequentialCommandGroup(
                    IndexerToggle(container=self.container, indexer=self.container.indexer, power=1, force='on', timeout=None),
                              ArmMove(container=self.container, arm=self.container.crank_arm,
                                      degrees=constants.k_crank_presets['shoot']['lower'], absolute=True,
                                      wait_to_finish=True),
                              ArmMove(container=self.container, arm=self.container.shooter_arm,
                                      degrees=constants.k_crank_presets['shoot']['upper'], absolute=True),
                              IndexerToggle(container=self.container, indexer=self.container.indexer, power=1,
                                            force='on', timeout=2),
                              ArmMove(container=self.container, arm=self.container.shooter_arm,
                                      degrees=constants.k_crank_presets['climb_second']['upper'], absolute=True),

                          )
                      )

        elif self.stage == 2: # second part of climb - drive back and deploy servos
            command = commands2.ParallelCommandGroup(
                self.container.led.set_indicator_with_timeout(Led.Indicator.CLIMB, 5),
                        commands2.SequentialCommandGroup(
                            ArmMove(container=self.container, arm=self.container.shooter_arm,
                                        degrees=20, absolute=True, wait_to_finish=True),
                                    DriveSwerveAutoVelocity(self.container, self.container.drive, -0.2, 'forwards').withTimeout(1.5),
                                    ToggleClimbServos(self.container, self.container.climber, force='on'),
                                    IndexerToggle(container=self.container, indexer=self.container.indexer, power=1, force='off', timeout=1),
                        )
                       )

        elif self.stage == 3:  # third part of climb - climb up a bit, lower shooter, set crank?
            # command = commands2.PrintCommand("Nothing to do in stage 3 yet - Sanjith should climb a bit")
            command = commands2.ParallelCommandGroup(
                self.container.led.set_indicator_with_timeout(Led.Indicator.CLIMB, 5),
                commands2.SequentialCommandGroup(
                    RunClimber(container=self.container, climber=self.container.climber, left_volts=3, right_volts=3).withTimeout(2.0),
                    ArmMove(container=self.container, arm=self.container.shooter_arm,
                            degrees=0, absolute=True, wait_to_finish=True),
                    ArmMove(container=self.container, arm=self.container.crank_arm,
                            degrees=100, absolute=True, wait_to_finish=True),
                    ToggleClimbServos(self.container, self.container.climber, force='off'),
                )
            )
        elif self.stage == 4:  # last part of climb - deploy trap opener, shoot
            command = commands2.ParallelCommandGroup(
                self.container.led.set_indicator_with_timeout(Led.Indicator.CLIMB, 5),
                        commands2.SequentialCommandGroup(
                            commands2.InstantCommand(lambda:self.container.climber.toggle_trap_servo),  # can't use a runCommand here
                            commands2.PrintCommand("Just opened the trap servo"),
                            IndexerToggle(container=self.container, indexer=self.container.indexer, power=1, force='off', timeout=1),
                            AutoShootCycle(container=self.container, go_to_shoot=False),
                        )
                       )
        else:  # reset or do nothing?
            # self.stage = 0
            command = commands2.PrintCommand(f"Nothing to do in exception stage ({self.stage})")

        # schedule the command
        command.schedule()

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = False
        if print_end_message:
            print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")