#  Container for 2429's 2023 swerve robot with turret, elevator, arm, wrist, and manipulator
import math
import time
import wpilib
from commands2 import WaitCommand
from commands2.button import CommandXboxController
import os

# pathplanner
from pathplannerlib.auto import NamedCommands, AutoBuilder, PathPlannerPath

import constants
from autonomous.pathplannermaker import PathPlannerConfiguration
from autonomous.pathplannermakercommand import PathPlannerConfigurationCommand, AutomatedPath

# subsystems
from subsystems.swerve import Swerve
from subsystems.intake import Intake
from subsystems.led import Led
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from subsystems.climber import Climber
from subsystems.vision import Vision

# auto
from autonomous.auto_shoot_cycle import AutoShootCycle
from autonomous.playback_auto import PlaybackAuto
from autonomous.aim_and_shoot import AimAndShoot
from autonomous.shoot_and_drive_out import ShootAndDriveOut
from autonomous.auto_climb_arm import AutoClimbArm
# from autonomous.shoot_and_drive_out_reset_gyro import ShootAndDriveOutResetGyro

# commands
from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from commands.gyro_reset import GyroReset
from commands.led_toggle import LedToggle
from commands.acquire_note_toggle import AcquireNoteToggle
from commands.arm_move import ArmMove
from commands.arm_smart_go_to import ArmSmartGoTo
from commands.indexer_toggle import IndexerToggle
from commands.intake_toggle import IntakeToggle
from commands.shooter_toggle import ShooterToggle
from commands.run_climber import RunClimber
from commands.toggle_climb_servos import ToggleClimbServos
from commands.record_auto import RecordAuto
from commands.eject_all import EjectAll
from commands.calibrate_lower_crank_by_limit_switch import CalibrateLowerCrankByLimitSwitch
from commands.arm_coast import CrankArmCoast
from commands.arm_preset_go_tos import GoToShoot, GoToIntake, GoToAmp
from commands.move_arm_by_pose import MoveArmByPose
from commands.drive_and_auto_aim_chassis import DriveAndAutoAimChassis

# autonomous
from autonomous.drive_wait import DriveWait



class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        self.start_time = time.time()

        # The robot's subsystems
        self.drive = Swerve()

        if not constants.k_swerve_only:
            self.intake = Intake()
            self.led = Led()
            self.crank_arm = LowerCrankArmTrapezoidal()
            self.shooter_arm = UpperCrankArmTrapezoidal()
            self.indexer = Indexer()
            self.shooter = Shooter()
            self.climber = Climber()
            self.vision = Vision()

        self.registerCommands()

        # set up driving
        self.configure_driver_joystick()
        self.configure_swerve_bindings()
        # swerve driving
        self.drive.setDefaultCommand(DriveByJoystickSwerve(container=self, swerve=self.drive,
                            field_oriented=constants.k_field_centric, rate_limited=constants.k_rate_limited))

        # optionally skip the copilot and non-drivetrain subsystems for debugging
        if not constants.k_swerve_only:  # only test the swerve - no other subsystems
            self.bind_driver_buttons()
            if constants.k_enable_copilot:
                self.configure_copilot_joystick()
                self.bind_copilot_buttons()

        self.initialize_dashboard()

        self.arm_mode = 'intake'
        # arm_degrees = 10 if wpilib.RobotBase.isReal() else 100
        # self.indexer.setDefaultCommand(IndexerByJoystick(container=self, indexer=self.indexer))
        # self.led.setDefaultCommand(LedLoop(container=self))

    def set_start_time(self):  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def get_enabled_time(self):  # call when we want to know the start/elapsed time for status and debug messages
        return time.time() - self.start_time

    def configure_driver_joystick(self):
        # The driver's controller
        self.driver_command_controller = CommandXboxController(constants.k_driver_controller_port)  # 2024 way
        self.trigger_a = self.driver_command_controller.a()  # 2024 way
        self.trigger_b = self.driver_command_controller.b()
        self.trigger_x = self.driver_command_controller.x()
        self.trigger_y = self.driver_command_controller.y()
        self.trigger_rb = self.driver_command_controller.rightBumper()
        self.trigger_start = self.driver_command_controller.start()
        self.trigger_d = self.driver_command_controller.povDown()
        self.trigger_u = self.driver_command_controller.povUp()
        self.trigger_r = self.driver_command_controller.povRight()
        self.trigger_l = self.driver_command_controller.povLeft()

    def configure_copilot_joystick(self):
        self.co_pilot_command_controller = CommandXboxController(constants.k_co_pilot_controller_port)  # 2024 way
        self.co_trigger_left_stick_y = self.co_pilot_command_controller.axisGreaterThan(axis=1, threshold=0.5)
        self.co_trigger_a = self.co_pilot_command_controller.a()  # 2024 way
        self.co_trigger_b = self.co_pilot_command_controller.b()
        self.co_trigger_y = self.co_pilot_command_controller.y()
        self.co_trigger_x = self.co_pilot_command_controller.x()
        self.co_trigger_rb = self.co_pilot_command_controller.rightBumper()
        self.co_trigger_lb = self.co_pilot_command_controller.leftBumper()
        self.co_trigger_r = self.co_pilot_command_controller.povRight()
        self.co_trigger_l = self.co_pilot_command_controller.povLeft()
        self.co_trigger_u = self.co_pilot_command_controller.povUp()
        self.co_trigger_d = self.co_pilot_command_controller.povDown()
        self.co_trigger_l_trigger = self.co_pilot_command_controller.leftTrigger(0.2)
        self.co_trigger_r_trigger = self.co_pilot_command_controller.rightTrigger(0.2)
        self.co_trigger_start = self.co_pilot_command_controller.start()
        self.co_trigger_back = self.co_pilot_command_controller.back()

    def configure_swerve_bindings(self):
        #self.trigger_a.debounce(0.05).onTrue(DriveSwerveAutoVelocity(container=self, drive=self.drive, velocity=0.25,
        #direction = 'forwards', decide_by_turret = False).withTimeout(0.5))
        self.trigger_b.debounce(0.05).onTrue(GyroReset(self, swerve=self.drive))
        # self.trigger_y.whileTrue(DriveSwervePointTrajectory(container=self,drive=self.drive,pointlist=None,velocity=None,acceleration=None))


    def bind_driver_buttons(self):
        # bind driver buttons not related to swerve
        # self.trigger_a.onTrue(commands2.ConditionalCommand(
        #                         onTrue=IntakeToggle(container=self, intake=self.intake, force='on'),
        #                         onFalse=IntakeToggle(container=self,intake=self.intake, force='off'),
        #                         condition=lambda: math.degrees(self.crank_arm.get_angle()) < 70))
        self.trigger_a.onTrue(AcquireNoteToggle(container=self))
        # self.trigger_a.onTrue(commands2.ConditionalCommand(
        #                         onTrue=AcquireNoteToggle(container=self, force='on'),
        #                         onFalse=AcquireNoteToggle(container=self, force='off'),
        #                         condition=lambda: math.degrees(self.crank_arm.get_angle()) < 70))
        self.trigger_u.onTrue(ToggleClimbServos(self, self.climber))
        self.trigger_d.debounce(0.05).whileTrue(RunClimber(container=self, climber=self.climber, left_volts=3, right_volts=3))
        self.trigger_l.debounce(0.05).whileTrue(RunClimber(container=self, climber=self.climber, left_volts=3, right_volts=0))
        self.trigger_r.debounce(0.05).whileTrue(RunClimber(container=self, climber=self.climber, left_volts=0, right_volts=3))
        self.trigger_start.whileTrue(CrankArmCoast(container=self, crank_arm=self.crank_arm))


        # amp_pose = constants.k_blue_amp if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue else constants.k_red_amp
        # speaker_pose = constants.k_blue_speaker if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue else constants.k_red_speaker
        amp_pose = constants.k_blue_amp
        speaker_pose = constants.k_blue_speaker
        self.trigger_y.whileTrue(PathPlannerConfiguration.on_the_fly_path(self.drive, {"x": amp_pose[0], "y": amp_pose[1], "rotation": amp_pose[2]}, 0, speed_factor=0.25, fast_turn=True))
                                 # .andThen(ArmSmartGoTo(container=self, desired_position='amp', wait_for_finish=True))).toggleOnFalse(ArmSmartGoTo(container=self, desired_position='intake'))
        self.trigger_x.whileTrue(PathPlannerConfiguration.on_the_fly_path(self.drive, {"x": speaker_pose[0], "y": speaker_pose[1], "rotation": speaker_pose[2]}, 0, speed_factor=0.25, fast_turn=True))
                                 # .andThen(ArmSmartGoTo(container=self, desired_position='shoot', wait_for_finish=True))).toggleOnFalse(ArmSmartGoTo(container=self, desired_position='intake'))
        # self.trigger_x.whileTrue(AutomatedPath(self, self.drive, {"x": speaker_pose[0], "y": speaker_pose[1], "rotation": speaker_pose[2]}, final_velocity=0, speed_factor=0.5, fast_turn=True))
       
        if wpilib.RobotBase.isReal():
            pass
        #     self.trigger_start.onTrue(RecordAuto(self, "/home/lvuser/input_log.json"))
        # else:
        #     self.trigger_start.onTrue(RecordAuto(self, 'input_log.json'))

    def bind_copilot_buttons(self):
        # bind shooter - forcing 'off' and 'on' ignores the rpm parameter - for now, anyway
        # self.co_trigger_a.onTrue(ShooterToggle(container=self, shooter=self.shooter, rpm=None, force='on'))
        # self.co_trigger_b.onTrue(ShooterToggle(container=self, shooter=self.shooter, force='off'))

        self.co_trigger_left_stick_y.whileTrue(MoveArmByPose(self))
        self.co_trigger_left_stick_y.whileTrue(DriveAndAutoAimChassis(self, self.drive, field_oriented=constants.k_field_centric, rate_limited=constants.k_rate_limited))

        self.co_trigger_a.onTrue(ArmSmartGoTo(container=self, desired_position='low_shoot'))
        self.co_trigger_b.onTrue(ArmSmartGoTo(container=self, desired_position='low_amp'))
        self.co_trigger_x.onTrue(ArmSmartGoTo(container=self, desired_position='intake'))
        self.co_trigger_y.onTrue(ArmSmartGoTo(container=self, desired_position='shoot'))
        # self.co_trigger_y.onTrue(LedToggle(container=self))
        # self.co_trigger_y.onTrue(IntakeToggle(container=self, intake=self.intake, force='on'))

        self.co_trigger_lb.onTrue(AcquireNoteToggle(container=self, force='off'))
        self.co_trigger_lb.onTrue(self.led.set_indicator_with_timeout(Led.Indicator.KILL, 5))
        self.co_trigger_rb.onTrue(AutoShootCycle(container=self))

        self.co_trigger_l_trigger.whileTrue(EjectAll(self, self.intake, self.indexer, self.shooter, self.co_pilot_command_controller))
        self.co_trigger_r_trigger.onTrue(LedToggle(container=self))

        #bind crank arm
        setpoints = False
        if setpoints:
            self.co_trigger_r.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=5, direction='up'))
            self.co_trigger_l.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=-5, direction='down'))
            self.co_trigger_u.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=20, direction='up'))
            self.co_trigger_d.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=-20, direction='down'))
        else:
            direction = None
            self.co_trigger_r.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=5, direction=direction)) # was 15 and -15
            self.co_trigger_l.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=-5, direction=direction))
            self.co_trigger_u.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=5, direction=direction)) # was 10 and -10 lhack testing 3/12/24
            self.co_trigger_d.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=-5, direction=direction))

        self.co_trigger_start.whileTrue(CalibrateLowerCrankByLimitSwitch(container=self, lower_crank=self.crank_arm, led=self.led))

        # self.container.led.set_indicator_with_timeout(Led.Indicator.CLIMB, 5)
        self.co_trigger_back.onTrue(AutoClimbArm(self))


        # self.co_trigger_y.whileTrue(CrankArmCoast(container=self, crank_arm=self.crank_arm))
        # self.co_trigger_x.whileTrue(CrankArmCoast(container=self, crank_arm=self.shooter_arm))

        # bind intake
        # self.co_trigger_y.onTrue(IntakeToggle(container=self, intake=self.intake, force='on'))

        # bind indexer
        # Left bumper kills everything
        # Right bumper shoot cycle
        # X move to intake position
        # A move to shoot position
        # B move to amp positon
        # Y activate intake
        # use that trigger command to call something like intake drive by trigger if the trigger is greater than sometbing

        # bind LED

    def set_automated_path(self, command : PathPlannerPath):
        self.autonomous_command = command

    def get_automated_path(self) -> PathPlannerPath:
        if self.autonomous_command is not None:
            return self.autonomous_command
        return None

    def registerCommands(self):
        print("!! Registering commands !!")
        NamedCommands.registerCommand('Go to shoot', GoToShoot(self))
        NamedCommands.registerCommand('Go to intake', GoToIntake(self))
        NamedCommands.registerCommand('Go to amp', GoToAmp(self))
        NamedCommands.registerCommand('Move arm by pose', MoveArmByPose(self))

        NamedCommands.registerCommand('Move crank to shoot position', ArmMove(container=self, arm=self.crank_arm, degrees=constants.k_crank_presets['shoot']['lower'],
                                                                              absolute=True, wait_to_finish=True))
        NamedCommands.registerCommand('Move shooter to shoot position', ArmMove(container=self,arm=self.shooter_arm, degrees=constants.k_crank_presets['shoot']['upper'],
                                                                                absolute=True, wait_to_finish=True))

        NamedCommands.registerCommand('Auto shoot cycle', AutoShootCycle(self, go_to_intake=False))
        NamedCommands.registerCommand('Acquire note toggle', AcquireNoteToggle(self))

    def get_arm_mode(self):
        return self.arm_mode

    def initialize_dashboard(self):
        PathPlannerMaker = PathPlannerConfiguration()

        # populate autonomous routines
        self.autonomous_chooser = wpilib.SendableChooser()
        # Manually add our own
        self.autonomous_chooser.setDefaultOption('Shoot and drive out', ShootAndDriveOut(container=self, lower_arm=self.crank_arm, upper_arm=self.shooter_arm, drive=self.drive))
        self.autonomous_chooser.addOption('Aim and shoot', AimAndShoot(container=self, lower_arm=self.crank_arm, upper_arm=self.shooter_arm))
        self.autonomous_chooser.addOption('Drive wait', DriveWait(self, 2))
        if wpilib.RobotBase.isReal():
            self.autonomous_chooser.addOption('Playback auto', PlaybackAuto(self, "/home/lvuser/input_log.json"))
        else:
            self.autonomous_chooser.addOption('Playback auto: Sim edition', PlaybackAuto(self, 'input_log.json'))

        # Automatically get Pathplanner paths

        try_path_planner = True
        if try_path_planner:
            PathPlannerMaker.configure_paths(self.autonomous_chooser)

        wpilib.SmartDashboard.putData('autonomous routines', self.autonomous_chooser)

        self.position_chooser = wpilib.SendableChooser()
        wpilib.SmartDashboard.putData('Position Chooser', self.position_chooser)
        # self.autonomous_chooser.addOption("To Blue Amp from anywhere", PathPlannerConfiguration.on_the_fly_path(self.drive, {"x": constants.k_blue_amp[0]-self.drive.get_pose().X(), "y": constants.k_blue_amp[1]-self.drive.get_pose().Y(), "rotation": constants.k_blue_amp[2]-self.drive.get_angle()}, 0, speed_factor=0.5))
        self.autonomous_chooser.addOption("To Blue Amp from anywhere", PathPlannerConfigurationCommand(self, self.drive, {"x": constants.k_blue_amp[0], "y": constants.k_blue_amp[1], "rotation": constants.k_blue_amp[2]}, 0, speed_factor=0.5, fast_turn=True))

        # put commands that we want to call from the dashboard - IDE has problems w/ CommandBase vs Command
        wpilib.SmartDashboard.putData('GyroReset', GyroReset(self, swerve=self.drive))
        wpilib.SmartDashboard.putData('UpperCrankMoveUp', ArmMove(container=self, arm=self.shooter_arm, degrees=10, direction=None))
        wpilib.SmartDashboard.putData('UpperCrankMoveDown', ArmMove(container=self, arm=self.shooter_arm, degrees=-10, direction=None))
        wpilib.SmartDashboard.putData('LowerCrankMoveUp', ArmMove(container=self, arm=self.crank_arm, degrees=10, direction=None))
        wpilib.SmartDashboard.putData('LowerCrankMoveDown', ArmMove(container=self, arm=self.crank_arm, degrees=-10, direction=None))
        wpilib.SmartDashboard.putData('IntakeOn', IntakeToggle(container=self, intake=self.intake, force='on'))
        wpilib.SmartDashboard.putData('IntakeOff', IntakeToggle(container=self, intake=self.intake, force='off'))
        wpilib.SmartDashboard.putData('IndexerOn', IndexerToggle(container=self, indexer=self.indexer, force='on'))
        wpilib.SmartDashboard.putData('IndexerOff', IndexerToggle(container=self, indexer=self.indexer, force='off'))
        wpilib.SmartDashboard.putData('ShooterOn', ShooterToggle(container=self, shooter=self.shooter, rpm=None, force='on'))
        wpilib.SmartDashboard.putData('ShooterOff', ShooterToggle(container=self, shooter=self.shooter, force='off'))
        wpilib.SmartDashboard.putData('AutoShootCycle', AutoShootCycle(container=self))
        # wpilib.SmartDashboard.putData('Gaslight encoders', GaslightCrankEncoders(self, self.crank_arm))

    def get_autonomous_command(self):
        return self.autonomous_chooser.getSelected()
