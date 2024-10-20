#  Container for 2429's 2023 swerve robot with turret, elevator, arm, wrist, and manipulator
import math
import time

import commands2
import wpilib
from commands2.button import CommandXboxController
from commands2.button import CommandJoystick
from ntcore import NetworkTableInstance

# pathplanner
from pathplannerlib.auto import NamedCommands, PathPlannerPath

import constants
from autonomous.pathplannermaker import PathPlannerConfiguration
from autonomous.pathplannermakercommand import PathPlannerConfigurationCommand

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
from autonomous.auto_commands import *  # CJH STRONGLY DISAPPROVES OF THIS SLOPPINESS!
from autonomous.auto_climb_giselle import AutoClimbGiselle
from autonomous.drive_wait import DriveWait
from autonomous.auto_climb_sanjith import AutoClimbSanjith
from autonomous.auto_drive_to_tag import AutoDriveToTag
from autonomous.auto_drive_to_note import AutoDriveToNote

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
from commands.eject_all import EjectAll
from commands.calibrate_lower_crank_by_limit_switch import CalibrateLowerCrankByLimitSwitch
from commands.arm_coast import CrankArmCoast
from commands.move_arm_by_pose import MoveArmByPose
from commands.drive_and_auto_aim_chassis import DriveAndAutoAimChassis
from commands.smart_intake import SmartIntake
from commands.system_interrupt import SystemInterrupt
from commands.can_status import CANStatus
from autonomous.auto_lob_cycle import AutoLobCycle


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        self.start_time = time.time()

        # set the default arm position - will use this to control shot speed.  Need this before the arm subsystems...
        self.arm_configuration = 'unknown'
        wpilib.SmartDashboard.putString('arm_config', self.arm_configuration)

        #robot is shooting backwards for auto aim by default
        self.shooting_backwards = True

        # The robot's subsystems
        self.drive = Swerve()

        if not constants.k_swerve_only:
            self.intake = Intake()
            self.led = Led(self)
            self.crank_arm = LowerCrankArmTrapezoidal()
            self.shooter_arm = UpperCrankArmTrapezoidal()
            self.indexer = Indexer()
            self.shooter = Shooter()
            self.climber = Climber()
            self.vision = Vision()

        # set up driving
        self.keys_pressed_entry = NetworkTableInstance.getDefault().getEntry("SmartDashboard/keys_pressed")  # for operator control via keyboard
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
            if constants.k_enable_button_box:
                self.configure_button_box()
                self.bind_button_box()

        # this has to be before initialize_dashboard but after configuring controls 3/22/24 LHACK
        self.registerCommands()

        self.initialize_dashboard()

    def set_start_time(self):  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def get_enabled_time(self):  # call when we want to know the start/elapsed time for status and debug messages
        return time.time() - self.start_time

    def configure_driver_joystick(self):
        # The driver's controller
        self.driver_command_controller = CommandXboxController(constants.k_driver_controller_port)  # 2024 way
        self.trigger_a= self.driver_command_controller.a()  # 2024 way
        self.trigger_b = self.driver_command_controller.b()
        self.trigger_x = self.driver_command_controller.x()
        self.trigger_y = self.driver_command_controller.y()
        self.trigger_rb = self.driver_command_controller.rightBumper()
        self.trigger_lb = self.driver_command_controller.leftBumper()
        self.trigger_start = self.driver_command_controller.start()
        self.trigger_back = self.driver_command_controller.back()
        self.trigger_d = self.driver_command_controller.povDown()
        self.trigger_u = self.driver_command_controller.povUp()
        self.trigger_r = self.driver_command_controller.povRight()
        self.trigger_l = self.driver_command_controller.povLeft()
        self.trigger_l_trigger = self.driver_command_controller.leftTrigger(0.2)
        self.trigger_r_trigger = self.driver_command_controller.rightTrigger(0.2) # This is used as a gas pedal in drive_by_joystick_swerve

        # BUTTONS THAT WORK ONLY WITHOUT THE LB
        self.trigger_only_a = self.trigger_a.and_(self.trigger_lb.not_())
        self.trigger_only_b = self.trigger_b.and_(self.trigger_lb.not_())
        self.trigger_only_x = self.trigger_x.and_(self.trigger_lb.not_())
        self.trigger_only_y = self.trigger_y.and_(self.trigger_lb.not_())

        # SHIFT BUTTONS THAT WORK ONLY WITH THE LB
        self.trigger_shift_a = self.trigger_a.and_(self.trigger_lb)
        self.trigger_shift_b = self.trigger_b.and_(self.trigger_lb)
        self.trigger_shift_x = self.trigger_x.and_(self.trigger_lb)
        self.trigger_shift_y = self.trigger_y.and_(self.trigger_lb)


    def configure_copilot_joystick(self):

        self.co_pilot_command_controller = CommandXboxController(constants.k_co_pilot_controller_port)  # 2024 way
        self.co_trigger_left_stick_y = self.co_pilot_command_controller.axisGreaterThan(axis=1, threshold=0.5)
        self.co_trigger_a_controller = self.co_pilot_command_controller.a()  # 2024 way
        self.co_trigger_b_controller = self.co_pilot_command_controller.b()
        self.co_trigger_y_controller = self.co_pilot_command_controller.y()
        self.co_trigger_x_controller = self.co_pilot_command_controller.x()
        self.co_trigger_rb_controller = self.co_pilot_command_controller.rightBumper()
        self.co_trigger_lb_controller = self.co_pilot_command_controller.leftBumper()
        self.co_trigger_r_controller = self.co_pilot_command_controller.povRight()
        self.co_trigger_l_controller = self.co_pilot_command_controller.povLeft()
        self.co_trigger_u_controller = self.co_pilot_command_controller.povUp()
        self.co_trigger_d_controller = self.co_pilot_command_controller.povDown()
        self.co_trigger_l_trigger_controller = self.co_pilot_command_controller.leftTrigger(0.2)
        self.co_trigger_r_trigger_controller = self.co_pilot_command_controller.rightTrigger(0.2)
        self.co_trigger_start = self.co_pilot_command_controller.start()
        self.co_trigger_back = self.co_pilot_command_controller.back()

        self.co_trigger_s = commands2.button.Trigger(lambda: 83 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_S_capitalized = commands2.button.Trigger(lambda: 83 in self.keys_pressed_entry.getIntegerArray([])
                                                                         and 16777248 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_a = commands2.button.Trigger(lambda: 65 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_A_capitalized = commands2.button.Trigger(lambda: 65 in self.keys_pressed_entry.getIntegerArray([])
                                                                         and 16777248 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_i = commands2.button.Trigger(lambda: 73 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_I_capitalized = commands2.button.Trigger(lambda: 73 in self.keys_pressed_entry.getIntegerArray([])
                                                                         and 16777248 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_l = commands2.button.Trigger(lambda: 76 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_L_capitalized = commands2.button.Trigger(lambda: 76 in self.keys_pressed_entry.getIntegerArray([])
                                                                         and 16777248 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_k = commands2.button.Trigger(lambda: 75 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_tab = commands2.button.Trigger(lambda: 16777217 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_e = commands2.button.Trigger(lambda: 69 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_b = commands2.button.Trigger(lambda: 66 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_equals = commands2.button.Trigger(lambda: 61 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_minus = commands2.button.Trigger(lambda: 45 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_plus = commands2.button.Trigger(lambda: 43 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_underscore = commands2.button.Trigger(lambda: 95 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_6 = commands2.button.Trigger(lambda: 54 in self.keys_pressed_entry.getIntegerArray([]))
        self.co_trigger_g = commands2.button.Trigger(lambda: 71 in self.keys_pressed_entry.getIntegerArray([]))

        # armsmartgoto low shoot s
        # " amp a
        # " intake i
        # " lob l
        # kill indexer k
        # fire [TAB]
        # eject e
        # ledtoggle b
        # fine tune cranks positions:
            # lower up =
            # lower down -
            # upper up +
            # upper down_
        # calibrate by limit switch (no longer needed i think) 6
        # auto climb giselle g

    def configure_button_box(self):
        # The driver's controller
        self.button_box = CommandJoystick(constants.k_button_box_port)  # 2024 way
        # this should probably be a list completion
        self.button_1 = self.button_box.button(1)
        self.button_2 = self.button_box.button(2)
        self.button_3 = self.button_box.button(3)
        self.button_4 = self.button_box.button(4)
        self.button_5 = self.button_box.button(5)
        self.button_6 = self.button_box.button(6)
        self.button_7 = self.button_box.button(7)
        self.button_8 = self.button_box.button(8)
        self.button_9 = self.button_box.button(9)
        self.button_10 = self.button_box.button(10)

    def bind_button_box(self):

        self.button_1.onTrue(AcquireNoteToggle(container=self, force='off'))  # kill
        self.button_6.onTrue(AutoShootCycle(container=self))  # shoot

        self.button_2.onTrue(LedToggle(container=self))
        self.button_7.onTrue(LedToggle(container=self))

        self.button_3.onTrue(LedToggle(container=self))
        self.button_8.onTrue(LedToggle(container=self))

        self.button_4.onTrue(LedToggle(container=self))
        self.button_9.onTrue(LedToggle(container=self))

        self.button_5.whileTrue(AutoDriveToTag(container=self, drive=self.drive, destination='amp'))
        self.button_10.whileTrue(AutoDriveToTag(container=self, drive=self.drive, destination='speaker'))

    def configure_swerve_bindings(self):
        self.trigger_only_b.debounce(0.05).onTrue(GyroReset(self, swerve=self.drive))
        # self.trigger_y.whileTrue(DriveSwervePointTrajectory(container=self,drive=self.drive,pointlist=None,velocity=None,acceleration=None))

    def bind_driver_buttons(self):
        PathPlannerMaker = PathPlannerConfiguration()
        # bind driver buttons not related to swerve

        self.trigger_only_a.onTrue(GoToIntake(container=self).andThen(
                SmartIntake(container=self, wait_to_finish=True).andThen(
                    GoToShoot(self)
            )))  # force intake on, set LEDs orange, wait for note
        self.trigger_shift_a.onTrue(AcquireNoteToggle(container=self, force='on'))   # old version - does not go to intake in case they get in trouble
        # TRIGGER B BOUND IN SWERVE SECTION

        # AUTO AIMING
        self.trigger_only_x.debounce(0.05).whileTrue(MoveArmByPose(container=self, shooting_backwards=True))
        self.trigger_only_x.debounce(0.05).whileTrue(DriveAndAutoAimChassis(container=self, swerve=self.drive, field_oriented=constants.k_field_centric, rate_limited=constants.k_rate_limited, shooting_backwards=True))

        self.trigger_shift_x.debounce(0.05).whileTrue(MoveArmByPose(container=self, shooting_backwards=False))
        self.trigger_shift_x.debounce(0.05).whileTrue(DriveAndAutoAimChassis(container=self, swerve=self.drive, field_oriented=constants.k_field_centric, rate_limited=constants.k_rate_limited, shooting_backwards=False))

        self.trigger_only_y.whileTrue(AutoDriveToTag(container=self, drive=self.drive, destination='amp'))
        self.trigger_shift_y.whileTrue(AutoDriveToTag(container=self, drive=self.drive, destination='speaker'))

        self.trigger_l_trigger.onTrue(GoToIntake(container=self).andThen(
                SmartIntake(self, wait_to_finish=True, timeout=10).andThen(
                    GoToShoot(self)
        )))
        self.trigger_l_trigger.whileTrue(
            AutoDriveToNote(self)
        )

        # WE SHOULD NOT BIND LB.  IT IS USED AS ROBOT-CENTRIC IN DRIVE AND AS A SHIFT BUTTON ON OTHER COMMANDS
        self.trigger_rb.debounce(0.05).whileTrue(commands2.ParallelCommandGroup(
            DriveAndAutoAimChassis(container=self, swerve=self.drive, field_oriented=constants.k_field_centric,
                                   rate_limited=constants.k_rate_limited, shooting_backwards=True, aim_target="corner"),
            GoToShoot(self)
        ))
        self.trigger_rb.debounce(0.05).onFalse(AutoLobCycle(self))


        # DPAD
        # self.trigger_u.onTrue(ToggleClimbServos(self, climber=self.climber, force=None))
        climber_voltage = 4  # was 3 until Tempe
        # self.trigger_d.debounce(0.05).whileTrue(AutoClimbSanjith(container=self, climber=self.climber, left_volts=climber_voltage, right_volts=climber_voltage))
        # self.trigger_l.debounce(0.05).whileTrue(RunClimber(container=self, climber=self.climber, left_volts=climber_voltage, right_volts=0))
        # self.trigger_r.debounce(0.05).whileTrue(RunClimber(container=self, climber=self.climber, left_volts=0, right_volts=climber_voltage))

        self.trigger_start.whileTrue(CrankArmCoast(container=self, crank_arm=self.crank_arm))

        # SHIFT COMMANDS - using left bumper as a button multiplier
        #self.trigger_shift_a.onTrue(commands2.PrintCommand('You pressed A and LB - and nothing else happened'))
        self.trigger_shift_b.onTrue(commands2.PrintCommand('You pressed B and LB - and nothing else happened'))
        # self.trigger_shift_x.onTrue(commands2.PrintCommand('You pressed X and LB - and nothing else happened'))
        # self.trigger_shift_y.onTrue(commands2.PrintCommand('You pressed Y and LB - and nothing else happened'))

    def bind_copilot_buttons(self):

        # ---------------- CONTROLLER CONTROLS ---------------------

        # arm positions
        self.co_trigger_a_controller.onTrue(ArmSmartGoTo(container=self, desired_position='low_shoot'))
        self.co_trigger_y_controller.onTrue(ArmSmartGoTo(container=self, desired_position='amp'))
        self.co_trigger_x_controller.onTrue(ArmSmartGoTo(container=self, desired_position='intake'))
        self.co_trigger_b_controller.onTrue(AutoLobCycle(container=self))  # shoot
        # self.co_trigger_y.onTrue(LedToggle(container=self))

        # intake / shoot control
        self.co_trigger_lb_controller.onTrue(AcquireNoteToggle(container=self, force='off'))  # kill
        self.co_trigger_rb_controller.onTrue(AutoShootCycle(container=self))  # shoot
        self.co_trigger_l_trigger_controller.whileTrue(EjectAll(self, self.intake, self.indexer, self.shooter, self.co_pilot_command_controller))
        self.co_trigger_r_trigger_controller.onTrue(LedToggle(container=self))

        # bind crank arm  - TODO - figure out how to do a double-tap to make things go faster
        direction = None
        self.co_trigger_r_controller.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=8, direction=direction)) # was 15 and -15
        self.co_trigger_l_controller.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=-8, direction=direction))
        self.co_trigger_u_controller.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=5, direction=direction)) # was 10 and -10 lhack testing 3/12/24
        self.co_trigger_d_controller.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=-5, direction=direction))

        self.co_trigger_start.whileTrue(CalibrateLowerCrankByLimitSwitch(container=self, lower_crank=self.crank_arm, led=self.led))
        self.co_trigger_back.onTrue(AutoClimbGiselle(self))

        # ------------------- KEYBOARD CONTROLS ---------------------

        # arm positions
        self.co_trigger_s.onTrue(ArmSmartGoTo(container=self, desired_position='low_shoot'))
        self.co_trigger_S_capitalized.onTrue(AutoShootCycle(container=self))
        self.co_trigger_a.onTrue(ArmSmartGoTo(container=self, desired_position='amp'))
        self.co_trigger_A_capitalized.onTrue(AutoShootCycle(container=self))
        self.co_trigger_i.onTrue(ArmSmartGoTo(container=self, desired_position='intake'))
        self.co_trigger_I_capitalized.onTrue(AutoShootCycle(container=self))
        self.co_trigger_l.onTrue(ArmSmartGoTo(container=self, desired_position='low_shoot'))
        self.co_trigger_L_capitalized.onTrue(AutoLobCycle(container=self))  # shoot

        # self.co_trigger_y.onTrue(LedToggle(container=self))

        # intake / shoot control
        self.co_trigger_k.onTrue(AcquireNoteToggle(container=self, force='off'))  # kill

        self.co_trigger_e.whileTrue(EjectAll(self, self.intake, self.indexer, self.shooter))

        self.co_trigger_b.onTrue(LedToggle(container=self))

        # bind crank arm  - TODO - figure out how to do a double-tap to make things go faster
        direction = None
        self.co_trigger_equals.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=8, direction=direction)) # was 15 and -15
        self.co_trigger_minus.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=-8, direction=direction))
        self.co_trigger_plus.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=5, direction=direction)) # was 10 and -10 lhack testing 3/12/24
        self.co_trigger_underscore.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=-5, direction=direction))

        self.co_trigger_6.whileTrue(CalibrateLowerCrankByLimitSwitch(container=self, lower_crank=self.crank_arm, led=self.led))
        self.co_trigger_g.onTrue(AutoClimbGiselle(self))

        # armsmartgoto low shoot s
        # " amp a
        # " intake i
        # " lob l
        # kill indexer k
        # fire [TAB]
        # eject e
        # ledtoggle b
        # fine tune cranks positions:
        # lower up =
        # lower down -
        # upper up +
        # upper down_
        # calibrate by limit switch (no longer needed i think) 6
        # auto climb giselle g


    def set_automated_path(self, command : PathPlannerPath):
        self.autonomous_command = command

    def get_automated_path(self) -> PathPlannerPath:
        if self.autonomous_command is not None:
            return self.autonomous_command
        return None

    def registerCommands(self):
        for near_far in ['near', 'far']:
            if near_far == 'near':
                for near_ring_num in range(1, 4):
                    NamedCommands.registerCommand(f'Intake {near_far} ring {near_ring_num}', GetRing(self, near_ring_num, near_far))
            # Commented because we don't have paths for all far rings; if we have paths for some far rings, do for far_ring_num in [1, 5] for example
            else:
                for far_ring_num in range(1, 6):
                    NamedCommands.registerCommand(f'Intake {near_far} ring {far_ring_num}', GetRing(self, far_ring_num, near_far))

        for speaker_side in ['ampside', 'middle', 'sourceside']:
            NamedCommands.registerCommand(f'Go to {speaker_side} and shoot', GoToSpeakerAndShoot(self, speaker_side))

        NamedCommands.registerCommand('Shoot preload', ShootPreload(self, 5))
        NamedCommands.registerCommand('Go to shoot', GoToShoot(self))
        NamedCommands.registerCommand('Go to intake', GoToIntake(self))
        NamedCommands.registerCommand('Go to amp', GoToAmp(self))
        NamedCommands.registerCommand('Move arm by pose', MoveArmByPose(container=self, shooting_backwards=True))
        NamedCommands.registerCommand('Move arm by pose forward', MoveArmByPose(container=self, shooting_backwards=False))

        NamedCommands.registerCommand('Auto aim robot chassis towards speaker', DriveAndAutoAimChassis(container=self, swerve=self.drive, velocity_multiplier=1, shooting_backwards=True))
        NamedCommands.registerCommand('Auto aim front of robot chassis towards speaker', DriveAndAutoAimChassis(self, self.drive, constants.k_field_centric, constants.k_rate_limited, shooting_backwards=False))

        NamedCommands.registerCommand('Move crank to shoot position', ArmMove(container=self, arm=self.crank_arm, degrees=constants.k_crank_presets['shoot']['lower'],
                                                                              absolute=True, wait_to_finish=True))
        NamedCommands.registerCommand('Move shooter to shoot position', ArmMove(container=self,arm=self.shooter_arm, degrees=constants.k_crank_presets['shoot']['upper'],
                                                                                absolute=True, wait_to_finish=True))
        NamedCommands.registerCommand('Auto shoot cycle', AutoShootCycle(self, go_to_shoot=False))
        NamedCommands.registerCommand('Acquire note toggle', AcquireNoteToggle(self))
        NamedCommands.registerCommand('Switch Shooting Direction for Auto Aim', ChangeShootingDirection(container=self))
        NamedCommands.registerCommand('Shoot from anywhere', ShootFromAnywhere(self))
        NamedCommands.registerCommand('Smart intake on', SmartIntake(self, auto=True, timeout=6, wait_to_finish=True))
        NamedCommands.registerCommand('Spin up flywheels', ShooterToggle(container=self, shooter=self.shooter, force='on', rpm=3200, amp_rpm=3000, # usually rpm=3500, amp_rpm=2000; changed amp_rpm from 1500 to 3000 for LCE presentation 081524
                                       auto_amp_slowdown=True, wait_for_spinup=True).withTimeout(0.66))
        NamedCommands.registerCommand('Stop flywheels', ShooterToggle(container=self, shooter=self.shooter, force='off'))
        NamedCommands.registerCommand('Pull ring back from shooter', IndexerToggle(container=self, indexer=self.indexer, power=-0.33, force='on', timeout=0.1)
                                                                            .andThen(IndexerToggle(container=self, indexer=self.indexer, power=0, force='off', timeout=None)))
        NamedCommands.registerCommand('Put ring into shooter', IndexerToggle(container=self, indexer=self.indexer, power=1, force='on', timeout=0.65)
                                                                            .andThen(IndexerToggle(container=self, indexer=self.indexer, power=0, force='off', timeout=None)))



    def get_arm_configuration(self):
        return self.arm_configuration
    def set_arm_configuration(self, configuration):
        self.arm_configuration = configuration  # shoot, intake, amp, trap, etc

    def initialize_dashboard(self):
        PathPlannerMaker = PathPlannerConfiguration()

        # populate autonomous routines
        self.autonomous_chooser = wpilib.SendableChooser()
        # Manually add our own
        # self.autonomous_chooser.setDefaultOption('Shoot and drive out', ShootAndDriveOut(container=self, lower_arm=self.crank_arm, upper_arm=self.shooter_arm, drive=self.drive))
        # self.autonomous_chooser.addOption('Aim and shoot', AimAndShoot(container=self, lower_arm=self.crank_arm, upper_arm=self.shooter_arm))
        self.autonomous_chooser.setDefaultOption('Drive wait', DriveWait(self, 2))
        # if wpilib.RobotBase.isReal():
        #     self.autonomous_chooser.addOption('Playback auto', PlaybackAuto(self, "/home/lvuser/input_log.json"))
        # else:
        #     self.autonomous_chooser.addOption('Playback auto: Sim edition', PlaybackAuto(self, 'input_log.json'))

        # Automatically get Pathplanner paths

        try_path_planner = True
        if try_path_planner:
            PathPlannerMaker.configure_paths(self.autonomous_chooser)

        wpilib.SmartDashboard.putData('autonomous routines', self.autonomous_chooser)

        self.position_chooser = wpilib.SendableChooser()
        wpilib.SmartDashboard.putData('Position Chooser', self.position_chooser)
        # self.autonomous_chooser.addOption("To Blue Amp from anywhere", PathPlannerConfiguration.on_the_fly_path(self.drive, {"x": constants.k_blue_amp[0]-self.drive.get_pose().X(), "y": constants.k_blue_amp[1]-self.drive.get_pose().Y(), "rotation": constants.k_blue_amp[2]-self.drive.get_angle()}, 0, speed_factor=0.5))
        # self.autonomous_chooser.addOption("To Blue Amp from anywhere", PathPlannerConfigurationCommand(self, self.drive, {"x": constants.k_blue_amp[0], "y": constants.k_blue_amp[1], "rotation": constants.k_blue_amp[2]}, 0, speed_factor=0.5, fast_turn=True))

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
        wpilib.SmartDashboard.putData('MoveArmbyPose', MoveArmByPose(container=self))
        wpilib.SmartDashboard.putData('Drive and auto aim chassis', DriveAndAutoAimChassis(container=self, swerve=self.drive))
        # wpilib.SmartDashboard.putData('Gaslight encoders', GaslightCrankEncoders(self, self.crank_arm))
        wpilib.SmartDashboard.putData('ToStage', AutoDriveToTag(container=self, drive=self.drive, destination='stage'))
        wpilib.SmartDashboard.putData('ToAmp', AutoDriveToTag(container=self, drive=self.drive, destination='amp'))
        wpilib.SmartDashboard.putData('ToSpeaker', AutoDriveToTag(container=self, drive=self.drive, destination='speaker'))
        wpilib.SmartDashboard.putData('GyroFromPose', GyroReset(self, swerve=self.drive, from_pose=True))
        wpilib.SmartDashboard.putData('ChangeShootingDirection', ChangeShootingDirection(container=self))
        wpilib.SmartDashboard.putData('CANStatus', CANStatus(container=self))

        wpilib.SmartDashboard.putData(commands2.CommandScheduler.getInstance())

    def get_autonomous_command(self):
        return self.autonomous_chooser.getSelected()
