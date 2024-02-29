#  Container for 2429's 2023 swerve robot with turret, elevator, arm, wrist, and manipulator

import time, enum
import wpilib
import commands2
from commands2.button import CommandXboxController

# pathplanner
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder, NamedCommands

import constants
from autonomous.pathplannermaker import PathPlannerConfiguration  # all the constants except for swerve

# subsystems
from subsystems.swerve import Swerve
from subsystems.intake import Intake
from subsystems.led import Led
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from subsystems.climber import Climber

# auto
from autonomous.auto_shoot_cycle import AutoShootCycle
from autonomous.playback_auto import PlaybackAuto
from autonomous.aim_and_shoot import AimAndShoot
from autonomous.shoot_and_drive_out import ShootAndDriveOut

# commands
from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from commands.gyro_reset import GyroReset
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from autonomous.drive_swerve_point_trajectory import DriveSwervePointTrajectory
from commands.led_loop import LedLoop
from commands.led_toggle import LedToggle
from commands.intake_toggle import IntakeToggle
from commands.arm_coast import CrankArmCoast
from commands.arm_move import ArmMove
from commands.arm_cycle import ArmCycle
from commands.arm_joystick_control import ArmJoystickControl
from commands.indexer_by_joystick import IndexerByJoystick
from commands.indexer_toggle import IndexerToggle
from commands.shooter_toggle import ShooterToggle
# from commands.climber_toggle import ClimberToggle
from commands.record_auto import RecordAuto
import os

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
            #self.climber = Climber()

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

        arm_mode = 'intake'
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
        self.trigger_y = self.driver_command_controller.y()
        self.trigger_rb = self.driver_command_controller.rightBumper()
        self.trigger_start = self.driver_command_controller.start()

    def configure_copilot_joystick(self):
        self.co_pilot_command_controller = CommandXboxController(constants.k_co_pilot_controller_port)  # 2024 way
        self.co_trigger_a = self.co_pilot_command_controller.a()  # 2024 way
        self.co_trigger_b = self.co_pilot_command_controller.b()
        self.co_trigger_y = self.co_pilot_command_controller.y()
        self.co_trigger_x = self.co_pilot_command_controller.x()
        self.co_trigger_rb = self.co_pilot_command_controller.rightBumper()
        self.co_trigger_r = self.co_pilot_command_controller.povRight()
        self.co_trigger_l = self.co_pilot_command_controller.povLeft()
        self.co_trigger_u = self.co_pilot_command_controller.povUp()
        self.co_trigger_d = self.co_pilot_command_controller.povDown()

    def configure_swerve_bindings(self):
        self.trigger_a.debounce(0.05).onTrue(DriveSwerveAutoVelocity(container=self, drive=self.drive, velocity=0.25,
        direction = 'forwards', decide_by_turret = False).withTimeout(0.5))
        self.trigger_b.debounce(0.05).onTrue(GyroReset(self, swerve=self.drive))
        # self.trigger_y.whileTrue(DriveSwervePointTrajectory(container=self,drive=self.drive,pointlist=None,velocity=None,acceleration=None))


    def bind_driver_buttons(self):
        # bind driver buttons not related to swerve
        self.trigger_rb.onTrue(IntakeToggle(container=self, intake=self.intake))
        if wpilib.RobotBase.isReal():
            self.trigger_start.onTrue(RecordAuto(self, "/home/lvuser/input_log.json"))
        else:
            self.trigger_start.onTrue(RecordAuto(self, 'input_log.json'))

    def bind_copilot_buttons(self):
        # bind shooter - forcing 'off' and 'on' ignores the rpm parameter - for now, anyway
        # self.co_trigger_a.onTrue(ShooterToggle(container=self, shooter=self.shooter, rpm=None, force='on'))
        # self.co_trigger_b.onTrue(ShooterToggle(container=self, shooter=self.shooter, force='off'))
        self.co_trigger_a.onTrue(ArmCycle(container=self, upper_crank = self.shooter_arm, lower_crank = self.crank_arm, direction="up").withTimeout(2))
        self.co_trigger_b.onTrue(ArmCycle(container=self, upper_crank = self.shooter_arm, lower_crank = self.crank_arm, direction="down").withTimeout(2))
        # self.co_trigger_b.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=-5, direction='down'))

        #bind Auto Shoot Cycle
        self.co_trigger_rb.onTrue(AutoShootCycle(container=self))

         #bind crank arm
        setpoints = False
        if setpoints:
            self.co_trigger_r.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=10, direction='up'))
            self.co_trigger_l.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=-10, direction='down'))
            self.co_trigger_u.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=20, direction='up'))
            self.co_trigger_d.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=-20, direction='down'))
        else:
            direction = None
            self.co_trigger_r.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=15, direction=direction))
            self.co_trigger_l.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=-15, direction=direction))
            self.co_trigger_u.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=10, direction=direction))
            self.co_trigger_d.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=-10, direction=direction))

        # self.co_trigger_y.whileTrue(CrankArmCoast(container=self, crank_arm=self.crank_arm))
        # self.co_trigger_x.whileTrue(CrankArmCoast(container=self, crank_arm=self.shooter_arm))

        # bind intake
        # self.co_trigger_y.onTrue(IntakeToggle(container=self, intake=self.intake, rpm=2500, force='on'))

        # bind indexer
        self.co_trigger_y.onTrue(IntakeToggle(container=self, intake=self.intake, rpm=1000, force='on'))
        self.co_trigger_x.onTrue(IntakeToggle(container=self, intake=self.intake, rpm=1000, force='off'))

        # bind LED
        #  self.co_trigger_a.onTrue(LedToggle(container=self))

        # bind climber
        # self.co_trigger_x.onTrue(ClimberToggle(container=self, climber=self.climber, rpm=2500, force='on'))
        # self.co_trigger_y.onTrue(ClimberToggle(container=self, climber=self.climber, force='off'))

    def registerCommands(self):
        print("!! Registering commands !!")
        NamedCommands.registerCommand('Wait 1 second', DriveWait(self, 1))
        NamedCommands.registerCommand('Move shooter to next setpoint', ArmMove(container=self, arm=self.shooter_arm, direction='up'))
        NamedCommands.registerCommand('Toggle intake', IntakeToggle(self, self.intake))

    def initialize_dashboard(self):
        PathPlannerMaker = PathPlannerConfiguration()

        # populate autonomous routines
        self.autonomous_chooser = wpilib.SendableChooser()
        # Manually add our own
        self.autonomous_chooser.addOption('Aim and shoot', AimAndShoot(container=self, lower_arm=self.crank_arm, upper_arm=self.shooter_arm))
        self.autonomous_chooser.addOption('Shoot and drive out', ShootAndDriveOut(self, self.crank_arm, self.shooter_arm, self.drive))
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
        self.position_chooser.setDefaultOption("0, 0, 0", {"x": 0, "y": 0, "rotation": 0}) #Note: (0,0) is the bottom left corner of the field.
        self.position_chooser.addOption("0, 2, 0", {"x": 0, "y": 2, "rotation": 0})
        self.position_chooser.addOption("2, 0, 0", {"x": 2, "y": 0, "rotation": 0})
        self.position_chooser.addOption("2, 2, 0", {"x": 2, "y": 2, "rotation": 90})        
        self.autonomous_chooser.addOption("manual option", PathPlannerConfiguration.configure_path_manual(self.position_chooser.getSelected(), 0, 0).withTimeout(5))

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

    def get_autonomous_command(self):
        return self.autonomous_chooser.getSelected()
