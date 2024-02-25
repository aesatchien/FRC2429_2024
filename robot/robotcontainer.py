#  Container for 2429's 2023 swerve robot with turret, elevator, arm, wrist, and manipulator

import time, enum
import wpilib
import commands2
from commands2.button import JoystickButton, POVButton
from commands2.button import CommandXboxController

# pathplanner
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from commands2.button import JoystickButton, POVButton
import constants  # all of the constants except for swerve

# subsystems
from subsystems.swerve import Swerve
from subsystems.intake import Intake
from subsystems.led import Led
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from subsystems.indexer import Indexer
from subsystems.shooter import Shooter
from subsystems.climber import Climber

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
from commands.arm_joystick_control import ArmJoystickControl
from commands.indexer_by_joystick import IndexerByJoystick
from commands.shooter_toggle import ShooterToggle
from commands.climber_toggle import ClimberToggle



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
        self.intake = Intake()
        self.led = Led()
        self.crank_arm = LowerCrankArmTrapezoidal()
        self.shooter_arm = UpperCrankArmTrapezoidal()
        self.indexer = Indexer()
        self.shooter = Shooter()
        self.climber = Climber()

        self.configure_joysticks()
        self.bind_buttons()
        self.configure_swerve_bindings()

        self.initialize_dashboard()

        # swerve driving
        self.drive.setDefaultCommand(DriveByJoystickSwerve(container=self, swerve=self.drive,
                            field_oriented=constants.k_field_centric, rate_limited=constants.k_rate_limited))

        # arm_degrees = 10 if wpilib.RobotBase.isReal() else 100
        # self.indexer.setDefaultCommand(IndexerByJoystick(container=self, indexer=self.indexer))
        #
        # self.led.setDefaultCommand(LedLoop(container=self))

    def set_start_time(self):  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def get_enabled_time(self):  # call when we want to know the start/elapsed time for status and debug messages
        return time.time() - self.start_time

    def configure_joysticks(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        # The driver's controller
        self.driver_command_controller = CommandXboxController(constants.k_driver_controller_port)  # 2024 way
        self.trigger_a = self.driver_command_controller.a()  # 2024 way
        self.trigger_b = self.driver_command_controller.b()
        self.trigger_y = self.driver_command_controller.y()
        self.trigger_rb = self.driver_command_controller.rightBumper()

        self.driver_controller = wpilib.XboxController(constants.k_driver_controller_port)  # 2023 way
        # self.buttonA = JoystickButton(self.driver_controller, 1)
        # self.buttonB = JoystickButton(self.driver_controller, 2)
        # self.buttonX = JoystickButton(self.driver_controller, 3)
        # self.buttonY = JoystickButton(self.driver_controller, 4)
        # self.buttonLB = JoystickButton(self.driver_controller, 5)
        # self.buttonRB = JoystickButton(self.driver_controller, 6)
        # self.buttonBack = JoystickButton(self.driver_controller, 7)
        # self.buttonStart = JoystickButton(self.driver_controller, 8)
        # self.buttonUp = POVButton(self.driver_controller, 0)
        # self.buttonDown = POVButton(self.driver_controller, 180)
        # self.buttonLeft = POVButton(self.driver_controller, 270)
        # self.buttonRight = POVButton(self.driver_controller, 90)
        #self.buttonLeftAxis = AxisButton(self.driver_controller, 2)
        #self.buttonRightAxis = AxisButton(self.driver_controller, 3)

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
        self.trigger_rb.onTrue(IntakeToggle(container=self, intake=self.intake,  ))

    def bind_buttons(self):
        pass
        # bind shooter - forcing 'off' and 'on' ignores the rpm parameter - for now, anyway
        self.co_trigger_a.onTrue(ShooterToggle(container=self, shooter=self.shooter, rpm=None, force='on'))
        self.co_trigger_b.onTrue(ShooterToggle(container=self, shooter=self.shooter, force='off'))

        #bind crank arm
        setpoints = False
        if setpoints:
            self.co_trigger_r.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=5, direction='up'))
            self.co_trigger_l.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=-5, direction='down'))
            self.co_trigger_u.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=15, direction='up'))
            self.co_trigger_d.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=-15, direction='down'))
        else:
            direction = None
            self.co_trigger_r.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=10, direction=direction))
            self.co_trigger_l.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=-10, direction=direction))
            self.co_trigger_u.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=5, direction=direction))
            self.co_trigger_d.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=-5, direction=direction))

        self.co_trigger_y.whileTrue(CrankArmCoast(container=self, crank_arm=self.crank_arm))
        self.co_trigger_x.whileTrue(CrankArmCoast(container=self, crank_arm=self.shooter_arm))

        # bind intake
        # self.co_trigger_y.onTrue(IntakeToggle(container=self, intake=self.intake, rpm=2500, force='on'))

        # bind LED
        #  self.co_trigger_a.onTrue(LedToggle(container=self))

        # bind climber
        # self.co_trigger_x.onTrue(ClimberToggle(container=self, climber=self.climber, rpm=2500, force='on'))
        # self.co_trigger_y.onTrue(ClimberToggle(container=self, climber=self.climber, force='off'))

    def initialize_dashboard(self):

        # populate autonomous routines
        self.autonomous_chooser = wpilib.SendableChooser()
        wpilib.SmartDashboard.putData('autonomous routines', self.autonomous_chooser)

        wpilib.SmartDashboard.putData('_CMD_arm_up', ArmMove(container=self, arm=self.crank_arm, degrees=5, direction='up'))

        self.pathplanner_names_chooser = wpilib.SendableChooser()
        # import os

        #self.autonomous_chooser.setDefaultOption('_ do nothing', DriveWait(self, duration=1))
        #self.autonomous_chooser.addOption('drive 2m', DriveSwerveAutoVelocity(self, self.drive, velocity=1).withTimeout(2))

    def get_autonomous_command(self):

        # return self.autonomous_chooser.getSelected()

        # Load the path you want to follow using its name in the GUI
        path = PathPlannerPath.fromPathFile('Auto 1')
        # Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path)

