#  Container for 2429's 2023 swerve robot with turret, elevator, arm, wrist, and manipulator

import time, enum
import wpilib
import commands2
from commands2.button import JoystickButton, POVButton
from commands2.button import CommandXboxController

# pathplanner
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

import constants  # all of the constants except for swerve

# subsystems
from subsystems.swerve import Swerve

# commands
from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from commands.gyro_reset import GyroReset
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from autonomous.drive_swerve_point_trajectory import DriveSwervePointTrajectory



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

        self.configure_joysticks()
        self.bind_buttons()
        self.configure_swerve_bindings()

        self.initialize_dashboard()

        # swerve driving
        self.drive.setDefaultCommand(DriveByJoystickSwerve(container=self, swerve=self.drive,
                            field_oriented=constants.k_field_centric, rate_limited=constants.k_rate_limited))



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

    def configure_swerve_bindings(self):
        self.trigger_a.debounce(0.05).onTrue(DriveSwerveAutoVelocity(container=self, drive=self.drive, velocity=0.25,
        direction = 'forwards', decide_by_turret = False).withTimeout(0.5))
        self.trigger_b.debounce(0.05).onTrue(GyroReset(self, swerve=self.drive))
        self.trigger_y.whileTrue(DriveSwervePointTrajectory(container=self,drive=self.drive,pointlist=None,velocity=None,acceleration=None))

    def bind_buttons(self):
       pass

    def initialize_dashboard(self):

        # populate autonomous routines
        self.autonomous_chooser = wpilib.SendableChooser()
        wpilib.SmartDashboard.putData('autonomous routines', self.autonomous_chooser)

        self.pathplanner_names_chooser = wpilib.SendableChooser()
        # import os

        #self.autonomous_chooser.setDefaultOption('_ do nothing', DriveWait(self, duration=1))
        #self.autonomous_chooser.addOption('drive 2m', DriveSwerveAutoVelocity(self, self.drive, velocity=1).withTimeout(2))

    def get_autonomous_command(self):

        # return self.autonomous_chooser.getSelected()

        # Load the path you want to follow using its name in the GUI
        path = PathPlannerPath.fromPathFile('Auto 7')
        # Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path)

