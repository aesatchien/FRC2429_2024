#  Container for 2429's 2023 swerve robot with turret, elevator, arm, wrist, and manipulator

import time, enum
import wpilib
import commands2
import commands2.cmd as cmd
from commands2.button import JoystickButton, POVButton

import constants  # all of the constants except for swerve

# from subsystems.drivetrain import Drivetrain
from subsystems.vision import Vision
from subsystems.led import Led
from subsystems.swerve import Swerve

from misc.axis_button import AxisButton
from commands.record_auto import RecordAuto
from commands.led_loop import LedLoop
from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from commands.gyro_reset import GyroReset
from commands.led_toggle import LedToggle

from autonomous.drive_wait import DriveWait
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from autonomous.auto_rotate_swerve import AutoRotateSwerve
from autonomous.auto_strafe_swerve import AutoStrafeSwerve

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
        if constants.k_use_swerve:
            self.drive = Swerve()


        self.vision = Vision()
        self.led = Led()

        self.game_piece_mode = 'cone'

        self.configure_joysticks()
        self.bind_buttons()
        self.configure_swerve_bindings()

        self.initialize_dashboard()

        self.led.setDefaultCommand(LedLoop(container=self))

        # swerve driving
        if constants.k_use_swerve:
            self.drive.setDefaultCommand(DriveByJoystickSwerve(container=self, swerve=self.drive,
                            field_oriented=constants.k_field_centric, rate_limited=constants.k_rate_limited))
        else:
            self.drive.setDefaultCommand(
                DriveByJoystickVelocity(container=self, drive=self.drive, control_type='velocity', scaling=1))

        # initialize the turret
        # commands2.ScheduleCommand(TurretInitialize(container=self, turret=self.turret, samples=50)).initialize()

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
        self.driver_controller = wpilib.XboxController(constants.k_driver_controller_port)
        self.buttonA = JoystickButton(self.driver_controller, 1)
        self.buttonB = JoystickButton(self.driver_controller, 2)
        self.buttonX = JoystickButton(self.driver_controller, 3)
        self.buttonY = JoystickButton(self.driver_controller, 4)
        self.buttonLB = JoystickButton(self.driver_controller, 5)
        self.buttonRB = JoystickButton(self.driver_controller, 6)
        self.buttonBack = JoystickButton(self.driver_controller, 7)
        self.buttonStart = JoystickButton(self.driver_controller, 8)
        self.buttonUp = POVButton(self.driver_controller, 0)
        self.buttonDown = POVButton(self.driver_controller, 180)
        self.buttonLeft = POVButton(self.driver_controller, 270)
        self.buttonRight = POVButton(self.driver_controller, 90)
        self.buttonLeftAxis = AxisButton(self.driver_controller, 2)
        self.buttonRightAxis = AxisButton(self.driver_controller, 3)

        # co-pilot controller
        self.co_driver_controller = wpilib.XboxController(constants.k_co_driver_controller_port)
        self.co_buttonA = JoystickButton(self.co_driver_controller, 1)
        self.co_buttonB = JoystickButton(self.co_driver_controller, 2)
        self.co_buttonX = JoystickButton(self.co_driver_controller, 3)
        self.co_buttonY = JoystickButton(self.co_driver_controller, 4)
        self.co_buttonLB = JoystickButton(self.co_driver_controller, 5)
        self.co_buttonRB = JoystickButton(self.co_driver_controller, 6)
        self.co_buttonBack = JoystickButton(self.co_driver_controller, 7)
        self.co_buttonStart = JoystickButton(self.co_driver_controller, 8)
        self.co_buttonUp = POVButton(self.co_driver_controller, 0)
        self.co_buttonDown = POVButton(self.co_driver_controller, 180)
        self.co_buttonLeft = POVButton(self.co_driver_controller, 270)
        self.co_buttonRight = POVButton(self.co_driver_controller, 90)
        self.co_buttonLeftAxis = AxisButton(self.co_driver_controller, 2)
        self.co_buttonRightAxis = AxisButton(self.co_driver_controller, 3)

    def configure_swerve_bindings(self):
        # self.buttonA.debounce(0.1).onTrue(SwerveX(container=self, swerve=self.drive))
        self.buttonB.debounce(0.1).onTrue(GyroReset(self, swerve=self.drive))
        self.buttonX.debounce(0.1).onTrue(AutoStrafeSwerve(container=self, drive=self.drive, vision=self.vision,
                                                           target_type='tag', auto=True).withTimeout(5))
        # self.buttonX.debounce(0.1).onTrue(SwerveAngleTest(self, swerve=self.drive))
        self.buttonY.debounce(0.1).onTrue(AutoRotateSwerve(container=self, drive=self.drive,).withTimeout(2))

    def bind_buttons(self):
        self.buttonUp.whenPressed(self.led.set_indicator_with_timeout(Led.Indicator.RAINBOW, 5))
        self.buttonLeft.whenPressed(self.led.set_indicator_with_timeout(Led.Indicator.RSL, 5))

        self.buttonRightAxis.whenPressed(LedToggle(container=self))

        if wpilib.RobotBase.isReal():
            # this log doesn't work with Windows machines
            self.buttonRight.whenPressed(RecordAuto(container=self, input_log_path='/home/lvuser/input_log.json'))
        else:
            # this log would get wiped with all new deploys
            self.buttonRight.whenPressed(RecordAuto(container=self, input_log_path='input_log.json'))

    def initialize_dashboard(self):

        # populate autonomous routines
        self.autonomous_chooser = wpilib.SendableChooser()
        wpilib.SmartDashboard.putData('autonomous routines', self.autonomous_chooser)
        self.autonomous_chooser.setDefaultOption('_ do nothing', DriveWait(self, duration=1))
        self.autonomous_chooser.addOption('drive 2m', DriveSwerveAutoVelocity(self, self.drive, velocity=1).withTimeout(2))

    def get_autonomous_command(self):
        return self.autonomous_chooser.getSelected()
