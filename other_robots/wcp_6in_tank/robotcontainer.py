import wpilib

import commands2
from commands2.button import JoystickButton, POVButton
import time
import enum
import constants

from subsystems.drivetrain import Drivetrain
from subsystems.vision import Vision
from subsystems.led import Led
from subsystems.bucket import Bucket

from misc.axis_button import AxisButton
from commands.drive_by_joystick import DriveByJoystick
from commands.drive_velocity_stick import DriveByJoystickVelocity
from commands.led_loop import LedLoop
from commands.bucket_move import BucketMove


from autonomous.charge_station_balance import ChargeStationBalance
from autonomous.drive_wait import DriveWait
from autonomous.drive_move import DriveMove
from autonomous.drive_and_balance import DriveAndBalance
from autonomous.drive_climber import DriveClimber

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
        self.drive = Drivetrain()
        self.vision = Vision()
        self.led = Led()
        self.bucket = Bucket()

        self.game_piece_mode = 'cube'

        self.configureButtonBindings()

        self.initialize_dashboard()

        # Set up default drive command
      #  if wpilib.RobotBase.isSimulation():
        if False:

            self.drive.setDefaultCommand(DriveByJoystick(self, self.drive,lambda: -self.driver_controller.getRawAxis(1),
                    lambda: self.driver_controller.getRawAxis(4),))
        else:
            self.drive.setDefaultCommand(DriveByJoystickVelocity(container=self, drive=self.drive, control_type='velocity', scaling=1))

        self.led.setDefaultCommand(LedLoop(container=self))


    def set_start_time(self):  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def get_enabled_time(self):  # call when we want to know the start/elapsed time for status and debug messages
        return time.time() - self.start_time

    def configureButtonBindings(self):
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
        use_co_pilot = True
        if use_co_pilot:
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


        # All untested still
        # bind commands to driver
        self.buttonY.whileHeld(ChargeStationBalance(self, self.drive, velocity=10, tolerance=10))

        # bind commands to co-pilot
        self.co_buttonA.whenPressed(BucketMove(self, setpoint=100, bucket=self.bucket, wait_to_finish=True ))
        self.co_buttonB.whenPressed(BucketMove(self, setpoint=0, bucket=self.bucket, wait_to_finish=True ))

        # self.co_buttonA.whenPressed(commands2.PrintCommand("Testing Button A"))
        # self.co_buttonBack.whenPressed(SafeCarry(self))

        # testing turret and elevator
        enable_testing = False
        if enable_testing:
            pass

        # commands2.button.JoystickButton(self.driverController, 3).whenHeld(
        #     HalveDriveSpeed(self.drive)
        # )

    def initialize_dashboard(self):

        # lots of putdatas for testing on the dash
        wpilib.SmartDashboard.putData(key='DriveMove', data=DriveMove(container=self, drive=self.drive, setpoint=1).withTimeout(5))
        wpilib.SmartDashboard.putData(key='DriveAndBalance',data=DriveAndBalance(container=self).withTimeout(10))

        # populate autonomous routines
        self.autonomous_chooser = wpilib.SendableChooser()
        wpilib.SmartDashboard.putData('autonomous routines', self.autonomous_chooser)
        self.autonomous_chooser.addOption('do nothing', DriveWait(self, duration=1))
        self.autonomous_chooser.addOption('drive 2m', DriveMove(self, self.drive, setpoint=2).withTimeout(4))
        self.autonomous_chooser.addOption('drive and balance', DriveAndBalance(self).withTimeout(15))
        self.autonomous_chooser.addOption('station climb 2m', DriveClimber(self, self.drive, setpoint_distance=1.9).withTimeout(8))

        self.led_modes = wpilib.SendableChooser()
        wpilib.SmartDashboard.putData('LED', self.led_modes)
        self.led_modes.setDefaultOption('NONE', 'NONE')
        self.led_modes.addOption('CONE', Led.Mode.CONE)
        self.led_modes.addOption('CUBE', Led.Mode.CUBE)
        self.led_modes.addOption('READY', Led.Mode.READY)
        self.led_modes.addOption('OFF', Led.Mode.OFF)

    def get_autonomous_command(self):
        return self.autonomous_chooser.getSelected()
