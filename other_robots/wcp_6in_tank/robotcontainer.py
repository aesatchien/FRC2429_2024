import wpilib

import commands2
from commands2.button import JoystickButton, POVButton
import time
import constants

from subsystems.vision import Vision
from subsystems.led import Led
from subsystems.shooter import Shooter
from subsystems.intake import Intake
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
#from subsystems.shooter_arm import ShooterCrankArm
from subsystems.shooter_crank_trapezoid import ShooterCrankArmTrapezoidal
from subsystems.indexer import Indexer

from commands.led_loop import LedLoop
from commands.led_toggle import LedToggle
from commands.shooter_toggle import ShooterToggle
from commands.arm_move import ArmMove
from commands.arm_joystick_control import ArmJoystickControl
from commands.arm_coast import CrankArmCoast
from commands.indexer_by_joystick import IndexerByJoystick


# from autonomous.drive_wait import DriveWait
# from autonomous.drive_move import DriveMove


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
        #self.drive = Drivetrain()
        self.vision = Vision()
        self.led = Led()
        self.shooter = Shooter()
        self.intake = Intake()
        self.crank_arm = LowerCrankArmTrapezoidal()  # CrankArm()
        self.shooter_arm = ShooterCrankArmTrapezoidal()
        self.indexer = Indexer()

        self.game_piece_mode = 'cube'  # TODO: change to empty and full? orange vs white?

        self.configureButtonBindings()

        # self.initialize_dashboard()

        # Set up default drive command
      #  if wpilib.RobotBase.isSimulation():
      #  if False:

        arm_degrees = 10 if wpilib.RobotBase.isReal() else 100
        self.indexer.setDefaultCommand(IndexerByJoystick(container=self, indexer=self.indexer))
        #self.shooter_arm.setDefaultCommand(ArmJoystickControl(container=self, arm=self.shooter_arm, controller=self.driver_controller, degrees=arm_degrees))
        #self.crank_arm.setDefaultCommand(ArmJoystickControl(container=self, arm=self.crank_arm, controller=self.driver_controller, degrees=arm_degrees))
    #    else:
        #self.drive.setDefaultCommand(DriveByJoystickVelocity(container=self, drive=self.drive, control_type='velocity', scaling=1))

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
        # self.buttonLeftAxis = AxisButton(self.driver_controller, 2)
        # self.buttonRightAxis = AxisButton(self.driver_controller, 3)

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
            # self.co_buttonLeftAxis = AxisButton(self.co_driver_controller, 2)
            # self.co_buttonRightAxis = AxisButton(self.co_driver_controller, 3)


        else:
            self.co_driver_controller = None

        # bind commands to driver

        # bind shooter
        self.buttonA.onTrue(ShooterToggle(container=self, shooter=self.shooter, rpm=2500, force='on'))
        self.buttonB.onTrue(ShooterToggle(container=self, shooter=self.shooter, force='off'))

        # bind intake
        #elf.buttonY.onTrue(IntakeToggle(container=self, intake=self.intake, rpm=2500, force='on'))

        # bind crank arms for testing
        #self.buttonUp.whileTrue(CrankArmToggle(container=self, crank_arm=self.crank_arm, power=1.5, force='on'))  # ToDo find out how to have it increment to the next position
        #self.buttonDown.whileTrue(CrankArmToggle(container=self, crank_arm=self.crank_arm, power=-1.0, force='on'))
        #self.buttonRight.whileTrue(ShooterArmToggle(container=self, shooter_arm=self.shooter_arm, power=2.0, force='on'))  # ToDo find out how to have it increment to the next position
        #self.buttonLeft.whileTrue(ShooterArmToggle(container=self, shooter_arm=self.shooter_arm, power=-1.5, force='on'))
        #self.buttonX.onTrue(CrankArmToggle(container=self, crank_arm=self.crank_arm, force='off'))
        #self.buttonUp.onTrue(self.crank_arm.move_degrees(degrees=10))
        #self.buttonDown.onTrue(self.crank_arm.move_degrees(degrees=-10))

        #self.buttonUp.onTrue(commands2.cmd.runOnce(lambda: test_system.set_next_position(direction='up'), self.shooter_arm))
        #self.buttonDown.onTrue(commands2.cmd.runOnce(lambda: test_system.set_next_position(direction='down'), self.shooter_arm))
        setpoints = False
        if setpoints:
            self.buttonRight.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=5, direction='up'))
            self.buttonLeft.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=-5, direction='down'))
            self.buttonUp.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=15, direction='up'))
            self.buttonDown.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=-15, direction='down'))
        else:
            direction = None
            self.buttonRight.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=10, direction=direction))
            self.buttonLeft.onTrue(ArmMove(container=self, arm=self.crank_arm, degrees=-10, direction=direction))
            self.buttonUp.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=5, direction=direction))
            self.buttonDown.onTrue(ArmMove(container=self, arm=self.shooter_arm, degrees=-5, direction=direction))

        self.buttonY.whileTrue(CrankArmCoast(container=self, crank_arm=self.crank_arm, ))
        self.buttonX.whileTrue(CrankArmCoast(container=self, crank_arm=self.shooter_arm))

       # bind LED
       #  self.buttonA.onTrue(LedToggle(container=self))

        # bind commands to co-pilot
        # self.co_buttonA.whenPressed(commands2.PrintCommand("Testing Button A"))
        # self.co_buttonBack.whenPressed(SafeCarry(self))

        # testing turret and elevator
        # enable_testing = False
        # if enable_testing:
        #     pass

        # commands2.button.JoystickButton(self.driverController, 3).whenHeld(
        #     HalveDriveSpeed(self.drive)
        # )


    # def initialize_dashboard(self):

        # lots of putdatas for testing on the dash
#        wpilib.SmartDashboard.putData(key='DriveMove', data=DriveMove(container=self, drive=self.drive, setpoint=1).withTimeout(5))


        # populate autonomous routines
        # self.autonomous_chooser = wpilib.SendableChooser()
        # wpilib.SmartDashboard.putData('autonomous routines', self.autonomous_chooser)
        # self.autonomous_chooser.addOption('do nothing', DriveWait(self, duration=1))
        # self.autonomous_chooser.addOption('drive 2m', DriveMove(self, self.drive, setpoint=2).withTimeout(4))

        # self.led_modes = wpilib.SendableChooser()
        # wpilib.SmartDashboard.putData('LED', self.led_modes)
        # self.led_modes.setDefaultOption('NONE', 'NONE')
        # self.led_modes.addOption('CONE', Led.Mode.CONE)
        # self.led_modes.addOption('CUBE', Led.Mode.CUBE)
        # self.led_modes.addOption('READY', Led.Mode.READY)
        # self.led_modes.addOption('OFF', Led.Mode.OFF)

    # def get_autonomous_command(self):
    #     return self.autonomous_chooser.getSelected()