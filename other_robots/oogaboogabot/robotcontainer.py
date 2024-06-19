#  Container for 2429's 2023 swerve robot with turret, elevator, arm, wrist, and manipulator
import math
import time

import commands2
import wpilib
from commands2.button import CommandXboxController

# pathplanner
from pathplannerlib.auto import NamedCommands, PathPlannerPath

import constants

# subsystems
from subsystems.swerve import Swerve
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from subsystems.vision import Vision

# commands
from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from commands.gyro_reset import GyroReset
from commands.move_arm_by_pose import MoveArmByPose
from commands.drive_and_auto_aim_chassis import DriveAndAutoAimChassis
from commands.arm_move import ArmMove


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
            self.shooter_arm = UpperCrankArmTrapezoidal()
            self.vision = Vision()

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
        self.trigger_only_b.debounce(0.05).onTrue(GyroReset(self, swerve=self.drive))
        # self.trigger_y.whileTrue(DriveSwervePointTrajectory(container=self,drive=self.drive,pointlist=None,velocity=None,acceleration=None))

    def bind_driver_buttons(self):
        # TRIGGER B BOUND IN SWERVE SECTION

        # AUTO AIMING
        self.trigger_shift_x.debounce(0.05).whileTrue(MoveArmByPose(container=self, shooting_backwards=False))
        self.trigger_shift_x.debounce(0.05).whileTrue(DriveAndAutoAimChassis(container=self, swerve=self.drive, field_oriented=constants.k_field_centric, rate_limited=constants.k_rate_limited, shooting_backwards=False))

        self.trigger_a.onTrue(ArmMove(self, self.shooter_arm, degrees=90, absolute=True))
        self.trigger_x.onTrue(ArmMove(self, self.shooter_arm, degrees=0, absolute=True))


        # WE SHOULD NOT BIND LB.  IT IS USED AS ROBOT-CENTRIC IN DRIVE AND AS A SHIFT BUTTON ON OTHER COMMANDS


        # SHIFT COMMANDS - using left bumper as a button multiplier
        #self.trigger_shift_a.onTrue(commands2.PrintCommand('You pressed A and LB - and nothing else happened'))
        self.trigger_shift_b.onTrue(commands2.PrintCommand('You pressed B and LB - and nothing else happened'))
        # self.trigger_shift_x.onTrue(commands2.PrintCommand('You pressed X and LB - and nothing else happened'))
        # self.trigger_shift_y.onTrue(commands2.PrintCommand('You pressed Y and LB - and nothing else happened'))

    def bind_copilot_buttons(self):
        pass
        # arm positions


    def set_automated_path(self, command : PathPlannerPath):
        self.autonomous_command = command

    def get_automated_path(self) -> PathPlannerPath:
        if self.autonomous_command is not None:
            return self.autonomous_command
        return None


    def get_arm_configuration(self):
        return self.arm_configuration
    def set_arm_configuration(self, configuration):
        self.arm_configuration = configuration  # shoot, intake, amp, trap, etc

    def initialize_dashboard(self):

        # populate autonomous routines
        self.autonomous_chooser = wpilib.SendableChooser()
        # Manually add our own
        # self.autonomous_chooser.setDefaultOption('Shoot and drive out', ShootAndDriveOut(container=self, lower_arm=self.crank_arm, upper_arm=self.shooter_arm, drive=self.drive))
        # self.autonomous_chooser.addOption('Aim and shoot', AimAndShoot(container=self, lower_arm=self.crank_arm, upper_arm=self.shooter_arm))
        # if wpilib.RobotBase.isReal():
        #     self.autonomous_chooser.addOption('Playback auto', PlaybackAuto(self, "/home/lvuser/input_log.json"))
        # else:
        #     self.autonomous_chooser.addOption('Playback auto: Sim edition', PlaybackAuto(self, 'input_log.json'))

        # Automatically get Pathplanner paths


        wpilib.SmartDashboard.putData('autonomous routines', self.autonomous_chooser)

        self.position_chooser = wpilib.SendableChooser()
        wpilib.SmartDashboard.putData('Position Chooser', self.position_chooser)
        # self.autonomous_chooser.addOption("To Blue Amp from anywhere", PathPlannerConfiguration.on_the_fly_path(self.drive, {"x": constants.k_blue_amp[0]-self.drive.get_pose().X(), "y": constants.k_blue_amp[1]-self.drive.get_pose().Y(), "rotation": constants.k_blue_amp[2]-self.drive.get_angle()}, 0, speed_factor=0.5))
        # self.autonomous_chooser.addOption("To Blue Amp from anywhere", PathPlannerConfigurationCommand(self, self.drive, {"x": constants.k_blue_amp[0], "y": constants.k_blue_amp[1], "rotation": constants.k_blue_amp[2]}, 0, speed_factor=0.5, fast_turn=True))

        # put commands that we want to call from the dashboard - IDE has problems w/ CommandBase vs Command
        wpilib.SmartDashboard.putData('GyroReset', GyroReset(self, swerve=self.drive))
        wpilib.SmartDashboard.putData('MoveArmbyPose', MoveArmByPose(container=self))
        wpilib.SmartDashboard.putData('Drive and auto aim chassis', DriveAndAutoAimChassis(container=self, swerve=self.drive))
        # wpilib.SmartDashboard.putData('Gaslight encoders', GaslightCrankEncoders(self, self.crank_arm))
        wpilib.SmartDashboard.putData('GyroFromPose', GyroReset(self, swerve=self.drive, from_pose=True))

        wpilib.SmartDashboard.putData(commands2.CommandScheduler.getInstance())

    def get_autonomous_command(self):
        return self.autonomous_chooser.getSelected()
