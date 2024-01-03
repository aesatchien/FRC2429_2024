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
from subsystems.arm import Arm
from subsystems.elevator import Elevator
from subsystems.turret import Turret
from subsystems.wrist import Wrist
from subsystems.pneumatics import Pneumatics

from misc.axis_button import AxisButton
from commands.record_auto import RecordAuto
from commands_unused.drive_velocity_stick import DriveByJoystickVelocity
from commands.arm_move import ArmMove
from commands.turret_move import TurretMove
from commands.turret_toggle import TurretToggle
from commands.elevator_move import ElevatorMove
from commands.wrist_move import WristMove
from commands.manipulator_toggle import ManipulatorToggle
from commands.compressor_toggle import CompressorToggle
from commands.generic_drive import GenericDrive  # Siraaj's way of moving all subsystems
from commands.led_loop import LedLoop
from commands.toggle_high_pickup import ToggleHighPickup
from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from commands.swerve_x import SwerveX
from commands.swerve_angle_test import SwerveAngleTest
from commands.gyro_reset import GyroReset
from commands.co_stow import CoStow
from commands.manipulator_auto_grab import ManipulatorAutoGrab
from commands.led_toggle import LedToggle
from commands.turret_reset import TurretReset
from commands.wrist_calibration import WristCalibration

from autonomous.playback_auto import PlaybackAuto
from autonomous.arm_calibration import ArmCalibration
from autonomous.score_hi_cone_from_stow import ScoreHiConeFromStow
from autonomous.score_low_cone_from_stow import ScoreLowConeFromStow
from autonomous.drive_and_balance import DriveAndBalance
from autonomous.score_exit_comm_and_balance import ScoreExitCommAndBalance
from autonomous.score_twice import ScoreTwice
from autonomous.charge_station_balance import ChargeStationBalance
from autonomous.safe_carry import SafeCarry
from autonomous.turret_move_by_vision import TurretMoveByVision
from autonomous.score_by_vision import ScoreByVision
from autonomous.drive_wait import DriveWait
from autonomous.turret_initialize import TurretInitialize
from autonomous.upper_substation_pickup import UpperSubstationPickup
from autonomous.release_and_stow import ReleaseAndStow
from autonomous.score_hi_and_move import ScoreHiAndMove
from autonomous.score_drive_and_balance import ScoreDriveAndBalance
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from autonomous.auto_rotate_swerve import AutoRotateSwerve
from autonomous.auto_strafe_swerve import AutoStrafeSwerve
from autonomous.swerve_score_by_vision import SwerveScoreByVision
from autonomous.auto_setup_score import AutoSetupScore
from autonomous.score_low_cone_from_stow import ScoreLowConeFromStow

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    class CommandSelector(enum.Enum):
        TURRET_DOWN = enum.auto()
        TURRET_UP = enum.auto()
        ELEVATOR_UP = enum.auto()
        ELEVATOR_DOWN = enum.auto()
        ARM_UP = enum.auto()
        ARM_DOWN = enum.auto()
        WRIST_UP = enum.auto()
        WRIST_DOWN = enum.auto()
        TURRET_DOWN_DRIVE = enum.auto()
        TURRET_UP_DRIVE = enum.auto()
        ELEVATOR_UP_DRIVE = enum.auto()
        ELEVATOR_DOWN_DRIVE = enum.auto()
        ARM_UP_DRIVE = enum.auto()
        ARM_DOWN_DRIVE = enum.auto()
        WRIST_UP_DRIVE = enum.auto()
        WRIST_DOWN_DRIVE = enum.auto()
        NONE = enum.auto()

    def select_preset(self, direction) -> CommandSelector:
        if self.co_driver_controller.getRawButton(1):
            # don't toggle the turret (do nothing when direction = UP/DOWN)
            if direction == 'UP_DRIVE' or direction == 'DOWN_DRIVE':
                return self.CommandSelector[f'TURRET_{direction}']
        elif self.co_driver_controller.getRawButton(2):
            return self.CommandSelector[f'ELEVATOR_{direction}']
        elif self.co_driver_controller.getRawButton(4):
            return self.CommandSelector[f'ARM_{direction}']
        elif self.co_driver_controller.getRawButton(3):
            return self.CommandSelector[f'WRIST_{direction}']

        return self.CommandSelector.NONE

    def __init__(self) -> None:

        self.start_time = time.time()

        # The robot's subsystems
        if constants.k_use_swerve:
            self.drive = Swerve()

        self.turret = Turret()
        self.arm = Arm()
        self.wrist = Wrist()
        self.elevator = Elevator()
        self.pneumatics = Pneumatics()  # can't enable unless there is a module there
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
        self.buttonA.debounce(0.1).onTrue(AutoSetupScore(container=self))
        self.buttonB.debounce(0.1).onTrue(GyroReset(self, swerve=self.drive))
        self.buttonX.debounce(0.1).onTrue(AutoStrafeSwerve(container=self, drive=self.drive, vision=self.vision,
                                                           target_type='tag', auto=True).withTimeout(5))
        # self.buttonX.debounce(0.1).onTrue(SwerveAngleTest(self, swerve=self.drive))
        self.buttonY.debounce(0.1).onTrue(AutoRotateSwerve(container=self, drive=self.drive,).withTimeout(2))

    def bind_buttons(self):
        # All untested still
        # bind commands to driver
        # self.buttonY.whileHeld(ChargeStationBalance(container=self, drive=self.drive, auto=False))
        self.buttonBack.whenPressed(CompressorToggle(self, self.pneumatics, force="stop"))
        self.buttonStart.whenPressed(CompressorToggle(self, self.pneumatics, force="start"))
        self.buttonRB.whenPressed(
            cmd.runOnce(action=lambda: self.wrist.set_driver_flag(state=True)).andThen(
            ReleaseAndStow(container=self).withTimeout(4)).andThen(
            cmd.runOnce(action=lambda: self.wrist.set_driver_flag(state=False))))

        self.buttonDown.whenPressed(ManipulatorToggle(container=self, pneumatics=self.pneumatics))
        self.buttonUp.whenPressed(self.led.set_indicator_with_timeout(Led.Indicator.RAINBOW, 5))
        self.buttonLeft.whenPressed(self.led.set_indicator_with_timeout(Led.Indicator.RSL, 5))

        self.buttonRightAxis.whenPressed(LedToggle(container=self))

        if wpilib.RobotBase.isReal():
            # this log doesn't work with Windows machines
            self.buttonRight.whenPressed(RecordAuto(container=self, input_log_path='/home/lvuser/input_log.json'))
        else:
            # this log would get wiped with all new deploys
            self.buttonRight.whenPressed(RecordAuto(container=self, input_log_path='input_log.json'))

        # self.buttonLeftAxis.whenPressed(self.led.set_indicator_with_timeout(Led.Indicator.VISION_TARGET_SUCCESS, 2))

        # bind commands to co-pilot
        # self.co_buttonLB.whenPressed(ManipulatorToggle(self, self.pneumatics, force="close"))
        # self.co_buttonRB.whenPressed(ManipulatorToggle(self, self.pneumatics, force="open"))
        # self.co_buttonRB.whenPressed(ManipulatorToggle(self, self.pneumatics))

        # self.co_buttonA.whileHeld(GenericDrive(self, self.turret, max_velocity=constants.k_PID_dict_vel_turret["SM_MaxVel"], axis=0, invert_axis=False))
        # self.co_buttonB.whileHeld(GenericDrive(self, self.elevator, max_velocity=constants.k_PID_dict_vel_elevator["SM_MaxVel"], axis=1, invert_axis=True))
        # self.co_buttonY.whileHeld(GenericDrive(self, self.arm, max_velocity=constants.k_PID_dict_vel_arm["SM_MaxVel"], axis=1, invert_axis=True))
        # self.co_buttonX.whileHeld(GenericDrive(self, self.wrist, max_velocity=constants.k_PID_dict_vel_wrist["SM_MaxVel"], axis=1, invert_axis=True))

        # self.co_buttonBack.whenPressed(SafeCarry(self))
        # self.co_buttonBack.whenPressed(TurretMove(self, self.turret, setpoint=0, wait_to_finish=False))

        self.co_buttonBack.whenPressed(CoStow(container=self))
        #self.co_buttonStart.whenPressed(TurretMoveByVision(self, turret=self.turret, vision=self.vision, find_alternate=False))
        self.co_buttonStart.whenPressed(TurretReset(container=self, turret=self.turret))  # allow copilot to reset turret
        self.co_buttonLeftAxis.whenPressed(TurretToggle(container=self, turret=self.turret, wait_to_finish=False))
        self.co_buttonRightAxis.whenPressed(TurretToggle(container=self, turret=self.turret, wait_to_finish=False))

        self.co_buttonRB.whileHeld(ManipulatorAutoGrab(container=self, pneumatics=self.pneumatics))
        # self.co_buttonA.whenPressed(ToggleGroundPickup(container=self, pneumatics=self.pneumatics, wrist=self.wrist, button=1))

        self.co_buttonLB.whenPressed(ToggleHighPickup(container=self, turret=self.turret, elevator=self.elevator, wrist=self.wrist, pneumatics=self.pneumatics, vision=self.vision))

        preset_command_map = [
            (self.CommandSelector.TURRET_UP, TurretMove(self, self.turret, direction="up", wait_to_finish=False)),
            (self.CommandSelector.TURRET_DOWN, TurretMove(self, self.turret, direction="down", wait_to_finish=False)),
            (self.CommandSelector.TURRET_UP_DRIVE, GenericDrive(self, self.turret, max_velocity=constants.k_PID_dict_vel_turret["SM_MaxVel"], input_type='dpad', direction=1)),
            (self.CommandSelector.TURRET_DOWN_DRIVE, GenericDrive(self, self.turret, max_velocity=constants.k_PID_dict_vel_turret["SM_MaxVel"], input_type='dpad', direction=-1)),
            (self.CommandSelector.ELEVATOR_UP, ElevatorMove(self, self.elevator, direction="up", wait_to_finish=False, drive_controls=True)),
            (self.CommandSelector.ELEVATOR_DOWN, ElevatorMove(self, self.elevator, direction="down", wait_to_finish=False, drive_controls=True)),
            (self.CommandSelector.ELEVATOR_UP_DRIVE, GenericDrive(self, self.elevator, max_velocity=constants.k_PID_dict_vel_elevator["SM_MaxVel"], input_type='dpad', direction=1)),
            (self.CommandSelector.ELEVATOR_DOWN_DRIVE, GenericDrive(self, self.elevator, max_velocity=constants.k_PID_dict_vel_elevator["SM_MaxVel"], input_type='dpad', direction=-1)),
            (self.CommandSelector.ARM_UP, ArmMove(self, self.arm, direction="up", wait_to_finish=False)),
            (self.CommandSelector.ARM_DOWN, ArmMove(self, self.arm, direction="down", wait_to_finish=False)),
            (self.CommandSelector.ARM_UP_DRIVE, GenericDrive(self, self.arm, max_velocity=constants.k_PID_dict_vel_arm["SM_MaxVel"], input_type='dpad', direction=1)),
            (self.CommandSelector.ARM_DOWN_DRIVE, GenericDrive(self, self.arm, max_velocity=constants.k_PID_dict_vel_arm["SM_MaxVel"], input_type='dpad', direction=-1)),
            (self.CommandSelector.WRIST_UP, WristMove(self, self.wrist, direction="down", wait_to_finish=False)),
            (self.CommandSelector.WRIST_DOWN, WristMove(self, self.wrist, direction="up", wait_to_finish=False)),
            # (
            #     self.CommandSelector.WRIST_UP,
            #     commands2.ConditionalCommand(commands2.PrintCommand('Aborted wrist up'), WristMove(self, self.wrist, direction="down", wait_to_finish=False), lambda: self.wrist.get_driver_flag()),
            # ),
            # (
            #     self.CommandSelector.WRIST_DOWN,
            #     commands2.ConditionalCommand(commands2.PrintCommand('Aborted wrist down'), WristMove(self, self.wrist, direction="up", wait_to_finish=False), lambda: self.wrist.get_driver_flag()),
            # ),
            (self.CommandSelector.WRIST_UP_DRIVE, GenericDrive(self, self.wrist, max_velocity=constants.k_PID_dict_vel_wrist["SM_MaxVel"], control_type='velocity', input_type='dpad', direction=1, invert_axis=True)),
            (self.CommandSelector.WRIST_DOWN_DRIVE, GenericDrive(self, self.wrist, max_velocity=constants.k_PID_dict_vel_wrist["SM_MaxVel"], control_type='velocity', input_type='dpad', direction=-1, invert_axis=True)),
            (self.CommandSelector.NONE, cmd.nothing()),
            # (self.CommandSelector.NONE, commands2.PrintCommand('Doing nothing'))
        ]

        self.co_buttonUp.debounce(debounceTime=0.1).whileTrue(commands2.SelectCommand(
            lambda: self.select_preset("UP_DRIVE"),
            preset_command_map,
        ))

        self.co_buttonDown.debounce(debounceTime=0.1).whileTrue(commands2.SelectCommand(
            lambda: self.select_preset("DOWN_DRIVE"),
            preset_command_map,
        ))

        self.co_buttonLeft.whenPressed(commands2.SelectCommand(
            lambda: self.select_preset("DOWN"),
            preset_command_map,
        ))

        self.co_buttonRight.whenPressed(commands2.SelectCommand(
            lambda: self.select_preset("UP"),
            preset_command_map,
        ))

        # testing turret and elevator
        enable_testing = False
        if enable_testing:
            self.buttonRight.whenPressed(TurretMove(self, self.turret, direction='up', wait_to_finish=True).withTimeout(2))
            self.buttonLeft.whenPressed(TurretMove(self, self.turret, direction='down', wait_to_finish=True).withTimeout(2))
            self.buttonDown.whenPressed(ElevatorMove(self, self.elevator, direction='up', wait_to_finish=True).withTimeout(1))
            self.buttonUp.whenPressed(ElevatorMove(self, self.elevator, direction='down', wait_to_finish=True).withTimeout(1))
            # manipulator
            self.buttonRB.whenPressed(ManipulatorToggle(container=self, pneumatics=self.pneumatics, force='open'))
            self.buttonLB.whenPressed(ManipulatorToggle(container=self, pneumatics=self.pneumatics, force='close'))

            #self.co_buttonRB.whileHeld(ElevatorDrive(container=self, elevator=self.elevator, button=self.co_buttonRB))

        # commands2.button.JoystickButton(self.driverController, 3).whenHeld(
        #     HalveDriveSpeed(self.drive)
        # )

    def initialize_dashboard(self):

        # lots of putdatas for testing on the dash
        wpilib.SmartDashboard.putData(GyroReset(container=self, swerve=self.drive))
        wpilib.SmartDashboard.putData(TurretInitialize(container=self, turret=self.turret))
        wpilib.SmartDashboard.putData(ScoreHiConeFromStow(container=self))
        wpilib.SmartDashboard.putData(ScoreLowConeFromStow(container=self))
        wpilib.SmartDashboard.putData(UpperSubstationPickup(container=self))
        wpilib.SmartDashboard.putData(key='SafeCarry', data=SafeCarry(container=self).withTimeout(5))
        wpilib.SmartDashboard.putData(ManipulatorToggle(container=self, pneumatics=self.pneumatics))
        wpilib.SmartDashboard.putData(CompressorToggle(container=self, pneumatics=self.pneumatics))
        wpilib.SmartDashboard.putData(key='ElevatorMoveUp', data=ElevatorMove(container=self, elevator=self.elevator, direction='up', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='ElevatorMoveDown', data=ElevatorMove(container=self, elevator=self.elevator, direction='down', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='WristMoveUp', data=WristMove(container=self, wrist=self.wrist, direction='up', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='WristMoveDown', data=WristMove(container=self, wrist=self.wrist, direction='down', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='ArmMoveUp', data=ArmMove(container=self, arm=self.arm, direction='up', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='ArmMoveDown', data=ArmMove(container=self, arm=self.arm, direction='down', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='TurretMoveUp', data=TurretMove(container=self, turret=self.turret, direction='up', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='TurretMoveDown', data=TurretMove(container=self, turret=self.turret, direction='down', wait_to_finish=False))
        wpilib.SmartDashboard.putData(key='ArmCalibration', data=ArmCalibration(container=self, arm=self.arm).withTimeout(5))
        # wpilib.SmartDashboard.putData(key='WristCalibration', data=WristCalibration(container=self, wrist=self.wrist).withTimeout(5))
        wpilib.SmartDashboard.putData(key='TurretMoveByVision', data=TurretMoveByVision(container=self, turret=self.turret, vision=self.vision, color='green').withTimeout(5))
        wpilib.SmartDashboard.putData(key='ScoreByVision', data=ScoreByVision(container=self, turret=self.turret, elevator=self.elevator, arm=self.arm, pneumatics=self.pneumatics, vision=self.vision))
        wpilib.SmartDashboard.putData(key='UpperSubstationPickup', data=UpperSubstationPickup(container=self).withTimeout(6))
        wpilib.SmartDashboard.putData(key='ReleaseAndStow', data=ReleaseAndStow(container=self).withTimeout(5))
        wpilib.SmartDashboard.putData(key='AutoRotate', data=AutoRotateSwerve(container=self, drive=self.drive, find_closest_heading=True, auto=True).withTimeout(5))
        wpilib.SmartDashboard.putData(key='AutoStrafeTag', data=AutoStrafeSwerve(container=self, drive=self.drive, vision=self.vision, target_type='tag', auto=True).withTimeout(5))
        wpilib.SmartDashboard.putData(key='AutoStrafeGreen', data=AutoStrafeSwerve(container=self, drive=self.drive, vision=self.vision, target_type='green', auto=True).withTimeout(5))
        wpilib.SmartDashboard.putData(key='AutoSetupScore', data=AutoSetupScore(container=self))
        wpilib.SmartDashboard.putData(key='LedToggle', data=LedToggle(container=self))
        wpilib.SmartDashboard.putData(WristCalibration(container=self, wrist=self.wrist))
        #wpilib.SmartDashboard.putData(key='DriveMove', data=DriveMove(container=self, drive=self.drive, setpoint=1).withTimeout(5))
        #wpilib.SmartDashboard.putData(key='DriveAndBalance',data=DriveAndBalance(container=self).withTimeout(10))

        # populate autonomous routines
        self.autonomous_chooser = wpilib.SendableChooser()
        wpilib.SmartDashboard.putData('autonomous routines', self.autonomous_chooser)
        self.autonomous_chooser.setDefaultOption('_ do nothing', DriveWait(self, duration=1))
        self.autonomous_chooser.addOption('drive 2m', DriveSwerveAutoVelocity(self, self.drive, velocity=1).withTimeout(2))
        self.autonomous_chooser.setDefaultOption('score hi and chill', ScoreHiConeFromStow(self))
        self.autonomous_chooser.addOption('score hi and move near', ScoreHiAndMove(self, distance=2.5))
        self.autonomous_chooser.addOption('score hi and move far', ScoreHiAndMove(self, distance=4.5))
        self.autonomous_chooser.addOption('drive and balance', DriveAndBalance(self).withTimeout(15))
        self.autonomous_chooser.addOption('score hi, drive and balance', ScoreDriveAndBalance(self).withTimeout(15))
        self.autonomous_chooser.addOption('score with swerve', SwerveScoreByVision(self))
        self.autonomous_chooser.addOption('score hi exit community and balance', ScoreExitCommAndBalance(self))
        self.autonomous_chooser.addOption('score twice', ScoreTwice(self))

        self.autonomous_chooser.addOption('score low cone froms stow', ScoreLowConeFromStow(container=self))

        if wpilib.RobotBase.isReal():
            self.autonomous_chooser.addOption('playback auto', PlaybackAuto(container=self, input_log_path='/home/lvuser/input_log.json'))
        else:
            self.autonomous_chooser.addOption('playback auto', PlaybackAuto(container=self, input_log_path='input_log.json'))

        # self.autonomous_chooser.addOption('low cone from stow', ScoreLowConeFromStow(self))
        # self.autonomous_chooser.addOption('balance on station', ChargeStationBalance(container=self, drive=self.drive).withTimeout(10))

    def get_autonomous_command(self):
        return self.autonomous_chooser.getSelected()
