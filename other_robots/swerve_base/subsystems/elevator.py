"""
Elevator subsystem
Shares the following with networktables:  elevator_height
Elevator needs to:
1) know its vertical position
2) not exceed set min/max values
3) drive to any of a element from a list of positions
4) allow a driver override of position
5) needs a function to set and hold a position



"""
from commands2 import SubsystemBase
import rev
from wpilib import SmartDashboard
from playingwithfusion import TimeOfFlight
import wpilib

import constants
from misc.configure_controllers import configure_sparkmax
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim


class Elevator(SubsystemBase):
    # elevator should probably have positions that we need to map out
    positions = {'top': 950, 'upper_pickup': 850, 'low': 650, 'lower_pickup': 300, 'bottom': 50}
    positions_close = {'top': 950, 'low': 650}  # top and middle scoring positions
    positions_open = {'upper_pickup': 850, 'bottom': 50}  # only go to pickup from station or ground

    def __init__(self):
        super().__init__()
        self.counter = 5  # offset the periodics

        self.max_height = 981  # the bottom of the carriage is 39in (991mm)  above the bottom at max height
        self.min_height = 49  # mm for now

        # initialize motors
        self.elevator_controller = rev.CANSparkMax(constants.k_elevator_motor_port, rev.CANSparkMax.MotorType.kBrushless)
        self.elevator_controller.setInverted(True)  # true for elevator
        self.sparkmax_encoder = self.elevator_controller.getEncoder()
        self.sparkmax_encoder.setPositionConversionFactor(constants.k_elevator_encoder_conversion_factor)  # mm per revolution
        self.sparkmax_encoder.setVelocityConversionFactor(constants.k_elevator_encoder_conversion_factor)  # necessary for smartmotion to behave
        self.pid_controller = self.elevator_controller.getPIDController()

        # set up distance sensor
        self.elevator_height_sensor = TimeOfFlight(constants.k_elevator_timeoflight)
        self.elevator_height_sensor.setRangingMode(TimeOfFlight.RangingMode.kShort, 50)

        # set soft limits - do not let spark max put out power above/below a certain value
        self.elevator_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, constants.k_enable_soft_limits)
        self.elevator_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, constants.k_enable_soft_limits)
        self.elevator_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_height)
        self.elevator_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_height)
        self.pid_controller.setSmartMotionAllowedClosedLoopError(1)

        configure_sparkmax(sparkmax=self.elevator_controller, pid_controller=self.pid_controller, slot=0, can_id=constants.k_elevator_motor_port,
                           pid_dict=constants.k_PID_dict_vel_elevator, pid_only=True, burn_flash=constants.k_burn_flash)

        # initialize the height of the elevator  - sensor is in mm, so stick with that
        initial_height = self.elevator_height_sensor.getRange()
        self.sparkmax_encoder.setPosition(initial_height)
        self.height = initial_height
        self.setpoint = self.height  # initial setting should be ?
        SmartDashboard.putNumber('elevator_height', self.height)
        SmartDashboard.putNumber('elevator_setpoint', self.setpoint)

        self.is_moving = False

    def get_height(self):  # getter for the relevant elevator parameter
        if wpilib.RobotBase.isReal():
            return self.sparkmax_encoder.getPosition()
        else:
            return self.height

    def reset_height(self, height):
        self.height = height
        self.sparkmax_encoder.setPosition(height)
        SmartDashboard.putNumber('elevator_height', self.height)

    def set_elevator_height(self, height, mode='smartmotion'):
        """
        We can do this multiple ways -
        1) we can use wpilib with a PID controller
        2) we can make a PID controller ourselves
        3) or we can use the sparkmax's built-in PID / smartmotion
        """
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(height, rev.CANSparkMax.ControlType.kSmartMotion)
        elif mode == 'position':
            # just use the position PID
            self.pid_controller.setReference(height, rev.CANSparkMax.ControlType.kPosition)

        self.setpoint = height
        SmartDashboard.putNumber('elevator_setpoint', self.setpoint)

        if wpilib.RobotBase.isSimulation():
            self.height = height
            SmartDashboard.putNumber('elevator_height', self.height)

    def periodic(self) -> None:
        self.counter += 1
        if self.counter % 25 == 0:
            self.height = self.get_height()

            self.is_moving = abs(self.sparkmax_encoder.getVelocity()) > 1000  #

            SmartDashboard.putNumber('elevator_height', self.height)
            SmartDashboard.putNumber('elevator_tof', self.elevator_height_sensor.getRange())