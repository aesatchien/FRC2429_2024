"""
Wrist subsystem
Shares the following with networktables:  wrist_position
"""
import time
from commands2 import SubsystemBase
import rev
import wpilib
from wpilib import SmartDashboard
import constants
from misc.configure_controllers import configure_sparkmax
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim


class Wrist(SubsystemBase):
    # wrist should probably have four positions that we need to map out
    positions = {'stow': 93, 'score': 55, 'op_score': 10, 'flat': 0, 'floor': -25}

    def __init__(self):
        super().__init__()
        self. counter = 20  # offset the periodics
        # defining angles so 0 is horizontal
        self.max_angle = 94  # call all the way up 125 degrees  todo: remeasure
        self.min_angle = -26

        self.in_use_by_driver = False

        # initialize motors
        self.wrist_controller = rev.CANSparkMax(constants.k_wrist_motor_port, rev.CANSparkMax.MotorType.kBrushless)

        # self.wrist_controller.restoreFactoryDefaults()
        self.wrist_controller.setIdleMode(mode=rev.CANSparkMax.IdleMode.kBrake)
        self.wrist_controller.setInverted(True)  # verified that this is true for the directions we want on wrist
        self.sparkmax_encoder = self.wrist_controller.getEncoder()

        # update sparkmax with appropriate system gains and constraints
        self.sparkmax_encoder.setPositionConversionFactor(constants.k_wrist_encoder_conversion_factor)  # mm per revolution
        self.sparkmax_encoder.setVelocityConversionFactor(constants.k_wrist_encoder_conversion_factor)  # necessary for smartmotion to behave
        self.pid_controller = self.wrist_controller.getPIDController()

        # no longer necessary after adding the absolute encoder
        # self.forward_limit_switch = self.wrist_controller.getForwardLimitSwitch(switchType=rev.SparkMaxLimitSwitch.Type.kNormallyOpen)
        # self.at_fwd_limit = self.forward_limit_switch.get()
        # SmartDashboard.putBoolean('wrist_limit', self.at_fwd_limit)

        # set soft limits - do not let spark max put out power above/below a certain value
        self.wrist_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, constants.k_enable_soft_limits)
        self.wrist_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, constants.k_enable_soft_limits)
        self.wrist_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_angle)
        self.wrist_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_angle)
        self.wrist_controller.setSmartCurrentLimit(20)
        self.pid_controller.setSmartMotionAllowedClosedLoopError(1)

        configure_sparkmax(sparkmax=self.wrist_controller, pid_controller=self.pid_controller, slot=0, can_id=constants.k_wrist_motor_port,
                           pid_dict=constants.k_PID_dict_vel_wrist, pid_only=True, burn_flash=constants.k_burn_flash)

        self.abs_encoder = self.wrist_controller.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.abs_encoder.setInverted(True)
        self.abs_encoder.setPositionConversionFactor(360)
        self.abs_encoder.setZeroOffset(360 * 0.38)

        # self.pid_controller.setFeedbackDevice(sensor=self.abs_encoder)

        # initialize the location of the wrist - from absolute encoder
        self.angle = self.abs_encoder.getPosition()
        # sometimes it does not initialize, so try again
        if abs(self.angle) < 0.1:
            print(f'Wrist absolute encoder measurement was {self.angle}; retrying')
            time.sleep(0.2)  # robot will complain, but it's only on boot up
            self.angle = self.abs_encoder.getPosition()

        if self.angle > 180:  # correct for if we start out negative.  e.g. -10 would come out as 350
            self.angle = self.angle - 360
        self.sparkmax_encoder.setPosition(self.angle)
        self.setpoint = self.angle
        SmartDashboard.putNumber('wrist_angle', self.angle)
        SmartDashboard.putNumber('wrist_setpoint', self.setpoint)
        self.is_moving = False  # use for determining if we are jumping setpoints

    def get_angle(self):  # getter for the relevant elevator parameter
        if wpilib.RobotBase.isReal():
            return self.sparkmax_encoder.getPosition()
        else:
            return self.angle

    def set_wrist_angle(self, angle, mode='smartmotion'):
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kSmartMotion)
        elif mode == 'position':
            # just use the position PID
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kPosition)

        self.setpoint = angle
        SmartDashboard.putNumber('wrist_setpoint', angle)
        if wpilib.RobotBase.isSimulation():
            self.angle = angle
            SmartDashboard.putNumber('wrist_angle', self.angle)

    def set_encoder_position(self, angle):
        self.sparkmax_encoder.setPosition(angle)
        if wpilib.RobotBase.isSimulation():
            self.angle = angle

    def set_driver_flag(self, state=False):
        self.in_use_by_driver = state
        SmartDashboard.putBoolean('wrist_in_use', state)

    def get_driver_flag(self):
        return self.in_use_by_driver

    def periodic(self) -> None:
        self.counter += 1

        if self.counter % 25 == 0:
            self.angle = self.get_angle()
            SmartDashboard.putNumber('wrist_angle', self.angle)
            SmartDashboard.putNumber('wrist_abs_encoder', self.abs_encoder.getPosition())

            self.is_moving = abs(self.sparkmax_encoder.getVelocity()) > 100  #
