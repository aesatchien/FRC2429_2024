"""
Crank-Arm subsystem
Shares the following with networktables:  crank_arm_position
"""
import time
from commands2 import Subsystem
import rev
import wpilib
from wpilib import SmartDashboard
import constants
from misc.configure_controllers import configure_sparkmax
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim


class CrankArm(Subsystem):
    # CrankArm should probably have four positions that we need to map out
    positions = {'rest': 30, 'amp': 90,'trap': 120}  # todo: might need to set "intake" position

    def __init__(self):
        super().__init__()
        self. counter = 20  # offset the periodics
        # defining angles so 0 is horizontal
        self.max_angle = 120  # call all the way up 125 degrees  todo: remeasure
        self.min_angle = 30

        self.in_use_by_driver = False

        # initialize motors
        self.crank_arm_controller = rev.CANSparkMax(constants.k_crank_motor_port, rev.CANSparkMax.MotorType.kBrushless)

        # self.wrist_controller.restoreFactoryDefaults()
        self.crank_arm_controller.setIdleMode(mode=rev.CANSparkMax.IdleMode.kBrake)
        self.crank_arm_controller.setInverted(True)  # verified that this is true for the directions we want on crank arm
        self.sparkmax_encoder = self.crank_arm_controller.getEncoder()

        # update sparkmax with appropriate system gains and constraints
        self.sparkmax_encoder.setPositionConversionFactor(constants.k_crank_encoder_conversion_factor)  # Unsure what we want our conversion factor to be (default is set in constants)
        self.sparkmax_encoder.setVelocityConversionFactor(constants.k_crank_encoder_conversion_factor)  # Unsure what we want our conversion factor to be (default is set in constants)
        self.pid_controller = self.crank_arm_controller.getPIDController()

        # set soft limits - do not let spark max put out power above/below a certain value
        self.crank_arm_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, constants.k_enable_soft_limits)
        self.crank_arm_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, constants.k_enable_soft_limits)
        self.crank_arm_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_angle)
        self.crank_arm_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_angle)
        self.crank_arm_controller.setSmartCurrentLimit(20)
        self.pid_controller.setSmartMotionAllowedClosedLoopError(1)

        configure_sparkmax(sparkmax=self.crank_arm_controller, pid_controller=self.pid_controller, slot=0, can_id= constants.k_crank_motor_port,
                           pid_dict=constants.k_PID_dict_vel_crank_arm, pid_only=True, burn_flash=constants.k_burn_flash)

        self.abs_encoder = self.crank_arm_controller.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.abs_encoder.setInverted(True)
        self.abs_encoder.setPositionConversionFactor(360)
        self.abs_encoder.setZeroOffset(360 * 0.38)

        self.pid_controller.setFeedbackDevice(sensor=self.abs_encoder)

        # initialize the location of the CrankArm - from absolute encoder
        self.angle = self.abs_encoder.getPosition()
        # sometimes it does not initialize, so try again
        if abs(self.angle) < 0.1:
            print(f'Crank absolute encoder measurement was {self.angle}; retrying')
            time.sleep(0.2)  # robot will complain, but it's only on boot up
            self.angle = self.abs_encoder.getPosition()

        if self.angle > 180:  # correct for if we start out negative.  e.g. -10 would come out as 350
            self.angle = self.angle - 360
        self.sparkmax_encoder.setPosition(self.angle)
        self.setpoint = self.angle
        SmartDashboard.putNumber('crank_angle', self.angle)
        SmartDashboard.putNumber('crank_setpoint', self.setpoint)
        self.is_moving = False  # use for determining if we are jumping setpoints

    def get_angle(self):  # getter for the relevant elevator parameter
        if wpilib.RobotBase.isReal():
            return self.sparkmax_encoder.getPosition()
        else:
            return self.angle

    def set_crank_arm_angle(self, angle, mode='smartmotion'):
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kSmartMotion)
        elif mode == 'position':
            # just use the position PID
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kPosition)

        self.setpoint = angle
        SmartDashboard.putNumber('crank_setpoint', angle)
        if wpilib.RobotBase.isSimulation():
            self.angle = angle
            SmartDashboard.putNumber('crank_angle', self.angle)

    def set_encoder_position(self, angle):
        self.sparkmax_encoder.setPosition(angle)
        if wpilib.RobotBase.isSimulation():
            self.angle = angle

    def set_driver_flag(self, state=False):
        self.in_use_by_driver = state
        SmartDashboard.putBoolean('crank_in_use', state)

    def get_driver_flag(self):
        return self.in_use_by_driver

    def periodic(self) -> None:
        self.counter += 1

        if self.counter % 25 == 0:
            self.angle = self.get_angle()
            SmartDashboard.putNumber('crank_angle', self.angle)
            SmartDashboard.putNumber('crank_abs_encoder', self.abs_encoder.getPosition())

            self.is_moving = abs(self.sparkmax_encoder.getVelocity()) > 100  #
