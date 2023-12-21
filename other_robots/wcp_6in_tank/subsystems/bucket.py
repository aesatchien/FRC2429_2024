import wpilib
from commands2 import SubsystemBase
from wpilib import MotorControllerGroup, PWMSparkMax, SmartDashboard
import rev

from misc.configure_controllers import configure_sparkmax

import constants

class Bucket(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.counter = 0

        self.min_degrees = -10
        self.max_degrees = 135

        # initialize motors
        self.spark_neo_bucket = rev.CANSparkMax(constants.k_bucket, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.spark_neo_bucket.setInverted(False)
        self.sparkmax_encoder = self.spark_neo_bucket.getEncoder()
        self.sparkmax_encoder.setPositionConversionFactor(constants.k_bucket_conversion_factor)  # mm per revolution
        self.sparkmax_encoder.setVelocityConversionFactor(constants.k_bucket_conversion_factor)  # necessary for smartmotion to behave

        # get encoders and PID controllers
        self.pid_controller = self.spark_neo_bucket.getPIDController()
        # self.spark_neo_bucket.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, constants.k_enable_soft_limts)
        # self.spark_neo_bucket.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, constants.k_enable_soft_limts)
        self.pid_controller.setSmartMotionAllowedClosedLoopError(1)

        configure_sparkmax(sparkmax=self.spark_neo_bucket, pid_controller=self.pid_controller, slot=0, can_id=constants.k_bucket,
                           pid_dict=constants.k_PID_dict_vel_bucket , pid_only=True, burn_flash=constants.k_burn_flash)


        # initialize the angle of the bucket
        self.angle = self.sparkmax_encoder.getPosition()
        self.setpoint = self.angle  # initial setting should be ?
        SmartDashboard.putNumber('bucket_angle', self.angle)
        SmartDashboard.putNumber('bucket_setpoint', self.setpoint)

        self.is_moving = False

    def get_angle(self):  # getter for the relevant bucket parameter
        if wpilib.RobotBase.isReal():
            return self.sparkmax_encoder.getPosition()
        else:
            return self.angle

    def reset_angle(self, angle):
        self.angle = angle
        self.sparkmax_encoder.setPosition(angle)
        SmartDashboard.putNumber('bucket_angle', self.angle)

    def set_bucket_angle(self, angle, mode='smartmotion'):
        """
        We can do this multiple ways -
        1) we can use wpilib with a PID controller
        2) we can make a PID controller ourselves
        3) or we can use the sparkmax's built-in PID / smartmotion
        """
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kSmartMotion)
        elif mode == 'position':
            # just use the position PID
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kPosition)

        self.setpoint = angle
        SmartDashboard.putNumber('bucket_setpoint', self.setpoint)

        if wpilib.RobotBase.isSimulation():
            self.angle = angle
            SmartDashboard.putNumber('bucket_angle', self.angle)

    def periodic(self) -> None:
        self.counter += 1
        if self.counter % 25 == 0:
            self.angle = self.get_angle()

            self.is_moving = abs(self.sparkmax_encoder.getVelocity()) > 1000  #

            SmartDashboard.putNumber('bucket_angle', self.angle)