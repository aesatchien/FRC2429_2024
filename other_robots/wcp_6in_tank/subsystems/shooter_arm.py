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


class ShooterCrankArm(Subsystem):
    # CrankArm should probably have four positions that we need to map out
    positions = {'intake': 0, 'shoot':0, 'amp': 90,'trap': 180}  # todo: might need to set "intake" position

    def __init__(self):
        super().__init__()
        self.setName('Shooter Arm')
        # defining angles so 0 is horizontal
        self.counter = 0

        self.max_angle = 180  # call all the way up ? degrees  todo: remeasure
        self.min_angle = 0

        # initialize motors
        motor_type = rev.CANSparkMax.MotorType.kBrushless
        # TODO - consider to set one of these motors as a follower with the correct inversion - halve the activity
        self.crank_motor_left_spark = rev.CANSparkMax(constants.k_top_crank_motor_left, motor_type)
        self.crank_motor_right_spark = rev.CANSparkMax(constants.k_top_crank_motor_right, motor_type)
        self.crank_motor_left_controller = self.crank_motor_left_spark.getPIDController()
        self.crank_motor_right_controller = self.crank_motor_right_spark.getPIDController()
        self.sparks = [self.crank_motor_left_spark, self.crank_motor_right_spark]
        self.controllers = [self.crank_motor_left_controller, self.crank_motor_right_controller]

        self.sparkmax_encoder = self.crank_motor_right_spark.getEncoder()  # helps with simulation
        self.abs_encoder = self.crank_motor_right_spark.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.abs_encoder.setInverted(True)
        initial_position = [self.abs_encoder.getPosition() for i in range(5)]
        self.abs_encoder.setPositionConversionFactor(360/3)  # pulley ratio must be taken into account
        self.abs_encoder.setZeroOffset(360 * 0.38)  # Todo - figure this out for upper crank - 0 is next / parallel to lower arm
        print(f'Upper crank absolute encoder position at boot: {initial_position} set to {self.abs_encoder.getPosition()} degrees')
        self.angle = self.abs_encoder.getPosition()

        # toogle state
        self.arm_enabled = False
        SmartDashboard.putBoolean('crank_arm_state', self.arm_enabled)


    def configure_motors(self):
        # move motor configuration out of __init__
        self.crank_motor_left_spark.setInverted(False)  # one MUST be true and the other MUST false
        self.crank_motor_right_spark.setInverted(True)

        for controller in self.controllers:
            controller.setP(0)  # not necessary when we pull in the velocity settings dictionary
            controller.setSmartMotionAllowedClosedLoopError(1)

        for spark in self.sparks:
            spark.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)

            # set soft limits - do not let spark max put out power above/below a certain value
            spark.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, constants.k_enable_soft_limits)
            spark.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, constants.k_enable_soft_limits)
            spark.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_angle)
            spark.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_angle)
            # spark.setSmartCurrentLimit(80)

            for slot in [0]:  # burn only one slot for now - should be all we need for an arm
                configure_sparkmax(sparkmax=self.crank_motor_left_spark, pid_controller=self.crank_motor_left_controller,
                                   pid_dict=constants.k_PID_dict_vel_top_crank_arm, can_id=constants.k_top_crank_motor_left,
                                   slot=slot, pid_only=False, burn_flash=constants.k_burn_flash)
                configure_sparkmax(sparkmax=self.crank_motor_right_spark, pid_controller=self.crank_motor_left_controller,
                                   pid_dict=constants.k_PID_dict_vel_top_crank_arm, can_id=constants.k_top_crank_motor_right,
                                   slot=slot, pid_only=False, burn_flash=constants.k_burn_flash)

    def set_crank_arm_angle(self, angle, mode='smartmotion'):
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kSmartMotion)
        elif mode == 'position':
            # just use the position PID -
            # not a great idea except to hold yourself there - may use this as a lock after smartmotion?
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kPosition)

        self.setpoint = angle
        SmartDashboard.putNumber('shoter_arm_setpoint', angle)
        if wpilib.RobotBase.isSimulation():
            self.angle = angle
            SmartDashboard.putNumber('shooter_arm_angle', self.angle)

    def set_crank_arm(self, power):
        # for debugging purposes - currently used in crank arm toggle to manually go up and down
        self.crank_arm_voltage = power
        self.crank_motor_left_controller.setReference(self.crank_arm_voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
        self.crank_motor_right_controller.setReference(self.crank_arm_voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
        self.arm_enabled = True
        print(f'setting power to {self.crank_arm_voltage}')
        SmartDashboard.putBoolean('shooter_arm_state', self.arm_enabled)

    def stop_crank_arm(self):  # todo - lock the position with sparkmax setpoints
        self.crank_motor_left_controller.setReference(0, rev.CANSparkFlex.ControlType.kVoltage)
        self.crank_motor_right_controller.setReference(0, rev.CANSparkFlex.ControlType.kVoltage)
        self.arm_enabled = False
        self.crank_arm_voltage =0
        SmartDashboard.putBoolean('shooter_arm_state', self.arm_enabled)

    def get_angle(self):  # getter for the relevant elevator parameter
        if wpilib.RobotBase.isReal():
            return self.sparkmax_encoder.getPosition()
        else:
            return self.angle

    def set_encoder_position(self, angle):
        self.sparkmax_encoder.setPosition(angle)
        if wpilib.RobotBase.isSimulation():
            self.angle = angle

    def periodic(self) -> None:

        self.counter += 1
        if self.counter % 25 == 0:
            self.angle = self.get_angle()
            SmartDashboard.putNumber('shooter_crank_sparkmax_angle', self.angle)
            SmartDashboard.putNumber('shooter_crank_encoder_degrees', self.abs_encoder.getPosition())
            self.is_moving = abs(self.sparkmax_encoder.getVelocity()) > 100  #



