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
        self.max_angle = 180  # call all the way up ? degrees  todo: remeasure and verify
        self.min_angle = 0

        # initialize motors
        motor_type = rev.CANSparkMax.MotorType.kBrushless
        # TODO - check if follower is working properly
        self.crank_motor_right_spark = rev.CANSparkMax(constants.k_top_crank_motor_right, motor_type)
        self.crank_controller = self.crank_motor_right_spark.getPIDController()

        self.crank_motor_left_spark = rev.CANSparkMax(constants.k_top_crank_motor_left, motor_type)
        self.crank_motor_left_spark.follow(self.crank_motor_right_spark,invert=True)  # now we only have to get one right

        self.sparks = [self.crank_motor_left_spark, self.crank_motor_right_spark]
        self.controllers = [self.crank_controller]

        # drive the shooter crank entirely by the absolute encoder mounted to the right motor
        self.abs_encoder = self.crank_motor_right_spark.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.abs_encoder.setInverted(True)  # needs to be inverted - clockwise on the right arm cranks the shooter down
        self.crank_controller.setFeedbackDevice(self.abs_encoder)
        initial_position = [self.abs_encoder.getPosition() for i in range(5)]

        # shooter abs encoder is 100:1 to the spark but 1:1 to the arm
        pos_factor = constants.k_top_crank_abs_encoder_position_conversion_factor  # 360
        self.abs_encoder.setPositionConversionFactor(pos_factor)  # degrees
        self.abs_encoder.setVelocityConversionFactor(pos_factor / 60)  # degrees per second
        self.abs_encoder.setZeroOffset(pos_factor * 0.188)  # Todo - figure this out for upper crank - 0 is next / parallel to lower arm
        print(f'Upper crank absolute encoder position at boot: {initial_position} set to {self.abs_encoder.getPosition()} degrees')
        self.angle = self.abs_encoder.getPosition()

        # we may not even want to use this, although it may help in the simulation
        self.sparkmax_encoder = self.crank_motor_right_spark.getEncoder()
        self.sparkmax_encoder.setPositionConversionFactor(360 / 100)
        self.sparkmax_encoder.setPosition(self.angle + 0)  # seems to track it backwards though, not sure how to fix this

        # update controllers from constants file and optionally burn flash
        self.configure_motors()

        # toogle state
        self.arm_enabled = False
        SmartDashboard.putBoolean('shooter_crank_arm_state', self.arm_enabled)


    def configure_motors(self):
        # move motor configuration out of __init__
        self.crank_motor_right_spark.setInverted(True)
        # self.crank_motor_left_spark.setInverted(False)  # one MUST be true and the other MUST false

        for controller in self.controllers:
            controller.setSmartMotionAllowedClosedLoopError(1)

        for spark in self.sparks:
            spark.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)

            # set soft limits - do not let spark max put out power above/below a certain value - see constants
            spark.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, constants.k_enable_soft_limits)
            spark.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, constants.k_enable_soft_limits)
            spark.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_angle)
            spark.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_angle)
            # spark.setSmartCurrentLimit(80)

            # set PIDs for position and velocoty, burn flash only on the second time through
            configure_sparkmax(sparkmax=self.crank_motor_right_spark, pid_controller=self.crank_controller,
                               pid_dict=constants.k_PID_dict_pos_shooter_arm, can_id=constants.k_top_crank_motor_right,
                               slot=0, pid_only=False, burn_flash=False)
            configure_sparkmax(sparkmax=self.crank_motor_right_spark, pid_controller=self.crank_controller,
                               pid_dict=constants.k_PID_dict_vel_shooter_arm, can_id=constants.k_top_crank_motor_right,
                               slot=1, pid_only=False, burn_flash=constants.k_burn_flash)

    def set_crank_arm_angle(self, angle, mode='smartmotion'):
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.crank_controller.setReference(angle, rev.CANSparkMax.ControlType.kSmartMotion)
        elif mode == 'position':
            # just use the position PID -
            # not a great idea except to hold yourself there - may use this as a lock after smartmotion?
            self.crank_controller.setReference(value=angle, ctrl=rev.CANSparkMax.ControlType.kPosition, pidSlot=0)

        self.setpoint = angle
        SmartDashboard.putNumber('shoter_arm_setpoint', angle)
        if wpilib.RobotBase.isSimulation():
            self.angle = angle
            SmartDashboard.putNumber('shooter_arm_angle', self.angle)

    def set_crank_arm(self, power):
        # for debugging purposes - currently used in crank arm toggle to manually go up and down
        self.crank_arm_voltage = power
        self.crank_controller.setReference(self.crank_arm_voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
        self.arm_enabled = True
        print(f'setting power to {self.crank_arm_voltage}')
        SmartDashboard.putBoolean('shooter_arm_state', self.arm_enabled)

    def stop_crank_arm(self):  # todo - lock the position with sparkmax setpoints
        self.angle = self.get_angle()
        # TODO - figure out the arbitrary feedforward as a function of angle to feed here
        if wpilib.RobotBase.isReal():
            self.crank_controller.setReference(value=self.angle, ctrl=rev.CANSparkFlex.ControlType.kPosition, pidSlot=0)
        else:
            self.crank_controller.setReference(value=0, ctrl=rev.CANSparkFlex.ControlType.kVoltage, pidSlot=0)
        self.arm_enabled = False
        self.crank_arm_voltage = 0
        SmartDashboard.putBoolean('shooter_arm_state', self.arm_enabled)

    def get_angle(self):  # getter for the relevant elevator parameter
        if wpilib.RobotBase.isReal():
            return self.abs_encoder.getPosition()
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
            SmartDashboard.putNumber('shooter_crank_abs_encoder_degrees', self.abs_encoder.getPosition())
            SmartDashboard.putNumberArray('shooter_arm_powers', [self.crank_motor_right_spark.getAppliedOutput(),
                                                                 self.crank_motor_left_spark.getAppliedOutput()])
            self.is_moving = abs(self.sparkmax_encoder.getVelocity()) > 100  #



