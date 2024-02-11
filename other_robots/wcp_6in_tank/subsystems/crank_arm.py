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


class CrankArm(Subsystem):
    # CrankArm should probably have four positions that we need to map out
    positions = {'intake': 45, 'shoot':70, 'amp': 100,'trap': 110}  # todo: might need to set "intake" position

    def __init__(self):
        super().__init__()
        self.setName('Crank Arm')
        # defining angles so 0 is horizontal
        self.counter = 0
        self.max_angle = 120  # call all the way up 125 degrees  todo: remeasure and verify
        self.min_angle = 45

        # initialize motors
        motor_type = rev.CANSparkMax.MotorType.kBrushless
        # TODO - consider to set one of these motors as a follower with the correct inversion - halve the activity
        self.crank_motor_right_spark = rev.CANSparkMax(constants.k_lower_crank_motor_right, motor_type)
        self.crank_controller = self.crank_motor_right_spark.getPIDController()

        self.crank_motor_left_spark = rev.CANSparkMax(constants.k_lower_crank_motor_left, motor_type)
        self.crank_motor_left_spark.follow(self.crank_motor_right_spark,invert=True)  # now we only have to get one right

        self.sparks = [self.crank_motor_left_spark, self.crank_motor_right_spark]
        self.controllers = [self.crank_controller]

        # drive the shooter crank entirely by the absolute encoder mounted to the right motor
        self.abs_encoder = self.crank_motor_right_spark.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.abs_encoder.setInverted(False)  # verfied false by GAN on 20240210
        initial_position = [self.abs_encoder.getPosition() for i in range(5)]

        # crank abs encoder is 75:1 to the spark but 4:1 to the arm
        pos_factor = constants.k_lower_crank_abs_encoder_position_conversion_factor  # 90
        self.abs_encoder.setPositionConversionFactor(pos_factor)  # degrees, pulley ratio must be taken into account
        self.abs_encoder.setVelocityConversionFactor(pos_factor / 60)  # degrees per second
        self.abs_encoder.setZeroOffset(pos_factor * 0.576)  # verified .576 by GAN on 20240210
        print(f'Lower crank absolute encoder position at boot: {initial_position} set to {self.abs_encoder.getPosition()} degrees')
        self.angle = self.abs_encoder.getPosition()

        # we may not even want to use this, although it may help in the simulation
        self.sparkmax_encoder = self.crank_motor_right_spark.getEncoder()  # helps with simulation
        self.sparkmax_encoder.setPositionConversionFactor(360/300)
        self.sparkmax_encoder.setPosition(360/4 * self.angle + 90)

        # update controllers from constants file and optionally burn flash
        self.configure_motors()

        # toogle state
        self.arm_enabled = False
        SmartDashboard.putBoolean('crank_arm_state', self.arm_enabled)


    def configure_motors(self):
        # move motor configuration out of __init__
        self.crank_motor_right_spark.setInverted(True)  # right motor CW moves crank arm down

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

            # set PIDs for position and velocoty, burn flash only on the second time through
            configure_sparkmax(sparkmax=self.crank_motor_right_spark, pid_controller=self.crank_controller,
                               pid_dict=constants.k_PID_dict_pos_shooter_arm, can_id=constants.k_lower_crank_motor_right,
                               slot=0, pid_only=False, burn_flash=False)
            configure_sparkmax(sparkmax=self.crank_motor_right_spark, pid_controller=self.crank_controller,
                               pid_dict=constants.k_PID_dict_vel_shooter_arm, can_id=constants.k_lower_crank_motor_right,
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
        SmartDashboard.putNumber('crank_arm_setpoint', angle)
        if wpilib.RobotBase.isSimulation():
            self.angle = angle
            SmartDashboard.putNumber('crank_arm_angle', self.angle)

    def set_crank_arm(self, power):
        # for debugging purposes - currently used in crank arm toggle to manually go up and down
        self.crank_arm_voltage = power
        self.crank_controller.setReference(self.crank_arm_voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
        self.arm_enabled = True
        print(f'setting power to {self.crank_arm_voltage}')
        SmartDashboard.putBoolean('crank_arm_state', self.arm_enabled)

    def stop_crank_arm(self):
        self.angle = self.get_angle()
        # TODO - figure out the arbitrary feedforward as a function of angle to feed here
        if wpilib.RobotBase.isReal():
            self.crank_controller.setReference(value=self.angle, ctrl=rev.CANSparkFlex.ControlType.kPosition, pidSlot=0)
        else:
            self.crank_controller.setReference(value=0, ctrl=rev.CANSparkFlex.ControlType.kVoltage, pidSlot=0)
        self.arm_enabled = False
        self.crank_arm_voltage =0
        SmartDashboard.putBoolean('crank_arm_state', self.arm_enabled)

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
        super().periodic()  # this does the automatic motion profiling in the background
        self.counter += 1
        if self.counter % 10 == 0:
            self.angle = self.get_angle()
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_sparkmax_angle', self.angle)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_degrees', self.angle*180/math.pi)
            wpilib.SmartDashboard.putNumberArray(f'{self.getName()}_powers', [self.motor.getAppliedOutput(),
                                           self.follower.getAppliedOutput()])
            self.is_moving = abs(self.abs_encoder.getVelocity()) > 0.01  # rad per second
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_is_moving', self.is_moving)


