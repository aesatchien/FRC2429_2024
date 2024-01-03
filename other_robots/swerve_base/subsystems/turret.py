"""
Turret subsystem
Shares the following with networktables:  turret_angle

Turret needs to:
1) know its ABSOLUTE position at all times  - we need a calibrate function
2) rotate on command
3) not exceed certain angular limit
4) not tip, so check speeds
5) go to certain certain angles here - for stow, and autonomous scoring
"""

from commands2 import SubsystemBase
import rev
import wpilib
from wpilib import SmartDashboard

import constants
from misc.configure_controllers import configure_sparkmax
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim


class Turret(SubsystemBase):
    def __init__(self):
        super().__init__()

        self.max_angle = 251
        self.min_angle = -45
        self.counter = 15  # offset the periodics
        self.angle = 0  # just to initialize
        # turret should probably have positions that we need to map out
        # self.positions = {'full2': 250, 'full': 225, 'score': 180, 'middle': 90, 'stow': 0}
        self.positions = {'stow': 0, 'score': 180}

        # initialize motors
        self.turret_controller = rev.CANSparkMax(constants.k_turret_motor_port, rev.CANSparkMax.MotorType.kBrushless)
        self.turret_controller.setInverted(True)  # true for turret
        self.sparkmax_encoder = self.turret_controller.getEncoder()
        self.sparkmax_encoder.setPositionConversionFactor(constants.k_turret_encoder_conversion_factor)
        self.sparkmax_encoder.setVelocityConversionFactor(constants.k_turret_encoder_conversion_factor)  # needed for smartmotion
        self.sparkmax_encoder.setPosition(0)
        self.pid_controller = self.turret_controller.getPIDController()

        # set soft limits - do not let spark max put out power above/below a certain value
        self.turret_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, constants.k_enable_soft_limits)
        self.turret_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, constants.k_enable_soft_limits)
        self.turret_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_angle)
        self.turret_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_angle)
        self.pid_controller.setSmartMotionAllowedClosedLoopError(1)

        configure_sparkmax(sparkmax=self.turret_controller, pid_controller=self.pid_controller, slot=0, can_id=constants.k_turret_motor_port,
                           pid_dict=constants.k_PID_dict_vel_turret, pid_only=True, burn_flash=constants.k_burn_flash)

        # same here, and need the turret encoder to be set to analog (jumper change)
        self.analog_abs_encoder = wpilib.AnalogEncoder(constants.k_turret_abs_encoder_port)  # plug the analog encoder into channel 1
        # self.analog_abs_encoder = wpilib.DutyCycleEncoder(9)
        self.analog_conversion_factor = 360.0  # 5V is 360 degrees
        self.analog_abs_encoder.setDistancePerRotation(self.analog_conversion_factor)

        # set the offset on the absolute analog encoder
        self.absolute_position_offset = 0 #  0.842  # this is what the absolute encoder reports when in stow position
        # self.analog_abs_encoder.setPositionOffset(self.absolute_position_offset)  # now stow alignment is angle=0

        self.angle = self.get_angle()
        self.setpoint = self.angle  # initial setting should be?
        SmartDashboard.putNumber('turret_angle', self.angle)
        SmartDashboard.putNumber('turret_setpoint', self.setpoint)

    def get_angle(self):  # getter for the relevant turret parameter
        if wpilib.RobotBase.isReal():
            return self.sparkmax_encoder.getPosition()
        else:
            return self.angle

    def set_turret_angle(self, angle, mode='smartmotion'):
        """
        We can do this multiple ways -
        1) we can use wpilib with a PID controller
        2) we can make a PID controller ourselves
        3) or we can use the sparkmax's built-in PID / smartmotion
        """
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kSmartMotion, )
        elif mode == 'position':
            # just use the position PID
            self.pid_controller.setReference(angle, rev.CANSparkMax.ControlType.kPosition)

        self.angle = angle
        self.setpoint = angle
        SmartDashboard.putNumber('turret_setpoint', self.angle)

        if wpilib.RobotBase.isSimulation():
            SmartDashboard.putNumber('turret_angle', self.angle)

    # add this so the user has an override - basically you align it and can flip back and forth b/w 0 and 180
    def reset_turret_encoder(self, angle=None):
        self.pid_controller.setReference(0, rev.CANSparkMax.ControlType.kVelocity, pidSlot=0)
        if angle is None:
            if abs(self.get_angle() - 180) < 20:  # we are near 180, call it zero
                self.sparkmax_encoder.setPosition(0)
                self.setpoint = 0
            else:
                self.sparkmax_encoder.setPosition(180)
                self.setpoint = 180
        else:
            self.sparkmax_encoder.setPosition(angle)

    def periodic(self) -> None:
        self.counter += 1
        if self.counter % 25 == 0:
            self.angle = self.get_angle()
            SmartDashboard.putNumber('turret_abs_encoder', self.analog_abs_encoder.getDistance())
            SmartDashboard.putNumber('turret_angle', self.angle)