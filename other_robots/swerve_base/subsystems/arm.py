"""
Arm subsystem
Shares the following with networktables:  arm_extension

Arm needs to
1) pull back all the way for stow
2) have a controllable distance from stow to fully extended
3) can probably trust the default encoder for the distance, just need to start with it retracted to define zero

"""
import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard
import rev
import constants
from misc.configure_controllers import configure_sparkmax
import math

class Arm(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.counter = 0
        self.motion_log_counter = 0

        self.max_extension = 580  # need to see what is max legal amount
        self.min_extension = 2  # mm for now
        # arm should probably have positions that we need to map out
        self.positions = {'full': 568, 'middle': 450, 'stow': 3}

        # initialize motors
        self.arm_controller = rev.CANSparkMax(constants.k_arm_motor_port, rev.CANSparkMax.MotorType.kBrushless)
        self.arm_controller.setInverted(True)  # todo: arm needs to be true
        self.sparkmax_encoder = self.arm_controller.getEncoder()
        self.sparkmax_encoder.setPositionConversionFactor(constants.k_arm_encoder_conversion_factor)  # mm per revolution
        self.sparkmax_encoder.setVelocityConversionFactor(constants.k_arm_encoder_conversion_factor)
        self.pid_controller = self.arm_controller.getPIDController()

        # set soft limits - do not let spark max put out power above/below a certain value
        self.arm_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, True)
        self.arm_controller.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, True)
        self.arm_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_extension)
        self.arm_controller.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_extension)
        self.arm_controller.setSmartCurrentLimit(20)  # is 20 amps enough?
        self.pid_controller.setSmartMotionAllowedClosedLoopError(1)

        configure_sparkmax(sparkmax=self.arm_controller, pid_controller=self.pid_controller, slot=0, can_id=constants.k_arm_motor_port,
                           pid_dict=constants.k_PID_dict_vel_arm, pid_only=True, burn_flash=constants.k_burn_flash)
        configure_sparkmax(sparkmax=self.arm_controller, pid_controller=self.pid_controller, slot=1, can_id=constants.k_arm_motor_port,
                           pid_dict=constants.k_PID_dict_vel_arm_retract, pid_only=True, burn_flash=constants.k_burn_flash)
        # where are we when we start?  how do we stay closed w/o power?  do we leave pin in at power on?


        # initialize the extension of the arm - say it's fully closed at start
        self.extension = 0
        self.setpoint = self.extension  # initial setting should be ?
        self.sparkmax_encoder.setPosition(0)
        SmartDashboard.putNumber('arm_extension', self.extension)
        SmartDashboard.putNumber('arm_setpoint', self.setpoint)
        self.is_moving = False
        self.motion_log = [False] * 5
        self.movement_commanded = False

    def get_extension(self):  # getter for the relevant elevator parameter
        if wpilib.RobotBase.isReal():
            return self.sparkmax_encoder.getPosition()
        else:
            return self.extension

    def set_arm_extension(self, distance, mode='smartmotion', slot=0):
        if mode == 'smartmotion':
            # use smartmotion to send you there quickly
            self.pid_controller.setReference(distance, rev.CANSparkMax.ControlType.kSmartMotion, pidSlot=slot)
        elif mode == 'position':
            # just use the position PID
            self.pid_controller.setReference(distance, rev.CANSparkMax.ControlType.kPosition)

        self.movement_commanded = True  # keep track of if we have told the arm to move
        self.setpoint = distance
        SmartDashboard.putNumber('arm_setpoint', self.setpoint)

        if wpilib.RobotBase.isSimulation():
            self.extension = distance
            SmartDashboard.putNumber('arm_extension', self.extension)

    def set_encoder_position(self, distance):
        self.sparkmax_encoder.setPosition(distance)
        if wpilib.RobotBase.isSimulation():
            self.extension = distance

    def quiet_arm(self):  # the motor keeps oscillating after a smart motion
        # setting it to current locations seems to quiet it
        current_extension = self.get_extension()
        self.set_arm_extension(current_extension)

    def periodic(self) -> None:
        self.counter += 1
        if self.counter % 25 == 0:  # generic periodic updates twice a second
            self.extension = self.get_extension()
            SmartDashboard.putNumber('arm_extension', self.extension)
            self.is_moving = abs(self.sparkmax_encoder.getVelocity()) > 100  # for skipping through arm setpoints

            # TODO - see if arm motor is oscillating while trying to maintain a setpoint and quiet it.
            # seems to quiet down if you set the setpoint to the current location
            self.motion_log_counter += 1
            self.motion_log[self.motion_log_counter % len(self.motion_log)] = self.is_moving
            if any(self.motion_log):  # we've been moving at some point in the past few seconds
                pass  # nothing to check
            else:  # maybe we want to quiet the arm
                pass
                # if self.movement_commanded:  # we've been sitting still for a while and we've moved recently
                #     self.movement_commanded = False
                #     self.quiet_arm()

