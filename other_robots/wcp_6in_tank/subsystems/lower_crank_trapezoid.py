#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math
import wpilib
import commands2
import commands2.cmd
import wpimath.controller
import wpimath.trajectory
import rev

import constants
from misc.configure_controllers import configure_sparkmax


class LowerCrankArmTrapezoidal(commands2.TrapezoidProfileSubsystem):
    """A robot arm subsystem that moves with a motion profile."""

    # CrankArm should probably have four positions that we need to map out
    positions = {'intake': 45, 'shoot': 70, 'amp': 100, 'trap': 110}  # todo: set a rest?

    def __init__(self) -> None:
        super().__init__(
            wpimath.trajectory.TrapezoidProfile.Constraints(
                constants.k_crank_MaxVelocityRadPerSecond,
                constants.k_crank_MaxAccelerationRadPerSecSquared,
            ),
            constants.k_crank_kArmOffsetRads,  # neutral position, usually up (0.5) or down (-0.5)
        )

        self.feedforward = wpimath.controller.ArmFeedforward(
            constants.k_crank_kSVolts,
            constants.k_crank_kGVolts,
            constants.k_crank_kVVoltSecondPerRad,
            constants.k_crank_kAVoltSecondSquaredPerRad,
        )

        # ------------   2429 Additions to the template's __init__  ------------
        self.setName('crank_lower')
        self.counter = 0
        self.max_angle = 120 * math.pi / 180  # straight up is 90, call max allawable 120 degrees  todo: remeasure and verify
        self.min_angle = 45 * math.pi / 180  # do not close more than this
        self.is_moving = False  # may want to keep track of if we are in motion

        # initialize the motors
        self.motor = rev.CANSparkMax(constants.k_lower_crank_motor_right, rev.CANSparkMax.MotorType.kBrushless)
        self.follower = rev.CANSparkMax(constants.k_lower_crank_motor_left, rev.CANSparkMax.MotorType.kBrushless)
        self.follower.follow(self.motor,invert=True)  # now we only have to get one right
        self.sparks = [self.motor, self.follower]

        # drive the shooter crank entirely by the absolute encoder mounted to the right motor TODO: clean this
        self.abs_encoder = self.motor.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.abs_encoder.setInverted(False)  # verfied false by GAN on 20240210
        # crank abs encoder is 4:1 to the arm
        pos_factor = constants.k_lower_crank_abs_encoder_position_conversion_factor  # 2pi/4
        self.abs_encoder.setPositionConversionFactor(pos_factor)  # radians, with pulley ratio taken into account
        self.abs_encoder.setVelocityConversionFactor(pos_factor / 60)  # radians per second
        self.abs_encoder.setZeroOffset(pos_factor * 0.576)  # verified .576 by GAN on 20240210
        initial_position = [self.abs_encoder.getPosition() for i in range(5)]
        boot_message = f'{self.getName()} absolute encoder position at boot: {initial_position}'
        boot_message += f'set to {self.abs_encoder.getPosition() * 180 / math.pi:.1f} degrees'
        print(boot_message)
        self.angle = self.abs_encoder.getPosition()

        # configure our PID controller
        self.controller = self.motor.getPIDController()
        self.controller.setFeedbackDevice(self.abs_encoder)
        self.controller.setP(constants.k_crank_kP)  # P is pretty much all we need in the controller!

        # use the encoder for positioning in the sim
        self.sim_encoder = self.motor.getEncoder()

        # update controllers from constants file and optionally burn flash
        self.configure_motors()

    def useState(self, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:
        # Calculate the feedforward from the setpoint
        feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)

        # Add the feedforward to the PID output to get the motor output
        # TODO - check if the feedforward is correct in units for the sparkmax - documentation says 32, not 12
        self.controller.setReference(setpoint.position, rev.CANSparkMax.ControlType.kPosition, 0, arbFeedforward=feedforward)

        if wpilib.RobotBase.isSimulation():
            self.sim_encoder.setPosition(setpoint.position)


    def setArmGoalCommand(self, kArmOffsetRads: float) -> commands2.Command:
        return commands2.cmd.runOnce(lambda: self.setGoal(kArmOffsetRads), self)

    # ------------   2429 Additions to the template  ------------

    def enable(self):   # built-in function of the subsystem - turns on continuous motion profiling
        self.enable()

    def disable(self):  # built-in function of the subsystem - turns off continuous motion profiling
        self.disable()

    def move_rads(self, rads: float) -> commands2.Command:  # way to bump up and down for testing
        current_angle = self.get_angle()
        goal = current_angle + rads
        message = f'setting {self.getName()} from {current_angle:.1f} to {goal:.1f}'
        pc = commands2.PrintCommand(message)
        return commands2.cmd.runOnce(lambda: self.setGoal(goal), self).andThen(pc)

    def move_degrees(self, degrees: float) -> commands2.Command:  # way to bump up and down for testing
        current_angle = self.get_angle()
        goal = current_angle + degrees * math.pi / 180
        message = f'setting {self.getName()} from {current_angle*180/math.pi:.1f} to {goal*180/math.pi:.1f}'
        pc = commands2.PrintCommand(message)
        return commands2.cmd.runOnce(lambda: self.setGoal(goal), self).andThen(pc)

    def configure_motors(self):
        # move motor configuration out of __init__
        self.motor.setInverted(True)  # right motor CW moves crank arm down

        for spark in self.sparks:
            spark.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)

            # set soft limits - do not let spark max put out power above/below a certain value
            spark.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, constants.k_enable_soft_limits)
            spark.enableSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, constants.k_enable_soft_limits)
            spark.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kForward, self.max_angle)
            spark.setSoftLimit(rev.CANSparkMax.SoftLimitDirection.kReverse, self.min_angle)
            # spark.setSmartCurrentLimit(80)

            # set PIDs for position and velocoty, burn flash only on the second time through
            can_id = self.motor.getDeviceId()
            configure_sparkmax(sparkmax=self.motor, pid_controller=self.controller,
                               pid_dict=constants.k_PID_dict_pos_shooter_arm, can_id=can_id,
                               slot=0, pid_only=False, burn_flash=False)
            configure_sparkmax(sparkmax=self.motor, pid_controller=self.controller,
                               pid_dict=constants.k_PID_dict_vel_shooter_arm, can_id=can_id,
                               slot=1, pid_only=False, burn_flash=constants.k_burn_flash)

    def get_angle(self):  # getter for the relevant angles
        if wpilib.RobotBase.isReal():
            return self.abs_encoder.getPosition()
        else:
            return self.angle

    def stop_crank_arm(self):
        pass

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
