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


class ShooterCrankArmTrapezoidal(commands2.TrapezoidProfileSubsystem):
    """A robot arm subsystem that moves with a motion profile."""
    
    # CrankArm should probably have four positions that we need to map out
    positions = {'intake': -90, 'shoot': -45, 'amp': 50, 'trap': 110}  # todo: set a rest?
    for key in positions.keys():
        positions[key] *= math.pi / 180


    def __init__(self) -> None:
        super().__init__(
            wpimath.trajectory.TrapezoidProfile.Constraints(
                constants.k_shooter_arm_MaxVelocityRadPerSecond,
                constants.k_shooter_arm_MaxAccelerationRadPerSecSquared,
            ),
            constants.k_shooter_arm_kArmOffsetRads,  # neutral position, usually up (0.5) or down (-0.5)
        )

        self.feedforward = wpimath.controller.ArmFeedforward(
            constants.k_shooter_arm_kSVolts,
            constants.k_shooter_arm_kGVolts,
            constants.k_shooter_arm_kVVoltSecondPerRad,
            constants.k_shooter_arm_kAVoltSecondSquaredPerRad,
        )

        # ------------   2429 Additions to the template's __init__  ------------
        self.setName('crank_upper')
        self.counter = 0
        self.max_angle = 120 * math.pi / 180  # straight up is 90, call max allawable 120 degrees  todo: remeasure and verify
        self.min_angle = -95 * math.pi / 180  # do not close more than this - angle seems to mess up at the bottom
        self.is_moving = False  # may want to keep track of if we are in motion

        # initialize the motors
        self.motor = rev.CANSparkMax(constants.k_top_crank_motor_right, rev.CANSparkMax.MotorType.kBrushless)
        self.follower = rev.CANSparkMax(constants.k_top_crank_motor_left, rev.CANSparkMax.MotorType.kBrushless)
        self.follower.follow(self.motor,invert=True)  # now we only have to get one right
        self.sparks = [self.motor, self.follower]

        # drive the shooter crank entirely by the absolute encoder mounted to the right motor TODO: clean this
        self.abs_encoder = self.motor.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.abs_encoder.setInverted(True)  # needs to be inverted - clockwise on the right arm cranks the shooter down
        # crank abs encoder is 1:1 to the arm
        pos_factor = constants.k_top_crank_abs_encoder_position_conversion_factor  # 2pi
        self.abs_encoder.setPositionConversionFactor(pos_factor)  # radians,
        self.abs_encoder.setVelocityConversionFactor(pos_factor / 60)  # radians per second
        self.abs_encoder.setZeroOffset(pos_factor * 0.417)  # TODO - reverify 0.171 is true down
        initial_position = [self.abs_encoder.getPosition() for i in range(5)]
        boot_message = f'{self.getName()} absolute encoder position at boot: {initial_position}'
        boot_message += f'set to {self.abs_encoder.getPosition() * 180 / math.pi:.1f} degrees'
        print(boot_message)
        self.angle = self.abs_encoder.getPosition()

        # configure our PID controller
        self.controller = self.motor.getPIDController()
        self.controller.setFeedbackDevice(self.abs_encoder)
        self.controller.setP(constants.k_shooter_arm_kP)  # P is pretty much all we need in the controller!
        self.controller.setPositionPIDWrappingEnabled(enable=True)
        self.controller.setPositionPIDWrappingMaxInput(math.pi)
        self.controller.setPositionPIDWrappingMinInput(-math.pi)

        # use the encoder for positioning in the sim
        self.sim_encoder = self.motor.getEncoder()

        # update controllers from constants file and optionally burn flash
        self.configure_motors()

        self.goal = self.get_angle()
        self.setGoal(self.goal)  # do we want to do this?
        self.enable()

    def useState(self, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:
        # Calculate the feedforward from the setpoint
        feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)

        # Add the feedforward to the PID output to get the motor output
        # TODO - check if the feedforward is correct in units for the sparkmax - documentation says 32, not 12
        self.controller.setReference(setpoint.position, rev.CANSparkMax.ControlType.kPosition, 0, arbFeedforward=feedforward)
        self.goal = setpoint.position

        if wpilib.RobotBase.isSimulation():
            self.sim_encoder.setPosition(setpoint.position)


    def setArmGoalCommand(self, kArmOffsetRads: float) -> commands2.Command:
        return commands2.cmd.runOnce(lambda: self.setGoal(kArmOffsetRads), self)

    # ------------   2429 Additions to the template  ------------

    def enable_arm(self):   # built-in function of the subsystem - turns on continuous motion profiling
        self.enable()

    def disable_arm(self):  # built-in function of the subsystem - turns off continuous motion profiling
        self.disable()

    def move_rads(self, rads: float) -> commands2.Command:  # way to bump up and down for testing
        current_angle = self.get_angle()
        goal = current_angle + rads
        message = f'setting {self.getName()} from {current_angle:.1f} to {goal:.1f}'
        pc = commands2.PrintCommand(message)
        cmd = commands2.cmd.InstantCommand()
        return commands2.cmd.runOnce(lambda: self.setGoal(goal), self).andThen(pc)

    def move_degrees(self, degrees: float) -> None:  # way to bump up and down for testing
        current_angle = self.get_angle()
        goal = current_angle + degrees * math.pi / 180
        message = f'setting {self.getName()} from {current_angle*180/math.pi:.1f} to {goal*180/math.pi:.1f}'
        print(message)
        self.setGoal(goal)
        #pc = commands2.PrintCommand(message)
        #return commands2.cmd.runOnce(lambda: self.setGoal(goal), self).andThen(pc)

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

    def set_brake_mode(self, mode='brake'):
        if mode == 'brake':
            brake_mode = rev.CANSparkBase.IdleMode.kBrake
        else:
            brake_mode = rev.CANSparkBase.IdleMode.kCoast

        for spark in self.sparks:
            spark.setIdleMode(brake_mode)

    def set_next_position(self, direction='up'):
        tolerance = 0.1
        angle = self.get_angle()
        if direction == 'up':
            allowed_positions = [value for value in self.positions.values() if value > angle + tolerance]
            temp_setpoint = sorted(allowed_positions)[0] if len(allowed_positions) > 0 else angle
        else:
            allowed_positions = [value for value in self.positions.values() if value < angle - tolerance]
            temp_setpoint = sorted(allowed_positions)[-1] if len(allowed_positions) > 0 else angle

        self.setGoal(temp_setpoint)

    def get_angle(self):  # getter for the relevant angles
        if wpilib.RobotBase.isReal():
            # keep angle between -pi/2 an 3pi/2  - is there another way to do this?
            arm_angle = self.abs_encoder.getPosition()
            if arm_angle > 1.0 * math.pi:
                arm_angle = arm_angle - 2 * math.pi
            return arm_angle
        else:
            return self.angle

    def stop_crank_arm(self):
        pass

    def periodic(self) -> None:
        super().periodic()  # this does the automatic motion profiling in the background
        self.counter += 1
        if self.counter % 10 == 0:
            self.angle = self.get_angle()
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_rad_goal', self.goal)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_rads', self.angle)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_degrees', self.angle*180/math.pi)
            wpilib.SmartDashboard.putNumberArray(f'{self.getName()}_powers', [self.motor.getAppliedOutput(),
                                           self.follower.getAppliedOutput()])
            self.is_moving = abs(self.abs_encoder.getVelocity()) > 0.01  # rad per second
            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_is_moving', self.is_moving)