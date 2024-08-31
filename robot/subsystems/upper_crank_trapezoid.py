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


class UpperCrankArmTrapezoidal(commands2.TrapezoidProfileSubsystem):
    """A robot arm subsystem that moves with a motion profile."""
    
    # CrankArm should probably have four positions that we need to map out
    positions = {'intake': -78, 'shoot': -55, 'shoot2': -35, 'amp': 50, 'trap': 110}  # todo: set a rest?
    for key in positions.keys():  # convert to radians for the subsystem's internal math
        positions[key] *= math.pi / 180


    def __init__(self) -> None:
        self.config = constants.k_shooter_arm_dict  # main difference between crank and shooter arms
        super().__init__(
            wpimath.trajectory.TrapezoidProfile.Constraints(
                self.config['k_MaxVelocityRadPerSecond'],
                self.config['k_MaxAccelerationRadPerSecSquared'],
            ),
            self.config['k_kArmOffsetRads'],  # neutral position, usually up (1.57) or down (-1.57)?
        )

        self.feedforward = wpimath.controller.ArmFeedforward(
            self.config['k_kSVolts'],
            self.config['k_kGVolts'],
            self.config['k_kVVoltSecondPerRad'],
            self.config['k_kAVoltSecondSquaredPerRad'],
        )

        # ------------   2429 Additions to the template's __init__  ------------
        self.setName(self.config['name'])
        self.counter = 7
        self.max_angle = self.config['max_angle'] * math.pi / 180  # straight up is 90, call max allawable 120 degrees  todo: remeasure and verify
        self.min_angle = self.config['min_angle'] * math.pi / 180  # do not close more than this - angle seems to mess up at the bottom
        self.is_moving = False  # may want to keep track of if we are in motion
        self.tolerance = math.radians(45)  # degrees  # updated from 2 to 4 to 8 at Tempe- 20240319 CJH

        # initialize the motors and keep a list of them for configuration later
        self.motor = rev.CANSparkMax(self.config['motor_can_id'], rev.CANSparkMax.MotorType.kBrushless)
        self.follower = rev.CANSparkMax(self.config['follower_can_id'], rev.CANSparkMax.MotorType.kBrushless)
        self.follower.follow(self.motor,invert=True)  # now we only have to get one right
        self.sparks = [self.motor, self.follower]

        # drive the shooter crank entirely by the ABSOLUTE encoder mounted to the right motor TODO: clean this
        self.abs_encoder = self.motor.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.abs_encoder.setInverted(True)  # needs to be inverted - clockwise on the right arm cranks the shooter down
        # crank abs encoder is 1:1 to the arm
        pos_factor = self.config['encoder_position_conversion_factor']  # 2pi
        self.abs_encoder.setPositionConversionFactor(pos_factor)  # radians,
        self.abs_encoder.setVelocityConversionFactor(pos_factor / 60)  # radians per second
        self.abs_encoder.setZeroOffset(pos_factor * self.config['abs_encoder_zero_offset'])  # TODO - get quick recal procedure
        initial_position = sum([self.abs_encoder.getPosition() for i in range(5)]) / 5
        boot_message = f'{self.getName()} absolute encoder position at boot: {initial_position}'
        boot_message += f'set to {self.abs_encoder.getPosition() * 180 / math.pi:.1f} degrees'
        print(boot_message)
        self.angle = self.abs_encoder.getPosition()

        # configure our PID controller
        self.controller = self.motor.getPIDController()
        self.controller.setFeedbackDevice(self.abs_encoder)
        self.controller.setP(self.config['k_kP'])  # P is pretty much all we need in the controller!
        self.controller.setPositionPIDWrappingEnabled(enable=True)
        self.controller.setPositionPIDWrappingMaxInput(math.pi)
        self.controller.setPositionPIDWrappingMinInput(-math.pi)

        # use the encoder for positioning in the sim
        self.sim_encoder = self.motor.getEncoder()
        self.sim_encoder.setPositionConversionFactor(pos_factor)  # radians,
        self.sim_encoder.setVelocityConversionFactor(pos_factor / 60)  # radians per second
        initial_position = initial_position if wpilib.RobotBase.isReal() else math.radians(-85)
        self.sim_encoder.setPosition(initial_position)

        # update controllers from constants file and optionally burn flash
        self.configure_motors()

        self.goal = self.get_angle()
        self.set_goal(self.goal)  # do we want to do this?
        self.at_goal = True
        self.enable()
        
        self.disable()
        self.set_brake_mode("coast")


    def useState(self, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:
        # Calculate the feedforward from the setpoint
        feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)

        # Add the feedforward to the PID output to get the motor output
        # TODO - check if the feedforward is correct in units for the sparkmax - documentation says 32, not 12
        self.controller.setReference(setpoint.position, rev.CANSparkMax.ControlType.kPosition, 0, arbFeedforward=feedforward)
        # self.goal = setpoint.position  # don't want this - unless we want to plot the trapezoid

        if wpilib.RobotBase.isSimulation():
            self.sim_encoder.setPosition(setpoint.position)


    def setArmGoalCommand(self, kArmOffsetRads: float) -> commands2.Command:
        return commands2.cmd.runOnce(lambda: self.set_goal(kArmOffsetRads), self)

    # ------------   2429 Additions to the template  ------------

    def enable_arm(self):  # built-in function of the subsystem - turns on continuous motion profiling
        self.enable()  # call the underlying enable / disable
        self.enabled = True
        wpilib.SmartDashboard.putBoolean(f'{self.getName()}_is_enabled', self.enabled)

    def disable_arm(self):  # built-in function of the subsystem - turns off continuous motion profiling
        self.disable()  # call the underlying enable / disable
        self.disable()  # call the underlying enable / disable
        self.enabled = False
        wpilib.SmartDashboard.putBoolean(f'{self.getName()}_is_enabled', self.enabled)

    def is_enabled(self):
        return self.enabled

    def move_degrees(self, degrees: float, silent=False) -> None:  # way to bump up and down for testing
        current_angle = self.get_angle()
        goal = current_angle + math.radians(degrees)
        self.set_goal(goal)  # check and set
        message = f'setting {self.getName()} from {math.degrees(current_angle):.0f} to {math.degrees(self.goal):.0f}'
        if not silent:
            print(message)

        #pc = commands2.PrintCommand(message)
        #return commands2.cmd.runOnce(lambda: self.set_goal(goal), self).andThen(pc)


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
                               slot=0, pid_only=False, burn_flash=constants.k_burn_flash)
            # configure_sparkmax(sparkmax=self.motor, pid_controller=self.controller,
            #                    pid_dict=constants.k_PID_dict_vel_shooter_arm, can_id=can_id,
            #                    slot=1, pid_only=False, burn_flash=constants.k_burn_flash)

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

        temp_setpoint = self.check_goal(temp_setpoint)
        self.set_goal(temp_setpoint)

    def set_crank_preset(self, preset) -> None:
        self.preset = preset

    def get_angle(self):  # getter for the relevant angles
        self.angle = self.abs_encoder.getPosition()
        if wpilib.RobotBase.isReal():
            pass
            # TODO - does this affect the subsystem - we wrapped the controller already, so this should just track it
            # keep angle between -pi/2 an 3pi/2  - is there another way to do this?  Should this exactly match the PID wrapper?
            if self.angle > 1.25 * math.pi:
                self.angle = self.angle - 2 * math.pi
        else:  # figure out if we need to do something else in the sim
            self.angle = self.sim_encoder.getPosition()
        return self.angle

    def stop_crank_arm(self):
        pass

    def check_goal(self, goal):
        if goal > self.max_angle or goal < self.min_angle:
            max_degrees = math.degrees(self.max_angle)
            min_degrees = math.degrees(self.min_angle)
            clamped_goal = constants.clamp(goal, bottom=self.min_angle, top=self.max_angle)
            message = f'** WARNING: ATTEMPT TO EXCEED {self.getName().upper()} LIMITS [{max_degrees:.0f},{min_degrees:.0f}] '
            message += f'with {math.degrees(goal):.0f} **  setting to -> {math.degrees(clamped_goal):.0f}'
            print(message)
            goal = clamped_goal
        return goal

    def set_goal(self, goal):
        # make our own sanity-check on the subsystem's setGoal funtion
        self.goal = self.check_goal(goal)
        self.setGoal(self.goal)

    def get_at_goal(self) -> bool:
        # Update at_goal before returning it
        self.angle = self.get_angle()
        self.at_goal = math.fabs(self.angle - self.goal) < self.tolerance
        return self.at_goal

    def periodic(self) -> None:
        super().periodic()  # this does the automatic motion profiling in the background
        self.counter += 1
        if self.counter % 10 == 0:
            self.angle = self.get_angle()
            self.at_goal = math.fabs(self.angle - self.goal) < self.tolerance  # maybe we want to call this an error
            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_at_goal', self.at_goal)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_rad_goal', self.goal)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_rads', self.angle)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_degrees', math.degrees(self.angle))
            wpilib.SmartDashboard.putNumberArray(f'{self.getName()}_powers', [self.motor.getAppliedOutput(),
                                           self.follower.getAppliedOutput()])
            self.is_moving = abs(self.abs_encoder.getVelocity()) > 0.01  # rad per second
            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_is_moving', self.is_moving)


