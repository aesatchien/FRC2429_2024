#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math
import json
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
    positions = {'intake': 60, 'shoot': 90, 'amp': 100, 'trap': 110}  # todo: set a rest?
    for key in positions.keys():  # convert to radians for the subsystem's internal math
        positions[key] *= math.pi / 180

    def __init__(self) -> None:
        self.config = constants.k_crank_arm_dict  # main difference between crank and shooter arms
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
        self.counter = 0
        self.max_angle = math.radians(self.config['max_angle'])   # straight up is 90, call max allawable 120 degrees  todo: remeasure and verify
        self.min_angle = math.radians(self.config['min_angle'])   # do not close more than this - angle seems to mess up at the bottom
        self.is_moving = False  # may want to keep track of if we are in motion

        # initialize the motors and keep a list of them for configuration later
        self.motor = rev.CANSparkMax(self.config['motor_can_id'], rev.CANSparkMax.MotorType.kBrushless)
        if self.config['k_motor_count'] > 1:
            self.follower = rev.CANSparkMax(self.config['follower_can_id'], rev.CANSparkMax.MotorType.kBrushless)
            self.follower.follow(self.motor, invert=True)  # now we only have to get one right
            self.sparks = [self.motor, self.follower]
        else:
            self.sparks = [self.motor]

        # drive the shooter crank entirely by the SPARKMAX encoder mounted to the right motor TODO: clean this
        self.abs_encoder = self.motor.getAbsoluteEncoder(encoderType=rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.abs_encoder.setInverted(True)  # clockwise on the current configuration cranks the arm up, but the encoder is mounted backwards
        self.abs_encoder.setPositionConversionFactor(1)  # stay absolute,
        self.abs_encoder.setVelocityConversionFactor(1 / 60)  # stay absolute
        # Cory says don't use this too confusing
        # self.abs_encoder.setZeroOffset(self.config['abs_encoder_zero_offset'])  # get abs encoder value near straight up (90deg) and call that its zero
        initial_position = sum([self.abs_encoder.getPosition() for i in range(5)]) / 5
        # crank abs encoder is 4:1 to the arm

        # todo: for the first time copy-paste the current abs encoder offset and add a flag that decides whether

        self.spark_encoder = self.motor.getEncoder()
        pos_factor = self.config['encoder_position_conversion_factor']  # 2pi / 300
        self.spark_encoder.setPositionConversionFactor(pos_factor)  # radians,
        self.spark_encoder.setVelocityConversionFactor(pos_factor / 60)  # radians per second

        # When we're at the limit switch, WE know the angle is 60 deg, and the abs encoder thinks we're at say 90 degrees.
        # We save the difference, 30 degrees.  Ie, we save (where abs thinks we are) - (where we know we are)
        # Next time we boot up, we're at 60 degrees, the abs encoder still thinks we're at 90 degrees
        # and we subtract the difference and we're happy

        # If we boot up at 100 deg, the abs encoder thinks we're at 130 degrees, and we subtract the difference
        # once more and we're happy.

        read_offset_from_file = False
        if read_offset_from_file:
            with open('abs_encoder_data.json', 'r') as abs_encoder_data:
                position_to_set_encoder = 2 * math.pi * (initial_position - json.load(abs_encoder_data)) / 3
        else:
            absolute_offset_from_90 = 2 * math.pi * (initial_position - self.config['abs_encoder_zero_offset']) / 3  # this is in radians, the 3 is to account for the gear ratio
            position_to_set_encoder = absolute_offset_from_90 + math.pi/2
        self.spark_encoder.setPosition(position_to_set_encoder)  # convert absolute encoder to a spot near pi/2 radians

        boot_message = f'{self.getName()} absolute encoder position at boot: {initial_position}'
        boot_message += f'set to {self.spark_encoder.getPosition() * 180 / math.pi:.1f} degrees'
        print(boot_message)
        self.angle = self.spark_encoder.getPosition()

        # configure our PID controller
        self.controller = self.motor.getPIDController()
        self.controller.setFeedbackDevice(self.spark_encoder)
        # self.controller.setFeedbackDevice(self.abs_encoder)  # do not do this
        self.controller.setP(self.config['k_kP'])  # P is pretty much all we need in the controller!
        self.controller.setPositionPIDWrappingEnabled(enable=True)
        self.controller.setPositionPIDWrappingMaxInput(math.pi)
        self.controller.setPositionPIDWrappingMinInput(-math.pi)

        self.limit_switch = wpilib.DigitalInput(constants.k_lower_crank_limit_switch_channel)

        # use the encoder for positioning in the sim
        self.sim_encoder = self.spark_encoder

        # update controllers from constants file and optionally burn flash
        self.configure_motors()

        self.goal = self.get_angle()
        self.set_goal(self.goal)  # do we want to do this?
        self.at_goal = True
        self.error = 0
        self.limit_switch_state = self.limit_switch.get()

        if wpilib.RobotBase.isReal():
            if self.goal > self.min_angle and self.goal < self.max_angle:
                self.enable_arm()
            else:
                self.disable_arm()
                print(f'arm out of allowed range disabling')

    def useState(self, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:
        # Calculate the feedforward from the setpoint
        # print("SETPOINT POSITION: " + str(math.degrees(setpoint.position)))
        feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)

        # Add the feedforward to the PID output to get the motor output
        # TODO - check if the feedforward is correct in units for the sparkmax - documentation says 32, not 12
        self.controller.setReference(setpoint.position, rev.CANSparkMax.ControlType.kPosition, 0,
                                     arbFeedforward=feedforward)
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

        # pc = commands2.PrintCommand(message)
        # return commands2.cmd.runOnce(lambda: self.set_goal(goal), self).andThen(pc)

    def configure_motors(self):
        # move motor configuration out of __init__
        # self.motor.setInverted(False)  # chain motor (facing left) CW moves crank arm positive
        self.motor.setInverted(True)  # gear motor (facing left) CW reverses big gear and moves crank arm NEGATIVE

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

    def set_encoder_position(self, position: float):
        """
        sets the encoder position to position and sets the goal to position (so we stay where we are).
        """
        self.spark_encoder.setPosition(position)
        self.sim_encoder.setPosition(position)
        self.angle = position
        self.set_goal(position)

    def set_voltage(self, volts):
        """
        Sets the voltage of the motor.  Only works if the motor is disabled.
        """
        if not self.enabled:
            self.controller.setReference(value=volts, ctrl=rev.CANSparkMax.ControlType.kVoltage)

    def get_angle(self):  # getter for the relevant angles
        # will always read in radians a value near 90 degrees plus or minus 45 - otherwise we are out of design
        # should be no other math to do for this system
        self.angle = self.spark_encoder.getPosition()
        if wpilib.RobotBase.isReal():
            pass
        else:  # figure out if we need to do something else in the sim
            pass
        return self.angle

    def stop_crank_arm(self):
        pass

    def check_goal(self, goal):
        if goal > self.max_angle or goal < self.min_angle:
            max_degrees = math.degrees(self.max_angle)
            min_degrees = math.degrees(self.min_angle)
            clamped_goal = constants.clamp(goal, bottom=self.min_angle, top=self.max_angle)
            message = f'** WARNING: ATTEMPT TO EXCEED {self.getName().upper()} LIMITS [{max_degrees:.0f},{min_degrees:.0f}] '
            message += f'with {math.degrees(goal):.0f} --> setting to {math.degrees(clamped_goal):.0f} **'
            print(message)
            goal = clamped_goal
        return goal

    def set_goal(self, goal):
        # make our own sanity-check on the subsystem's setGoal funtion
        self.goal = self.check_goal(goal)
        self.setGoal(self.goal)

    def get_at_goal(self) -> bool:
        self.at_goal = math.fabs(self.angle - self.goal) < math.radians(2)  # update it before returning it
        return self.at_goal

    def get_limit_switch_state(self) -> bool:
        if wpilib.RobotBase.isReal():
            self.limit_switch_state = self.limit_switch.get()
        else:
            self.limit_switch_state = self.angle > math.radians(constants.k_lower_crank_position_when_limit_switch_true_rad)
        return self.limit_switch_state

    def get_current(self):
        if wpilib.RobotBase.isReal():
            return self.motor.getOutputCurrent()
        else:
            return 6 if self.angle < math.radians(constants.k_crank_arm_dict['min_angle']) else 4

    def get_abs_encoder_reading(self):
        """
        Returns the absolute encoder's position with the conversion factors applied.
        """
        return self.abs_encoder.getPosition()

    def periodic(self) -> None:
        # What if we didn't call the below for a few cycles after we set the position?
        super().periodic()  # this does the automatic motion profiling in the background
        self.counter += 1
        if self.counter % 10 == 0:
            self.angle = self.get_angle()
            self.at_goal = math.fabs(self.angle - self.goal) < math.radians(2)  # maybe we want to call this an error
            self.error = self.angle - self.goal
            if wpilib.RobotBase.isReal():
                self.limit_switch_state = self.limit_switch.get()
            else:
                self.limit_switch_state = self.angle > math.radians(constants.k_lower_crank_position_when_limit_switch_true_rad)
            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_at_goal', self.at_goal)
            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_limit_switch', self.limit_switch_state)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_error', self.error)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_spark_pos', self.spark_encoder.getPosition())
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_rad_goal', self.goal)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_rads', self.angle)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_degrees', math.degrees(self.angle))
            if self.config['k_motor_count'] > 1:
                wpilib.SmartDashboard.putNumberArray(f'{self.getName()}_powers', [self.motor.getAppliedOutput(),
                                                                              self.follower.getAppliedOutput()])
            else:
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_power', self.motor.getAppliedOutput())
            self.is_moving = abs(self.abs_encoder.getVelocity()) > 0.01  # rad per second
            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_is_moving', self.is_moving)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_abs_encoder', self.abs_encoder.getPosition())


