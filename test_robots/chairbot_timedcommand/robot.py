#!/usr/bin/env python3
# chairbot_timedrobot - using the simple wpilib.TimedRobot
# this is a demonstration robot on how to use the basics of wpilib
# after this, upgrade to a TimedCommandRobot template and move all code to appropriate subsystems

import wpilib
from wpimath.filter import SlewRateLimiter
import rev
import math
import ntcore
import commands2

class MyRobot(wpilib.TimedRobot):

    def robotInit(self) -> None:
        # hardware
        self.joystick = wpilib.Joystick(0)
        # chairbot human interface
        self.forward_switch = wpilib.DigitalInput(3)
        self.back_switch = wpilib.DigitalInput(2)
        self.analog_pedal = wpilib.AnalogInput(0)
        self.steering_encoder = wpilib.DutyCycleEncoder(6)
        # motors
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.drive_l1 = rev.CANSparkMax(1, motor_type)
        self.drive_l2 = rev.CANSparkMax(2, motor_type)
        self.drive_r1 = rev.CANSparkMax(3, motor_type)
        self.drive_r2 = rev.CANSparkMax(4, motor_type)

        self.encoder_l = self.drive_l1.getEncoder()  # c.f. wpilib.Encoder(2, 3)
        self.encoder_r = self.drive_r1.getEncoder()

        self.drive_l1.setInverted(True)  # need to see if + pwr moves motor clockwise and therefore robot fwd
        self.drive_l2.setInverted(True)
        self.drive_r1.setInverted(False)  # need to see if + pwr moves motor clockwise and therefore robot fwd
        self.drive_r2.setInverted(False)

        self.drive_l2.follow(self.drive_l1)  # sets 2 to follow 1
        self.drive_r2.follow(self.drive_r1)

        # class variables
        self.counter = 0
        self.thrust = 0
        self.previous_thrust = 0  # used if we are limiting acceleration by limiting a change on the input
        self.twist = 0
        # set limits on how fast we can go in thrust (forward/reverse) and twist (turning)
        self.thrust_limit = .25
        self.twist_limit = .33
        self.deadband = 0.1
        # rate limit calculators
        self.limit_type = "slewrate"
        if self.limit_type == "slewrate":  # if we use wpilib's slew rate limiter
            self.max_thrust_change = 0.5  # slew rate limiters take units of 100's of percent per second (weird)
        else:  # if we calculate things ourselves by calculating the delta
            self.max_thrust_change = 0.04  # allowed change in thrust per robot cycle, in terms of power output
        self.max_twist_change = 0.5  # only demonstrating slewratelimiter on twist
        self.thrust_slew_rate_limiter = SlewRateLimiter(self.max_thrust_change, -self.max_thrust_change)
        self.twist_slew_rate_limiter = SlewRateLimiter(self.max_thrust_change, -self.max_thrust_change)

        # control mode takes values remote or onboard
        self.control_mode = 'remote'
        wpilib.SmartDashboard.putString('control_mode', self.control_mode)
        wpilib.SmartDashboard.putNumber('thrust_limit', self.thrust_limit)
        wpilib.SmartDashboard.putNumber('twist_limit', self.twist_limit)

        # networktables section - using the NT4 version from 2023+
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable("datatable")
        # publishers - call the set() function to write them
        self.thrust_limit_pub = self.table.getDoubleTopic("thrust_limit").publish()
        self.max_thrust_change_pub = self.table.getDoubleTopic("max_thrust_change").publish()
        self.twist_limit_pub = self.table.getDoubleTopic("twist_limit").publish()
        self.control_mode_pub = self.table.getStringTopic("control_mode").publish()
        self.thrust_limit_pub.set(self.thrust_limit)
        self.max_thrust_change_pub.set(self.max_thrust_change)
        self.twist_limit_pub.set(self.twist_limit)
        self.control_mode_pub.set(self.control_mode)
        # subscribers - call the get() function to read them
        self.thrust_limit_sub = self.table.getDoubleTopic("thrust_limit").subscribe(self.thrust_limit)
        self.max_thrust_change_sub = self.table.getDoubleTopic("max_thrust_change").subscribe(self.max_thrust_change)
        self.twist_limit_sub = self.table.getDoubleTopic("twist_limit").subscribe(self.twist_limit)
        self.control_mode_sub = self.table.getStringTopic("control_mode").subscribe(self.control_mode)

    def teleopPeriodic(self) -> None:
        # teleop is quite simple for this robot - just set the motors
        self.drive_l1.set(self.thrust + self.twist)
        self.drive_r1.set(self.thrust - self.twist)

    def robotPeriodic(self) -> None:
        # here we do all the checking of the robot state - read inputs, calculate outputs
        self.counter += 1
        # calculate outputs based on if we care controlled by driver station joystick or human in chairbot
        if self.control_mode == 'remote':  # control from driver station joystick
            self.thrust = self.joystick.getRawAxis(1) * -1 * self.thrust_limit
            self.twist = self.joystick.getRawAxis(4) * self.twist_limit
        elif self.control_mode == 'onboard':  # control from the human sitting on chairbot
            # determine if we are moving forward, backward, or in neutral based on 3-position switch
            direction = 0
            if not self.forward_switch.get():
                direction = 1
            elif not self.back_switch.get():
                direction = -1
            # get thrust from the analog pedal.  It rests at 0.8V and goes to 3.2V when floored
            # to map this from 0 to 1, subtract 0.8 and divide by (3.2-0.8)
            self.thrust = direction * max(0, self.analog_pedal.getVoltage() - 0.8) / 2.4  # math to get thrust value between 0 and 1 from pedal
            self.thrust *= self.thrust_limit
            # analog encoder is at 0.61 at neutral position (not turning) and travels .225 above and below that
            # so to scale from -1 to 1, subtract the central value and divide by .225
            self.twist = (self.steering_encoder.get() - 0.608) / 0.225
            self.twist *= self.twist_limit
            self.twist = 0 if math.fabs(self.twist) < self.deadband else self.twist
            # self.thrust = 0 if math.fabs(self.thrust) < self.deadband else self.thrust
        else:  # end of control mode
            pass  # no other control modes defined

        # enforce acceleration limit
        if self.limit_type == "slewrate":  # let the wpilib rate limiters do all the work
            self.thrust = self.thrust_slew_rate_limiter.calculate(self.thrust)
            self.twist = self.twist_slew_rate_limiter.calculate(self.twist)
        else:  # we calculate the changes manually and limit them to a pre-determined delta
            thrust_change = self.thrust - self.previous_thrust
            if math.fabs(thrust_change) > self.max_thrust_change:
                self.thrust = self.previous_thrust + self.max_thrust_change * math.copysign(1, thrust_change)
                print(f'limiting thrust to {self.thrust}')
        self.previous_thrust = self.thrust

        if self.counter % 10 == 0:
            # read networktables for updates to our mode/limit values
            nt_control_mode = self.control_mode_sub.get()
            if nt_control_mode != self.control_mode:
                if nt_control_mode in ['remote', 'onboard']:
                    self.control_mode = nt_control_mode
                    print(f'setting control_mode to {self.control_mode}')
                    wpilib.SmartDashboard.putString('control_mode', self.control_mode)
                else:
                    pass

            nt_thrust_limit = self.thrust_limit_sub.get()
            if nt_thrust_limit != self.thrust_limit:
                if 0.1 < nt_thrust_limit <= 1:
                    self.thrust_limit = nt_thrust_limit
                    print(f'setting thrust_limit to {self.thrust_limit}')
                    wpilib.SmartDashboard.putNumber('thrust_limit', self.thrust_limit)
                else:
                    self.thrust_limit_pub.set(self.thrust_limit)

            nt_max_thrust_change = self.max_thrust_change_sub.get()
            if nt_max_thrust_change != self.max_thrust_change:
                if 0.02 <= nt_max_thrust_change <= 10:
                    self.max_thrust_change = nt_max_thrust_change
                    print(f'setting max_thrust_change to {self.max_thrust_change}')
                    wpilib.SmartDashboard.putNumber('max_thrust_change', self.max_thrust_change)
                    # reset the slew rate limiter with the new value (if we are in this limit mode)
                    self.thrust_slew_rate_limiter = SlewRateLimiter(self.max_thrust_change, -self.max_thrust_change)
                else:
                    self.max_thrust_change_pub.set(self.max_thrust_change)

            nt_twist_limit = self.twist_limit_sub.get()
            if nt_twist_limit != self.twist_limit:
                if 0.1 < nt_twist_limit <= 1:
                    self.twist_limit = nt_twist_limit
                    print(f'setting twist_limit to {self.twist_limit}')
                    wpilib.SmartDashboard.putNumber('twist_limit', self.twist_limit)
                else:
                    self.twist_limit_pub.set(self.twist_limit)

            # update the SmartDashboard
            wpilib.SmartDashboard.putNumber('twist', self.twist)
            wpilib.SmartDashboard.putNumber('thrust', self.thrust)
            wpilib.SmartDashboard.putNumber('analog_pedal', self.analog_pedal.getVoltage())
            wpilib.SmartDashboard.putBoolean('forward_switch', self.forward_switch.get())
            wpilib.SmartDashboard.putBoolean('back_switch', self.back_switch.get())
            wpilib.SmartDashboard.putNumber('steering_encoder_abs', self.steering_encoder.getAbsolutePosition())
            wpilib.SmartDashboard.putNumber('steering_encoder', self.steering_encoder.get())


if __name__ == "__main__":
    wpilib.run(MyRobot)