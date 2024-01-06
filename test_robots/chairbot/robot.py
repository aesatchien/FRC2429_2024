#!/usr/bin/env python3

import wpilib
import wpilib.drive
import rev
import math
from wpimath.filter import SlewRateLimiter


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.power_limit = 0.55  #.3
        self.joystick = wpilib.Joystick(0)
        self.counter = 0
        self.left_limiter = SlewRateLimiter(5)
        self.right_limiter = SlewRateLimiter(5)
        self.twist = 0
        self.thrust = 0

        self.left_button = wpilib.DigitalInput(3)
        self.right_button = wpilib.DigitalInput(2)
        self.analog_pedal = wpilib.AnalogInput(0)
        self.steering_joystick = wpilib.AnalogInput(1)

        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.drive_l1 = rev.CANSparkMax(1, motor_type)  #
        self.drive_l2 = rev.CANSparkMax(2, motor_type)  #
        self.drive_r1 = rev.CANSparkMax(3, motor_type)  #
        self.drive_r2 = rev.CANSparkMax(4, motor_type)

        self.encoder_l = self.drive_l1.getEncoder()  # c.f. wpilib.Encoder(2, 3)
        self.encoder_r = self.drive_r1.getEncoder()

        self.drive_l1.setInverted(True)  # need to see if + pwr moves motor clockwise and therefore robot fwd
        self.drive_l2.setInverted(True)
        self.drive_r1.setInverted(False)  # need to see if + pwr moves motor clockwise and therefore robot fwd
        self.drive_r2.setInverted(False)

        self.drive_l2.follow(self.drive_l1) # sets 2 to follow 1
        self.drive_r2.follow(self.drive_r1)

        # self.drive = wpilib.drive.DifferentialDrive(self.drive_l1, self.drive_r1)
        # self.drive.arcadeDrive()

        self.drive_mode = 'pedal_shifter'  # arcade, tank, or car
        self.control_mode = 'onboard' # remote or onboard

        self.left_input = None
        self.right_input = None



    def teleopPeriodic(self) -> None:

        # self.drive.arcadeDrive(self.joystick.getY(), -self.joystick.getX(), False)
        twist_factor = 0.75 #.6

        deadzone = 0.05

        if self.control_mode == 'remote':
            self.left_input = -1 * self.joystick.getRawAxis(1)  # 1 is a "y" axis - so this sets up positive & down negative
            if abs(self.left_input) < deadzone:
                self.left_input = 0
            right_axis = 5 if self.drive_mode == 'tank' else 4
            self.right_input = 0.5 * self.joystick.getRawAxis(right_axis)  # 5 is also a "y" axis
            self.left_input = self.left_limiter.calculate(self.left_input)
            self.right_input = self.right_limiter.calculate(self.right_input)
        # elif self.control_mode == 'onboard':
        #     self.left_input = not self.left_button.get()
        #     self.right_input = not self.right_button.get()
        else:
            if self.drive_mode == 'tank':
                self.drive_l1.set(self.left_input * power_limit)
                self.drive_r1.set(self.right_input * power_limit)
            elif self.drive_mode == 'arcade':
                self.drive_l1.set((self.left_input + self.right_input) * power_limit)
                self.drive_r1.set((self.left_input - self.right_input) * power_limit)
            elif self.drive_mode == 'car':
                self.thrust = max(0, self.analog_pedal.getVoltage()-0.9) / 4.1  # math to get thrust value between 0 and 1 from pedal
                left_twist = (not self.left_button.get()) - (not self.right_button.get())
                self.drive_l1.set((2 * self.thrust+left_twist) * power_limit)
                self.drive_r1.set((2 * self.thrust-left_twist) * power_limit)
            elif self.drive_mode == 'pedal_shifter':
                # direction = 1 if not self.left_button.get() else direction = -1 if not self.right_button.get() else direction = 0
                direction = 0
                if not self.left_button.get():
                    direction = 1
                elif not self.right_button.get():
                    direction = -1

                #center = 0.2
                steering_value = self.steering_joystick.getVoltage()
                self.twist = ((3.07 - self.steering_joystick.getVoltage()) /2.5)
                self.twist = math.copysign(0.5 * self.twist ** 2, self.twist)
                #if steering_value-center > 0.1:
                #    self.twist = twist_factor
                #elif steering_value < 0.05:
                #    self.twist = - twist_factor
                #else:
                #    self.twist = 0

                self.thrust = direction * max(0, self.analog_pedal.getVoltage() - 3.07) / 4.1  # math to get thrust value between 0 and 1 from pedal
                self.drive_l1.set((self.thrust + 0.4 * self.twist) * self.power_limit)
                self.drive_r1.set((self.thrust - 0.4 * self.twist) * self.power_limit)


    def robotPeriodic(self) -> None:
        self.counter += 1
        self.twist = (3.07 - self.steering_joystick.getVoltage()) / 2.5
        self.twist = math.copysign(0.5 * self.twist ** 2, self.twist)
        self.curr_power_limit = 0;
        if self.counter % 10 == 0:
            pass
            # wpilib.SmartDashboard.putNumber("Encoder L", self.encoder_l.get())
            # wpilib.SmartDashboard.putNumber("Encoder R", self.encoder_r.get())
            # wpilib.SmartDashboard.putNumber("Power L", self.left_input)
            # wpilib.SmartDashboard.putNumber("Power R", self.right_input)
            wpilib.SmartDashboard.putBoolean('forward', self.left_button.get())
            wpilib.SmartDashboard.putBoolean('back', self.right_button.get())
            wpilib.SmartDashboard.putNumber('pedal_analog', self.analog_pedal.getVoltage())
            wpilib.SmartDashboard.putNumber('steering_analog', self.steering_joystick.getVoltage())
            wpilib.SmartDashboard.putNumber('twist', self.twist)
            wpilib.SmartDashboard.putNumber('power_limit', self.power_limit)
            wpilib.SmartDashboard.putNumber('thrust', self.thrust)

        self.curr_power_limit = wpilib.SmartDashboard.getNumber('power_limit', defaultValue=0)
        if self.power_limit != self.curr_power_limit:
            self.power_limit = self.curr_power_limit



if __name__ == "__main__":
    wpilib.run(MyRobot)
