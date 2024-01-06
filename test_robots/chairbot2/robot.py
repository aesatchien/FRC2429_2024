#!/usr/bin/env python3

import wpilib
import rev
import math

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
        # control mode takes values remote or onboard
        self.control_mode = 'onboard'

        #flow control
        self.counter = 0
        self.thrust = 0
        self.twist = 0
        self.thrust_limit = .25
        self.twist_limit = .33
        self.deadband = 0.1

    def teleopPeriodic(self) -> None:
        self.drive_l1.set(self.thrust + self.twist)
        self.drive_r1.set(self.thrust - self.twist)

    def robotPeriodic(self) -> None:
        self.counter += 1

        if self.control_mode == 'remote':
            self.thrust = self.joystick.getRawAxis(1) * -1 * self.thrust_limit
            self.twist = self.joystick.getRawAxis(4) * self.twist_limit
        elif self.control_mode == 'onboard':
            # direction = 1 if not self.left_button.get() else direction = -1 if not self.right_button.get() else direction = 0
            direction = 0
            if not self.forward_switch.get():
                direction = 1
            elif not self.back_switch.get():
                direction = -1
            # self.thrust = self.analog_pedal.getVoltage()
            self.thrust = direction * max(0,
            self.analog_pedal.getVoltage() - 0.8) / 2.4  # math to get thrust value between 0 and 1 from pedal
            self.thrust *= self.thrust_limit
            self.twist = (self.steering_encoder.get() - 0.608) / 0.225
            self.twist *= self.twist_limit
            self.twist = 0 if math.fabs(self.twist) < self.deadband else self.twist
            # self.thrust = 0 if math.fabs(self.thrust) < self.deadband else self.thrust


        else:
            pass

        if self.counter % 10 == 0:
            wpilib.SmartDashboard.putNumber('twist', self.twist)
            wpilib.SmartDashboard.putNumber('thrust', self.thrust)
            wpilib.SmartDashboard.putString('control_mode', self.control_mode)
            wpilib.SmartDashboard.putNumber('analog_pedal', self.analog_pedal.getVoltage())
            wpilib.SmartDashboard.putBoolean('forward_switch', self.forward_switch.get())
            wpilib.SmartDashboard.putBoolean('back_switch', self.back_switch.get())
            wpilib.SmartDashboard.putNumber('steering_encoder_abs', self.steering_encoder.getAbsolutePosition())
            wpilib.SmartDashboard.putNumber('steering_encoder', self.steering_encoder.get())





if __name__ == "__main__":
        wpilib.run(MyRobot)