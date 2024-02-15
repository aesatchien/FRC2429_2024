#!/usr/bin/env python3
""" CJH testing the PID wrapping of the rev controller"""

import wpilib
import rev
import math
from wpimath.controller import PIDController



class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.counter = 0
        self.x = 0
        self.kp = 0.01

        self.motor = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()

        self.controller = self.motor.getPIDController()
        self.controller.setP(self.kp)  # P is pretty much all we need in the controller!
        self.controller.setPositionPIDWrappingEnabled(enable=True)
        self.controller.setPositionPIDWrappingMaxInput(180)
        self.controller.setPositionPIDWrappingMinInput(-180)
        self.controller.setReference(0, rev.CANSparkMax.ControlType.kPosition)

        self.pid = PIDController(self.kp, 0, 0)
        self.pid.setSetpoint(0)
        self.pid.enableContinuousInput(-120, 240)

    def robotPeriodic(self) -> None:

        self.counter += 1

        if self.counter % 2 == 0:
            self.x = self.counter % 360 # vary from -2pi to 2pi

            self.encoder.setPosition(self.x)
            output = self.motor.getAppliedOutput() / 12
            pid_output = self.pid.calculate(self.x)

            wpilib.SmartDashboard.putNumber('x', self.x)
            wpilib.SmartDashboard.putNumber('spark_output', output)
            wpilib.SmartDashboard.putNumber('pid output', pid_output)
            wpilib.SmartDashboard.putNumber('kp', self.kp)
