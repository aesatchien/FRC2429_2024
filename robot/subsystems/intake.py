import math

import wpilib
from commands2 import Subsystem
from wpilib import SmartDashboard
import rev

import constants


class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Intake')
        self.counter = 3

        motor_type = rev.CANSparkMax.MotorType.kBrushless
        self.motor = rev.CANSparkMax(constants.k_intake_neo_port, motor_type)
        self.motor.setInverted(True)

        self.encoder = self.motor.getEncoder()

        self.intake_controller = self.motor.getPIDController()
        self.intake_controller.setP(0)

        self.intake_on = False
        SmartDashboard.putBoolean('intake_enabled', self.intake_on)
    def set_intake(self, power):
        self.intake_voltage = 12 * power
        self.intake_controller.setReference(self.intake_voltage, rev.CANSparkMax.ControlType.kVoltage, 0)
        self.intake_on = True if math.fabs(power) > 0.01 else False
        # print(f'setting power to {power} {self.intake_voltage}')
        SmartDashboard.putBoolean('intake_enabled', self.intake_on)

    def stop_intake(self):
        self.intake_controller.setReference(0, rev.CANSparkMax.ControlType.kVoltage)
        self.intake_on = False
        self.intake_voltage = 0
        SmartDashboard.putBoolean('intake_enabled', self.intake_on)

    def get_intake_velocity(self):
        return self.encoder.getVelocity()

    def toggle_intake(self, power):
        if self.intake_on:
            self.stop_intake()
        else:
            self.set_intake(power)

    def periodic(self) -> None:
        self.counter += 1
        if self.counter % 20 == 0:
            if wpilib.RobotBase.isReal():
                SmartDashboard.putNumber('intake_output', 12 * self.motor.getAppliedOutput())
            else:
                SmartDashboard.putNumber('intake_output', self.motor.getAppliedOutput())
        # todo - show intake rpm
        #SmartDashboard.putBoolean('intake_enabled', self.intake_enabled)