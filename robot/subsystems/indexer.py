import math

import wpilib
from commands2 import Subsystem
from wpilib import SmartDashboard
import rev

import constants


class Indexer(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Indexer')
        self.counter = 2
        self.smartmotion_maxvel = 5001
        self.smartmotion_maxacc = 5001
        self.current_limit = 35
        self.indexer_voltage = 0.05

        motor_type = rev.CANSparkMax.MotorType.kBrushless
        self.motor = rev.CANSparkMax(constants.k_indexer_neo_port, motor_type)
        self.motor.setInverted(False)

        #encoder
        self.indexer_encoder = self.motor.getEncoder()

        #controller
        self.indexer_controller = self.motor.getPIDController()
        self.indexer_controller.setP(0.0001)
        self.kFF = 1.03 * 1/6784 # feed forward for a spark flex from shooter
        self.indexer_controller.setFF(self.kFF, 0)

        self.indexer_on = False

    def set_indexer(self, power):
            self.indexer_voltage = power * 12
            self.indexer_controller.setReference(self.indexer_voltage, rev.CANSparkMax.ControlType.kVoltage, 0)
            self.indexer_on = True if math.fabs(power) > 0.01 else False
            SmartDashboard.putBoolean('indexer_enabled', self.indexer_on)

    def stop_indexer(self):
        self.indexer_controller.setReference(0, rev.CANSparkMax.ControlType.kVoltage)
        self.indexer_voltage = 0
        self.indexer_on = False
        SmartDashboard.putBoolean('indexer_enabled', self.indexer_on)

    def get_indexer(self):
        return self.indexer_encoder.getVelocity()

    def toggle_indexer(self, power):
        if self.indexer_on:
            self.stop_indexer()
        else:
            self.set_indexer(power)

    def periodic(self) -> None:

        self.counter += 1

        if self.counter % 20 == 0:
            SmartDashboard.putNumber('indexer_rpm', self.indexer_encoder.getVelocity())
            # SmartDashboard.putBoolean('indexer_ready', self.indexer_encoder.getVelocity() > 1800)
            # SmartDashboard.putNumber('indexer_current', self.motor.getOutputCurrent())
            if wpilib.RobotBase.isReal():
                SmartDashboard.putNumber('indexer_output', 12 * self.motor.getAppliedOutput())
            else:
                SmartDashboard.putNumber('indexer_output', self.motor.getAppliedOutput())