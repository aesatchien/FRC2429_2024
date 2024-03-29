from commands2 import Subsystem
from wpilib import SmartDashboard
import rev

import constants


class Indexer(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Indexer')
        self.counter = 0
        self.smartmotion_maxvel = 5001
        self.smartmotion_maxacc = 5001
        self.current_limit = 35
        self.indexer_voltage = 0.05

        motor_type = rev.CANSparkMax.MotorType.kBrushless
        self.indexer_motor = rev.CANSparkMax(constants.k_indexer_neo_port, motor_type)

        #encoder
        self.indexer_encoder = self.indexer_motor.getEncoder()

        #controller
        self.indexer_controller = self.indexer_motor.getPIDController()
        self.indexer_controller.setP(0.0001)
        self.kFF = 1.03 * 1/6784 # feed forward for a spark flex from shooter
        self.indexer_controller.setFF(self.kFF, 0)

    def set_indexer(self,power):
            self.indexer_voltage = power * 12
            self.indexer_controller.setReference(self.indexer_voltage, rev.CANSparkMax.ControlType.kVoltage, 0)

    def stop_indexer(self):
        self.indexer_controller.setReference(0, rev.CANSparkMax.ControlType.kVoltage)
        self.indexer_voltage = 0

    def get_indexer(self):
        return self.indexer_encoder.getVelocity()

    def toggle_indexer(self, power):
        if self.indexer_voltage > power * 12:
            self.stop_indexer()
        else:
            self.set_indexer(power)

    def periodic(self) -> None:

        self.counter += 1

        if self.counter % 20 == 0:
            SmartDashboard.putNumber('indexer_rpm', self.indexer_encoder.getVelocity())
            SmartDashboard.putBoolean('indexer_ready', self.indexer_encoder.getVelocity() > 1800)
            SmartDashboard.putNumber('indexer_current', self.indexer_motor.getOutputCurrent())
            SmartDashboard.putNumber('indexer_output', self.indexer_motor.getAppliedOutput())