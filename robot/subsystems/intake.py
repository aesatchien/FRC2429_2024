from commands2 import Subsystem
from wpilib import SmartDashboard
import rev

import constants


class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Intake')
        self.counter = 0

        motor_type = rev.CANSparkMax.MotorType.kBrushless
        self.motor = rev.CANSparkMax(constants.k_intake_neo_port, motor_type)
        self.motor.setInverted(True)

        self.encoder = self.motor.getEncoder()

        self.intake_controller = self.motor.getPIDController()
        self.intake_controller.setP(0)

        self.intake_enabled = False
        SmartDashboard.putBoolean('intake_enabled', self.intake_enabled)
    def set_intake_motor(self, rpm):
        # misleading- rpm does nothing!
        self.intake_voltage = 5
        self.intake_controller.setReference(self.intake_voltage, rev.CANSparkMax.ControlType.kVoltage, 0)
        self.intake_enabled = True
        print(f'setting rpm / voltage to {rpm} {self.intake_voltage}')
        SmartDashboard.putBoolean('intake_enabled', self.intake_enabled)

    def set_intake_motor_volts(self, volts):
        self.intake_voltage = volts
        self.intake_controller.setReference(self.intake_voltage, rev.CANSparkMax.ControlType.kVoltage, 0)
        self.intake_enabled = True
        print(f'setting VOLTAGE to {self.intake_voltage}')
        SmartDashboard.putBoolean('intake_enabled', self.intake_enabled)

    def stop_intake(self):
        self.intake_controller.setReference(0, rev.CANSparkMax.ControlType.kVoltage)
        self.intake_enabled = False
        self.intake_voltage = 0
        SmartDashboard.putBoolean('intake_enabled', self.intake_enabled)

    def get_intake_velocity(self):
        return self.encoder.getVelocity()

    def toggle_intake(self, rpm):
        if self.intake_enabled:
            self.stop_intake()
        else:
            self.set_intake_motor(rpm)

    def periodic(self) -> None:
        self.counter += 1

        # todo - show intake rpm
        #SmartDashboard.putBoolean('intake_enabled', self.intake_enabled)