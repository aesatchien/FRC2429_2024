from commands2 import Subsystem
from wpilib import SmartDashboard
import rev

import constants


class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Intake')
        self.counter = 0
        self.current_limit = 35

        motor_type = rev.CANSparkFlex.MotorType.kBrushless
        self.intake_motor_left = rev.CANSparkFlex(constants.k_intake_neo_port, motor_type)

        self.encoder = self.intake_motor_left.getEncoder()
        self.intake_controller = self.intake_motor_left.getPIDController()
        self.intake_controller.setP(0)

        self.intake_enabled = False
        SmartDashboard.putBoolean('intake_enabled', self.intake_enabled)

    def set_intake_motor(self, rpm):
        self.intake_voltage = 5
        self.intake_controller.setReference(self.intake_voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
        self.intake_enabled = True
        print(f'setting rpm / voltage to {rpm} {self.intake_voltage}')
        SmartDashboard.putBoolean('intake_enabled', self.intake_enabled)

    def stop_intake(self):
        self.intake_controller.setReference(0, rev.CANSparkFlex.ControlType.kVoltage)
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

    def set_pids(self, burn_flash=True):
        self.error_dict = {}
        i = 0
        self.error_dict.update({'kP0_' + str(i): self.intake_controller.setP(self.PID_dict_vel['kP'], 0)})
        self.error_dict.update({'kI0_' + str(i): self.intake_controller.setI(self.PID_dict_vel['kI'], 0)})
        self.error_dict.update(
            {'kIz0_' + str(i): self.intake_controller.setIZone(self.PID_dict_vel['kIz'], 0)})
        self.error_dict.update({'kD0_' + str(i): self.intake_controller.setD(self.PID_dict_vel['kD'], 0)})
        self.error_dict.update(
            {'kD0_' + str(i): self.intake_controller.setFF(self.PID_dict_vel['kFF'], 0)})
        self.error_dict.update({'Accel0_' + str(i): self.intake_controller.setSmartMotionMaxVelocity(
            self.smartmotion_maxvel, 0)})  #
        self.error_dict.update(
            {'Vel0_' + str(i): self.intake_controller.setSmartMotionMaxAccel(self.smartmotion_maxacc, 0)})

        if burn_flash:
            self.intake_motor_left.burnFlash()

    def periodic(self) -> None:
        self.counter += 1

        # todo - show intake rpm
        #SmartDashboard.putBoolean('intake_enabled', self.intake_enabled)