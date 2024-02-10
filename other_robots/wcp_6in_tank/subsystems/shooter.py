# drivetrain to use both in sim and robot mode - sim handles the Sparkmax now
# started 2022 0102 to update to commands2

from commands2 import Subsystem
from wpilib import SmartDashboard
import rev

import constants
from misc.configure_controllers import configure_sparkmax

class Shooter(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Shooter')
        self.counter = 0
        #self.PID_dict_vel = {'kP': 0.00021, 'kI': 0, 'kD': 0, 'kIz': 0, 'kFF': 0.000192}
        self.smartmotion_maxvel = 5001  # rpm
        self.smartmotion_maxacc = 5001
        self.current_limit = 35
        self.shooter_voltage = 5
        self.shooter_rpm = 1000

        # initialize motors
        # looking from back to front
        motor_type = rev.CANSparkFlex.MotorType.kBrushless
        self.flywheel_lower_left = rev.CANSparkFlex(constants.k_flywheel_lower_left_neo_port, motor_type)
        self.flywheel_upper_left = rev.CANSparkFlex(constants.k_flywheel_upper_left_neo_port, motor_type)

        # the follower is inverted
        self.flywheel_lower_left.setInverted(False)
        self.flywheel_upper_left.setInverted(True)

        # encoders
        self.flywheel_left_encoder = self.flywheel_lower_left.getEncoder()

        # controller
        self.flywheel_lower_left_controller = self.flywheel_lower_left.getPIDController()
        self.flywheel_lower_left_controller.setP(0)
        self.flywheel_upper_left_controller = self.flywheel_upper_left.getPIDController()
        self.flywheel_upper_left_controller.setP(0)
        self.kFF = 1 / 6784  # feed forward for a spark flex shooter
        self.flywheel_lower_left_controller.setFF(self.kFF, 0)
        self.flywheel_upper_left_controller.setFF(self.kFF, 0)

        # toggle state
        self.shooter_enable = False
        SmartDashboard.putBoolean('shooter_state', self.shooter_enable)

        # TODO configure_sparkmax()
        # configure_sparkmax()

    def set_flywheel(self, rpm):
        # self.flywheel_left_controller.setReference(rpm, rev.CANSparkLowLevel.ControlType.kSmartVelocity, 0)
        self.shooter_voltage = self.shooter_voltage + 1 if self.shooter_voltage < 12 else 5  # CJH increment voltage test
        self.shooter_rpm = self.shooter_rpm + 1000 if self.shooter_rpm < 5000 else 1000  # AEH increment rpm test
        use_voltage = False
        if use_voltage:
            self.flywheel_lower_left_controller.setReference(self.shooter_voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
            self.flywheel_upper_left_controller.setReference(self.shooter_voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
        else:
            self.flywheel_lower_left_controller.setReference(self.shooter_rpm, rev.CANSparkFlex.ControlType.kVelocity, 0)
            self.flywheel_upper_left_controller.setReference(self.shooter_rpm, rev.CANSparkFlex.ControlType.kVelocity, 0)
        #self.flywheel_left_controller.setReference(self.shooter_voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
        #self.flywheel_right_controller.setReference(self.shooter_voltage, rev.CANSparkLowLevel.ControlType.kVoltage, 0)
        self.shooter_enable = True
        print(f'setting rpm to {rpm} {self.shooter_voltage}')
        SmartDashboard.putBoolean('shooter_state', self.shooter_enable)

    
    def stop_shooter(self):
        self.flywheel_lower_left_controller.setReference(0, rev.CANSparkFlex.ControlType.kVoltage)
        self.flywheel_upper_left_controller.setReference(0, rev.CANSparkFlex.ControlType.kVoltage)
        # self.flywheel_left_controller.setReference(0, rev.CANSparkFlex.ControlType.kVoltage)
        #self.flywheel_right_controller.setReference(0, rev.CANSparkLowLevel.ControlType.kVoltage)
        self.shooter_enable = False
        self.shooter_voltage = 0  # CJH for 2024 testing
        SmartDashboard.putBoolean('shooter_state', self.shooter_enable)

    def get_flywheel(self):
        return self.flywheel_left_encoder.getVelocity()

    def toggle_shooter(self, rpm):
        if self.shooter_enable:
            self.stop_shooter()
        else:
            self.set_flywheel(rpm)

    def periodic(self) -> None:
        
        self.counter += 1

        # SmartDashboard.putBoolean('shooter_enable', self.shooter_enable)
        if self.counter % 20 == 0:
            # not too often
            SmartDashboard.putNumber('shooter_rpm', self.flywheel_left_encoder.getVelocity())
            SmartDashboard.putNumber('shooter_target_rpm', self.shooter_rpm)
            SmartDashboard.putBoolean('shooter_ready', self.flywheel_left_encoder.getVelocity() > 1800)
            SmartDashboard.putNumber('shooter_current', self.flywheel_lower_left.getOutputCurrent())
            SmartDashboard.putNumber('shooter_output', self.flywheel_lower_left.getAppliedOutput())