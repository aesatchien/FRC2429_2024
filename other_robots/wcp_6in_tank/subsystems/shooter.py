# drivetrain to use both in sim and robot mode - sim handles the Sparkmax now
# started 2022 0102 to update to commands2

from commands2 import SubsystemBase
from wpilib import SmartDashboard
import rev

import constants


class Shooter(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Shooter')
        self.counter = 0
        #self.PID_dict_vel = {'kP': 0.00021, 'kI': 0, 'kD': 0, 'kIz': 0, 'kFF': 0.000192}
        self.smartmotion_maxvel = 5001  # rpm
        self.smartmotion_maxacc = 5001
        self.current_limit = 35
        self.shooter_voltage = 12

        # initialize motors
        # looking from back to front
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.flywheel_left = rev.CANSparkMax(constants.k_flywheel_left_neo_port, motor_type)
        self.flywheel_right = rev.CANSparkMax(constants.k_flywheel_right_neo_port, motor_type)

        self.flywheel_left.setInverted(True) # inverted left so positive rpm is shooting
        #self.flywheel_right.setInverted(False)
        self.flywheel_right.follow(self.flywheel_left, invert=False)
        # the follower is inverted

        # encoders
        # self.flywheel_left_encoder = self.flywheel_left.getEncoder()

        # controller
        self.flywheel_left_controller = self.flywheel_left.getPIDController()
        self.flywheel_left_controller.setP(0)
       # self.flywheel_right_controller = self.flywheel_right.getPIDController()
       # self.flywheel_right_controller.setP(0)

        # toggle state
        self.shooter_enable = False
        SmartDashboard.putBoolean('shooter_state', self.shooter_enable)

        # self.set_pids()

    def set_flywheel(self, rpm):
        # self.flywheel_left_controller.setReference(rpm, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 0)
        self.shooter_voltage = 12  # CJH increment voltage test
        self.flywheel_left_controller.setReference(self.shooter_voltage, rev.CANSparkMaxLowLevel.ControlType.kVoltage, 0)
        #self.flywheel_right_controller.setReference(self.shooter_voltage, rev.CANSparkMaxLowLevel.ControlType.kVoltage, 0)
        self.shooter_enable = True
        print(f'setting rpm to {rpm} {self.shooter_voltage}')
        SmartDashboard.putBoolean('shooter_state', self.shooter_enable)

    
    def stop_shooter(self):
        self.flywheel_left_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        #self.flywheel_right_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
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

    def set_pids(self, burn_flash=True):
        self.error_dict = {}
        i = 0
        self.error_dict.update({'kP0_' + str(i): self.flywheel_left_controller.setP(self.PID_dict_vel['kP'], 0)})
        self.error_dict.update({'kI0_' + str(i): self.flywheel_left_controller.setI(self.PID_dict_vel['kI'], 0)})
        self.error_dict.update({'kIz0_' + str(i): self.flywheel_left_controller.setIZone(self.PID_dict_vel['kIz'], 0)})
        self.error_dict.update({'kD0_' + str(i): self.flywheel_left_controller.setD(self.PID_dict_vel['kD'], 0)})
        self.error_dict.update({'kD0_' + str(i): self.flywheel_left_controller.setFF(self.PID_dict_vel['kFF'], 0)})
        self.error_dict.update({'Accel0_' + str(i): self.flywheel_left_controller.setSmartMotionMaxVelocity(self.smartmotion_maxvel, 0)})  #
        self.error_dict.update({'Vel0_' + str(i): self.flywheel_left_controller.setSmartMotionMaxAccel(self.smartmotion_maxacc, 0)})



        # print(self.error_dict)
        if burn_flash:
            self.flywheel_left.burnFlash()

    def periodic(self) -> None:
        
        self.counter += 1

        SmartDashboard.putBoolean('shooter_enable', self.shooter_enable)
        # if self.counter % 20 == 0:
        #     # not too often
        #     SmartDashboard.putNumber('shooter_rpm', self.flywheel_left_encoder.getVelocity())
        #     SmartDashboard.putBoolean('shooter_ready', self.flywheel_left_encoder.getVelocity() > 1800)
        #     SmartDashboard.putNumber('shooter_current', self.flywheel_left.getOutputCurrent())
        #     SmartDashboard.putNumber('shooter_output', self.flywheel_left.getAppliedOutput())