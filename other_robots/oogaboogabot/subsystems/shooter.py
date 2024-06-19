# flywheel for 2024 bot
# started 2022 0102 to update to commands2
import math

import wpilib
from commands2 import Subsystem
from wpilib import SmartDashboard
import rev
from playingwithfusion import TimeOfFlight

import constants
from misc.configure_controllers import configure_sparkmax

class Shooter(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Shooter')
        self.counter = 5
        #self.PID_dict_vel = {'kP': 0.00021, 'kI': 0, 'kD': 0, 'kIz': 0, 'kFF': 0.000192}
        self.smartmotion_maxvel = 5001  # rpm
        self.smartmotion_maxacc = 5001
        self.current_limit = 35
        self.voltage = 5
        self.rpm = 1000
        self.rpm_list = [1000, 2000, 3000, 4000, 5000, 4000, 3000, 2000]  # cycle through these
        self.rpm_index = 0
        self.ring_loaded = False  # keep track of if we have the ring

        # initialize motors
        # looking from back to front
        motor_type = rev.CANSparkFlex.MotorType.kBrushless
        self.flywheel_lower_left = rev.CANSparkFlex(constants.k_flywheel_lower_left_neo_port, motor_type)
        self.flywheel_upper_left = rev.CANSparkFlex(constants.k_flywheel_upper_left_neo_port, motor_type)

        # the follower is inverted
        self.flywheel_lower_left.setInverted(False)   # False 2240228
        self.flywheel_upper_left.setInverted(False)    # False 2240228

        for spark in [self.flywheel_lower_left, self.flywheel_upper_left]:
            spark.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
            if constants.k_burn_flash:
                spark.burnFlash()

        # encoders
        self.flywheel_left_encoder = self.flywheel_lower_left.getEncoder()

        # controller
        self.flywheel_lower_left_controller = self.flywheel_lower_left.getPIDController()
        self.flywheel_lower_left_controller.setP(0.0001)
        self.flywheel_upper_left_controller = self.flywheel_upper_left.getPIDController()
        self.flywheel_upper_left_controller.setP(0.0001)
        self.kFF = 1.03 * 1 / 6784  # feed forward for a spark flex shooter
        self.flywheel_lower_left_controller.setFF(self.kFF, 0)
        self.flywheel_upper_left_controller.setFF(self.kFF, 0)

        # toggle state
        self.shooter_on = False
        SmartDashboard.putBoolean('shooter_on', self.shooter_on)

        # set up distance sensor
        self.shooter_height_sensor = TimeOfFlight(constants.k_shooter_timeoflight)
        self.shooter_height_sensor.setRangingMode(TimeOfFlight.RangingMode.kShort, 50)

        # TODO configure_sparkmax()
        # configure_sparkmax()

    def is_ring_loaded(self):
        # open shooter box seems to be about 380mm, so anything less indicates a ring
        if wpilib.RobotBase.isReal():
            self.ring_loaded = self.shooter_height_sensor.getRange() < 300
        else:
            pass  # handle this in the sim

        # if wpilib.RobotBase.isSimulation():  # test it
        #     self.ring_loaded = self.counter % 1000 < 500

        return self.ring_loaded

    def set_flywheel(self, rpm, volts=None, use_voltage=False):
        # self.flywheel_left_controller.setReference(rpm, rev.CANSparkLowLevel.ControlType.kSmartVelocity, 0)
        # self.shooter_voltage = self.shooter_voltage + 1 if self.shooter_voltage < 12 else 5  # CJH increment voltage test
        # self.shooter_rpm = self.shooter_rpm + 1000 if self.shooter_rpm < 5000 else 1000  # AEH increment rpm test

        if rpm is None:  # cycle through for testing or actually pass a value
            self.rpm_index += 1
            self.rpm = self.rpm_list[self.rpm_index % len(self.rpm_list)]
        else:
            self.rpm = rpm

        if use_voltage:
            self.voltage = volts
            self.flywheel_lower_left_controller.setReference(self.voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
            self.flywheel_upper_left_controller.setReference(self.voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
        else:
            self.flywheel_lower_left_controller.setReference(self.rpm, rev.CANSparkFlex.ControlType.kVelocity, 0)
            self.flywheel_upper_left_controller.setReference(self.rpm, rev.CANSparkFlex.ControlType.kVelocity, 0)
        #self.flywheel_left_controller.setReference(self.shooter_voltage, rev.CANSparkFlex.ControlType.kVoltage, 0)
        #self.flywheel_right_controller.setReference(self.shooter_voltage, rev.CANSparkLowLevel.ControlType.kVoltage, 0)
        self.shooter_on = True
        # print(f'setting rpm to {rpm} {self.voltage}')
        SmartDashboard.putBoolean('shooter_on', self.shooter_on)

    
    def stop_shooter(self):
        self.flywheel_lower_left_controller.setReference(0, rev.CANSparkFlex.ControlType.kVoltage)
        self.flywheel_upper_left_controller.setReference(0, rev.CANSparkFlex.ControlType.kVoltage)
        # self.flywheel_left_controller.setReference(0, rev.CANSparkFlex.ControlType.kVoltage)
        #self.flywheel_right_controller.setReference(0, rev.CANSparkLowLevel.ControlType.kVoltage)
        self.shooter_on = False
        self.voltage = 0  # CJH for 2024 testing
        self.rpm = 0
        SmartDashboard.putBoolean('shooter_on', self.shooter_on)

    def get_velocity(self):
        return self.flywheel_left_encoder.getVelocity()

    def get_at_velocity(self):
        return self.at_velocity


    def toggle_shooter(self, rpm=None):
        if self.shooter_on:
            self.stop_shooter()
        else:
            self.rpm = self.rpm if rpm is None else rpm
            self.set_flywheel(self.rpm)

    def periodic(self) -> None:
        
        self.counter += 1

        # SmartDashboard.putBoolean('shooter_enable', self.shooter_enable)
        if self.counter % 10 == 0:
            # not too often
            SmartDashboard.putNumber('shooter_rpm', self.flywheel_left_encoder.getVelocity())
            SmartDashboard.putNumber('shooter_rpm_target', self.rpm)
            velocity = self.get_velocity()
            self.at_velocity = math.fabs(velocity - self.rpm) < 300 and self.rpm > 100  # need to figure out this tolerance
            SmartDashboard.putBoolean('shooter_ready', self.at_velocity)
            # SmartDashboard.putNumber('shooter_current', self.flywheel_lower_left.getOutputCurrent())
            SmartDashboard.putNumber('shooter_output', self.flywheel_lower_left.getAppliedOutput())
            SmartDashboard.putNumber('shooter_tof', self.shooter_height_sensor.getRange())
            SmartDashboard.putBoolean('shooter_has_ring', self.is_ring_loaded())
