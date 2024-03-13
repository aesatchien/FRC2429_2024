from commands2 import Subsystem
import wpilib
from wpilib import SmartDashboard
import rev

import constants


class Climber(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Climber')
        self.counter = 0
        self.smartmotion_maxvel = 5001
        self.smartmotion_maxacc = 5001
        self.current_limit = 35
        self.climber_voltage = 5
        self.climber_rpm = 1000

        #initialize motors
        motor_type = rev.CANSparkMax.MotorType.kBrushless
        self.left_winch = rev.CANSparkMax(constants.k_left_winch_neo_port, motor_type)
        self.follower_winch = rev.CANSparkMax(constants.k_follower_winch_neo_port, motor_type)
        # self.follower_winch.follow(self.left_winch)

        self.left_winch.setInverted(True)
        self.follower_winch.setInverted(False)

        # #encoder
        # self.left_winch_encoder = self.left_winch.getEncoder()

        # #controller
        # self.left_winch_controller = self.left_winch.getPIDController()
        # self.left_winch_controller.setP(0.0001)
        # self.kFF = 1.03 * 1/6784 # feed forward for a spark flex from shooter
        # self.left_winch_controller.setFF(self.kFF, 0)
        # self.left_winch_controller.setFF(self.kFF, 0)

        self.left_servo = wpilib.Servo(constants.k_left_servo_port)
        self.right_servo = wpilib.Servo(constants.k_right_servo_port)

        # toggle state
        self.climber_enable = False
        self.servos_open = False

        self.open_servos() #Drew wanted servoes to be "open" on startup - JS
        
        SmartDashboard.putBoolean('climber_state', self.climber_enable)

    def set_climber(self, left_volts, right_volts):
        self.left_winch.setVoltage(left_volts)
        self.follower_winch.setVoltage(right_volts)
        self.climber_enable = True
        print(f'setting volts to {self.climber_voltage}')
        SmartDashboard.putBoolean('climber_state', self.climber_enable)

    def stop_climber(self):
        # self.left_winch_controller.setReference(0, rev.CANSparkMax.ControlType.kVoltage)
        self.left_winch.setVoltage(0)
        self.follower_winch.setVoltage(0)
        self.climber_voltage = 0
        self.climber_enable = False
        SmartDashboard.putBoolean('climber_state', self.climber_enable)

    def get_climber(self):
        return "nope lol"
        # return self.left_winch_encoder.getVelocity()

    def toggle_climber(self, rpm):
        if self.climber_enable:
            self.stop_climber()
        else:
            self.set_climber(rpm)

    def toggle_climber_servos(self):
        if self.servos_open:
            self.close_servos()
        else:
            self.open_servos()
        self.servos_open = not self.servos_open
        return self.servos_open

    def open_servos(self):
        # Not sure how the climber works, all I know is that 103 deg is the "open" position for the servos
        # and 10 is the "closed" position -LHACK
        self.left_servo.setAngle(103)
        self.right_servo.setAngle(10)

    def close_servos(self):
        self.left_servo.setAngle(10)
        self.right_servo.setAngle(103)

    def periodic(self) -> None:

        self.counter += 1

        if self.counter % 20 == 0:
            # SmartDashboard.putNumber('climber_rpm', self.left_winch_encoder.getVelocity())
            # SmartDashboard.putBoolean('climber_ready', self.left_winch_encoder.getVelocity() > 1800)
            SmartDashboard.putNumber('climber_current', self.left_winch.getOutputCurrent())
            SmartDashboard.putNumber('climber_output', self.left_winch.getAppliedOutput())
