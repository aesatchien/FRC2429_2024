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
        self.right_winch = rev.CANSparkMax(constants.k_follower_winch_neo_port, motor_type)  # actually doesn't follow
        # self.follower_winch.follow

        for spark in [self.left_winch, self.right_winch]:
            spark.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
            if constants.k_burn_flash:
                spark.burnFlash()

        self.left_winch.setInverted(True)
        self.right_winch.setInverted(False)

        # encoders
        self.left_winch_encoder = self.left_winch.getEncoder()
        self.right_winch_encoder = self.right_winch.getEncoder()
        self.encoders = [self.right_winch_encoder, self.left_winch_encoder]
        [encoder.setPosition(0) for encoder in self.encoders]

        # #controller
        # self.left_winch_controller = self.left_winch.getPIDController()
        # self.left_winch_controller.setP(0.0001)
        # self.kFF = 1.03 * 1/6784 # feed forward for a spark flex from shooter
        # self.left_winch_controller.setFF(self.kFF, 0)
        # self.left_winch_controller.setFF(self.kFF, 0)

        self.left_servo = wpilib.Servo(constants.k_left_servo_port)
        self.right_servo = wpilib.Servo(constants.k_right_servo_port)
        self.trap_servo = wpilib.Servo(constants.k_trap_servo_port)

        # toggle state
        self.climber_enable = False
        self.servos_open = False
        self.trap_open = False

        self.close_servos()  # Drew wanted servos to be "closed" on startup - JS
        self.close_trap_servo()

        SmartDashboard.putBoolean('climber_state', self.climber_enable)

    def set_climber(self, left_volts, right_volts, verbose=True):
        self.left_winch.setVoltage(left_volts)
        self.right_winch.setVoltage(right_volts)
        self.climber_enable = True
        if verbose:
            print(f'setting volts to {right_volts} {left_volts}')
        SmartDashboard.putBoolean('climber_state', self.climber_enable)

    def stop_climber(self):
        # self.left_winch_controller.setReference(0, rev.CANSparkMax.ControlType.kVoltage)
        self.left_winch.setVoltage(0)
        self.right_winch.setVoltage(0)
        self.climber_voltage = 0
        self.climber_enable = False
        SmartDashboard.putBoolean('climber_state', self.climber_enable)

    def get_climber(self):
        return "nope lol"
        # return self.left_winch_encoder.getVelocity()

    def get_encoders(self):
        return self.right_winch_encoder.getPosition(), self.left_winch_encoder.getPosition()

    def reset_encoders(self):
        [encoder.setPosition(0) for encoder in self.encoders]

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
        return self.servos_open

    def open_servos(self):
        # Not sure how the climber works, all I know is that 103 deg is the "open" position for the servos
        # and 10 is the "closed" position -LHACK
        self.left_servo.setAngle(103)
        self.right_servo.setAngle(10)
        self.servos_open = True
        print('OPENING CLIMBER SERVOS')

    def close_servos(self):
        self.left_servo.setAngle(10)
        self.right_servo.setAngle(103)
        self.servos_open = False
        print('CLOSING CLIMBER SERVOS')

    def open_trap_servo(self):
        self.trap_open = True
        # self.trap_servo.set(1)
        self.trap_servo.set(0)
        print(f"OPENED TRAP SERVO")

    def close_trap_servo(self):
        self.trap_open = False
        self.trap_servo.setAngle(180)
        print(f"CLOSED TRAP SERVO")

    def toggle_trap_servo(self):
        if self.trap_open:
            self.close_trap_servo()
        else:
            self.open_trap_servo()
        print(f"TOGGLED TRAP SERVO")

    def periodic(self) -> None:

        self.counter += 1

        if self.counter % 20 == 0:
            # SmartDashboard.putNumber('climber_rpm', self.left_winch_encoder.getVelocity())
            # SmartDashboard.putBoolean('climber_ready', self.left_winch_encoder.getVelocity() > 1800)
            SmartDashboard.putNumber('climber_current', self.left_winch.getOutputCurrent())
            SmartDashboard.putNumber('climber_output', self.left_winch.getAppliedOutput())
            SmartDashboard.putNumber('climber_r_encoder', self.right_winch_encoder.getPosition())
            SmartDashboard.putNumber('climber_l_encoder', self.left_winch_encoder.getPosition())
            SmartDashboard.putNumber("right servo angle", self.right_servo.getAngle())
            SmartDashboard.putNumber("left servo angle", self.left_servo.getAngle())

        if wpilib.RobotBase.isSimulation():  # fake the climber motors - 50 rev/second per 4 volts
            self.right_winch_encoder.setPosition(self.right_winch_encoder.getPosition() + self.right_winch.getAppliedOutput() * 50 / (4*50))
            self.left_winch_encoder.setPosition(self.left_winch_encoder.getPosition() + self.left_winch.getAppliedOutput() * 50 / (4*50))
