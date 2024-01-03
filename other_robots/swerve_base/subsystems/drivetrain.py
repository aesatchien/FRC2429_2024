import time
import wpilib
from commands2 import SubsystemBase
from wpilib.drive import DifferentialDrive
from wpilib import MotorControllerGroup, PWMSparkMax, SmartDashboard
import rev
import navx
import wpimath.geometry as geo
from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveWheelSpeeds

from misc.configure_controllers import configure_sparkmax
#from misc.sparksim import CANSparkMax  # takes care of switching to PWM for sim

import constants

class Drivetrain(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.counter = 0

        # initialize sensors - use the navx for headings
        self.navx = navx.AHRS.create_spi()

        # initialize motors
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.spark_neo_left_front = rev.CANSparkMax(constants.k_left_motor1_port, motor_type)
        self.spark_neo_left_back = rev.CANSparkMax(constants.k_left_motor2_port, motor_type)
        self.spark_neo_right_front = rev.CANSparkMax(constants.k_right_motor1_port, motor_type)
        self.spark_neo_right_back = rev.CANSparkMax(constants.k_right_motor2_port, motor_type)

        self.spark_neo_right_front.setInverted(False)  # now driving with battery in front
        self.spark_neo_left_front.setInverted(True)
        # save some typing and CAN bus activity this year by setting followers
        self.spark_neo_right_back.follow(self.spark_neo_right_front, invert=False)
        self.spark_neo_left_back.follow(self.spark_neo_left_front, invert=False)
        self.controllers = [self.spark_neo_left_front, self.spark_neo_right_front]

        # get encoders and PID controllers
        self.spark_neo_left_encoder = rev.CANSparkMax.getEncoder(self.spark_neo_left_front)
        self.spark_neo_right_encoder = rev.CANSparkMax.getEncoder(self.spark_neo_right_front)
        self.encoders = [self.spark_neo_left_encoder, self.spark_neo_right_encoder]
        self.spark_PID_controller_right_front = self.spark_neo_right_front.getPIDController()
        self.spark_PID_controller_left_front = self.spark_neo_left_front.getPIDController()
        self.pid_controllers = [self.spark_PID_controller_left_front, self.spark_PID_controller_right_front]
        self.left_encoder, self.right_encoder = self.spark_neo_left_encoder, self.spark_neo_right_encoder  # alias
        # set up the wpilib drive object
        self.left_motors = MotorControllerGroup(self.spark_neo_left_front)
        self.right_motors = MotorControllerGroup(self.spark_neo_right_front)
        self.left_motors.setInverted(False)
        self.right_motors.setInverted(False)
        self.drive = DifferentialDrive(self.left_motors, self.right_motors)
        self.drive.setMaxOutput(1.0)  # default, not necessary but here for training students
        self.drive.setSafetyEnabled(True)  # default, not necessary but here for training students
        self.drive.setExpiration(expirationTime=0.1)

        # configure the encoders and controllers
        self.configure_controllers()
        # Reset the encoders upon the initialization of the robot
        self.reset_encoders()
        self.odometry = DifferentialDriveOdometry(gyroAngle=geo.Rotation2d.fromDegrees(-self.navx.getAngle()),
                                leftDistance=0, rightDistance=0,
                                initialPose=geo.Pose2d(constants.k_start_x, constants.k_start_y,
                                geo.Rotation2d.fromDegrees(constants.k_start_heading)))


        # add two dummy PWMs so we can track the SparkMax in the sim (should be updated in sim periodic)
        if wpilib.RobotBase.isSimulation():
            self.dummy_motor_left = PWMSparkMax(1)
            self.dummy_motor_right = PWMSparkMax(3)

    # ----------------- DRIVE METHODS -----------------------

    def smart_motion(self, distance, feed_forward=0, slot=1):
        # use smartmotion to send you there quickly  TODO - check if the right needs to be negated
        control_type = rev.CANSparkMax.ControlType.kSmartMotion
        self.spark_PID_controller_left_front.setReference(value=distance, ctrl=control_type, arbFeedforward=feed_forward, pidSlot=slot)
        self.spark_PID_controller_right_front.setReference(value=distance, ctrl=control_type, arbFeedforward=feed_forward, pidSlot=slot)

    def arcade_drive(self, fwd, rot):
        self.drive.arcadeDrive(fwd, rot, squareInputs=True)
        if wpilib.RobotBase.isSimulation():
            # need to update the simulated PWMs here
            self.dummy_motor_left.set(self.spark_neo_left_front.get())
            self.dummy_motor_right.set(self.spark_neo_right_front.get())

    def tank_drive_volts(self, left_volts, right_volts):
        """Control the robot's drivetrain with voltage inputs for each side.  Used by Ramsete """
        # Set the voltage for each side - sometimes one needs to be inverted to drive correctly.
        self.left_motors.setVoltage(left_volts)
        self.right_motors.setVoltage(right_volts)

        # need to update the simulated PWMs here
        if wpilib.RobotBase.isSimulation():
            self.dummy_motor_left.set(left_volts/12)
            self.dummy_motor_right.set(-right_volts/12)
            SmartDashboard.putNumber('/drive/left_volts', left_volts)
            SmartDashboard.putNumber('/drive/right_volts', right_volts)
        # Resets the timer for this subsystem's MotorSafety
        self.drive.feed()

    def drive_forwards_vel(self, targetvel, pidSlot=0, l_feed_forward=0, r_feed_forward=0):
        self.spark_PID_controller_left_front.setReference(targetvel, rev.CANSparkMax.ControlType.kSmartVelocity,
                                                          pidSlot=pidSlot, feed_forward=l_feed_forward)
        self.spark_PID_controller_right_front.setReference(targetvel, rev.CANSparkMax.ControlType.kSmartVelocity,
                                                          pidSlot=pidSlot, feed_forward=r_feed_forward)

    # ----------------- DRIVETRAIN STATE AND MODIFICATION METHODS -----------------------
    def configure_controllers(self):
        for encoder in self.encoders:
            encoder.setPositionConversionFactor(constants.k_sparkmax_conversion_factor_meters)  # want in m
            encoder.setPositionConversionFactor(constants.k_sparkmax_conversion_factor_meters)  # want in m/s but screws up smartmotion, so m/min

        for slot in [0,1,2]:  # limit the accumulator for the incline drive
            [pid_controller.setIMaxAccum(constants.k_drive_accumulator_max, slotID=slot) for pid_controller in self.pid_controllers]

        # send controller info, burn if necessary
        ids = [constants.k_left_motor1_port, constants.k_right_motor1_port]
        for controller, pid_controller, id in zip(self.controllers, self.pid_controllers, ids):
            configure_sparkmax(sparkmax=controller, pid_controller=pid_controller, can_id=id, slot=0,
                               burn_flash=constants.k_burn_flash, pid_dict=constants.k_PID_dict_pos, pid_only=False)
            configure_sparkmax(sparkmax=controller, pid_controller=pid_controller, can_id=id, slot=1,
                               burn_flash=constants.k_burn_flash, pid_dict=constants.k_PID_dict_vel, pid_only=False)
            configure_sparkmax(sparkmax=controller, pid_controller=pid_controller, can_id=id, slot=2,
                               burn_flash=constants.k_burn_flash, pid_dict=constants.k_PID_dict_vel_slow, pid_only=False)


    def get_positions(self):
        left_position = self.spark_neo_left_encoder.getPosition()
        right_position = self.spark_neo_right_encoder.getPosition()
        return (left_position, right_position)

    def set_brake_mode(self, mode='brake'):
        """ Sets the brake mode for the drivetrain to coast or brake"""
        brake_mode = rev.CANSparkMax.IdleMode.kBrake
        if mode == 'coast':
            brake_mode = rev.CANSparkMax.IdleMode.kCoast
        for controller in self.controllers:
            controller.setIdleMode(brake_mode)

    def reset_encoders(self):  # part of resetting odometry
        """Resets the encoders of the drivetrain."""
        for encoder in self.encoders:
            encoder.setPosition(0)
            time.sleep(0.025)
            self.feed()

    def reset(self):  # TODO: reset odometry here too?
        self.navx.reset()
        self.reset_encoders()

    def feed(self):
        self.drive.feed()

    # ------------------ RAMSETE METHODS  ------------------
    def get_rotation2d(self):  # used in ramsete
        return geo.Rotation2d.fromDegrees(-self.navx.getAngle())

    def get_pose(self):  # used in ramsete and in this subsystem's updates
        """Returns the current position of the robot using its odometry."""
        return self.odometry.getPose()

    def get_wheel_speeds(self):  # used in ramsete
        """Return an object which represents the wheel speeds of our drivetrain."""
        speeds = DifferentialDriveWheelSpeeds(self.left_encoder.getVelocity(), self.right_encoder.getVelocity())
        return speeds

    def reset_odometry(self, pose):  # used in ramsete
        """ Resets the robot's odometry to a given position."""
        print(f'resetting odometry to {pose}')
        self.reset_encoders()
        self.navx.setAngleAdjustment(-constants.k_start_heading - self.navx.getYaw())

    def get_rate(self, encoder): # spark maxes and regular encoders use different calls... annoying.  used in ramsete.
        return encoder.getVelocity()

    def get_average_encoder_rate(self):  # used in ramsete
        return (self.left_encoder.getVelocity() - self.right_encoder.getVelocity())/2

    # ----------------- PERIODIC UPDATES -----------------------
    def periodic(self):
        # Called periodically when it can be called. Updates the robot's odometry with sensor data.
        self.counter += 1
        self.odometry.update(geo.Rotation2d.fromDegrees(-self.navx.getAngle()),
                             self.left_encoder.getPosition(), -self.right_encoder.getPosition())

        if self.counter % 15 == 0:
            # keep track of where the robot is with an x and y position (only good for WCD)
            pose = self.get_pose()
            SmartDashboard.putNumberArray('drive_pose', [pose.X(), pose.Y(), pose.rotation().degrees()])
            SmartDashboard.putNumber('drive_lpos', self.left_encoder.getPosition())
            SmartDashboard.putNumber('drive_rpos', self.right_encoder.getPosition())
            SmartDashboard.putNumber('drive_lvel', self.left_encoder.getVelocity())
            SmartDashboard.putNumber('drive_rvel', self.right_encoder.getVelocity())
            # SmartDashboard.putNumber('navX', self.navx.getAngle())


