import math
import typing
import wpilib

from commands2 import SubsystemBase
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry,)
from wpimath.controller import PIDController
import navx
import rev

import constants
from .swervemodule_2429 import SwerveModule
from .swerve_constants import DriveConstants as dc


class Swerve (SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.counter = 0

        # Create SwerveModules
        self.frontLeft = SwerveModule(
            drivingCANId=dc.kFrontLeftDrivingCanId, turningCANId=dc.kFrontLeftTurningCanId,
            encoder_analog_port=dc.kFrontLeftAbsEncoderPort, turning_encoder_offset=dc.k_lf_zero_offset,
            driving_inverted=dc.k_lf_drive_motor_inverted, turning_inverted=dc.k_lf_turn_motor_inverted, label= 'lf' )
        self.frontRight = SwerveModule(
            dc.kFrontRightDrivingCanId, dc.kFrontRightTurningCanId, dc.kFrontRightAbsEncoderPort, dc.k_rf_zero_offset,
            dc.k_rf_drive_motor_inverted, dc.k_rf_turn_motor_inverted, label='rf')
        self.rearLeft = SwerveModule(
            dc.kRearLeftDrivingCanId, dc.kRearLeftTurningCanId, dc.kBackLeftAbsEncoderPort, dc.k_lb_zero_offset,
            dc.k_lb_drive_motor_inverted, dc.k_lb_turn_motor_inverted, label='lb')
        self.rearRight = SwerveModule(
            dc.kRearRightDrivingCanId, dc.kRearRightTurningCanId, dc.kBackRightAbsEncoderPort, dc.k_rb_zero_offset,
            dc.k_rb_drive_motor_inverted, dc.k_rb_turn_motor_inverted, label='rb')

        # let's make this pythonic so we can do things quickly and with readability
        self.swerve_modules = [self.frontLeft, self.frontRight, self.rearLeft, self.rearRight]

        # The gyro sensor
        #self.gyro = wpilib.ADIS16470_IMU()
        self.gyro = navx.AHRS.create_spi(update_rate_hz=50)
        self.navx = self.gyro
        if self.navx.isCalibrating():
            # schedule a command to reset the navx
            print('unable to reset navx: Calibration in progress')
        else:
            pass

        #  stupid gyro never resets on boot
        self.navx.zeroYaw()  # we boot up at zero degrees  - note - you can't reset this while calibrating
        self.gyro_calibrated = False

        # timer and variables for checking if we should be using pid on rotation
        self.keep_angle = 0  # the heading we try to maintain when not rotating
        self.keep_angle_timer = wpilib.Timer()
        self.keep_angle_timer.start()
        self.keep_angle_timer.reset()
        self.keep_angle_pid = PIDController(0.015, 0, 0)  # todo: put these in constants.  allow 1% stick per degree
        self.keep_angle_pid.enableContinuousInput(-180, 180)  # using the gyro's yaw is b/w -180 and 180
        self.last_rotation_time = 0
        self.time_since_rotation = 0
        self.last_drive_time = 0
        self.time_since_drive = 0

        # Slew rate filter variables for controlling lateral acceleration
        self.currentRotation, self.currentTranslationDir, self.currentTranslationMag  = 0.0, 0.0, 0.0

        self.fwd_magLimiter = SlewRateLimiter(0.9 * dc.kMagnitudeSlewRate)
        self.strafe_magLimiter = SlewRateLimiter(dc.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(dc.kRotationalSlewRate)
        self.prevTime = wpilib.Timer.getFPGATimestamp()

        # Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            dc.kDriveKinematics, Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions(),
        initialPose=Pose2d(constants.k_start_x, constants.k_start_y, Rotation2d.fromDegrees(self.get_angle())))

    def periodic(self) -> None:

        self.counter += 1
        # Update the odometry in the periodic block -
        # TODO - figure out if the odometry and the swerve use the same angle conventions - seems backwards
        # or I faked it incorrectly in the sim
        if wpilib.RobotBase.isReal():
            self.odometry.update(Rotation2d.fromDegrees(self.get_angle()), *self.get_module_positions(),)
        else:
            pass
            # get pose from simulation's post to NT
            # pose = wpilib.SmartDashboard.getNumberArray('drive_pose', [0,0,0])
            # self.odometry.resetPosition(Rotation2d.fromDegrees(self.get_angle()), Pose2d(pose[0], pose[1], pose[2]))

        if self.counter % 10 == 0:
            pose = self.get_pose()  # self.odometry.getPose()
            if wpilib.RobotBase.isReal():
                wpilib.SmartDashboard.putNumberArray('drive_pose', [pose.X(), pose.Y(), pose.rotation().degrees()])
            wpilib.SmartDashboard.putNumber('_navx', self.get_angle())
            wpilib.SmartDashboard.putNumber('_navx_yaw', self.navx.getYaw())

            if constants.k_debugging_messages:  # this is just a bit much, so
                angles = [m.turningEncoder.getPosition() for m in self.swerve_modules]
                absolutes = [m.get_turn_encoder() for m in self.swerve_modules]
                wpilib.SmartDashboard.putNumberArray(f'_angles', angles)
                wpilib.SmartDashboard.putNumberArray(f'_analog_radians', absolutes)
                ypr = [self.navx.getYaw(), self.get_pitch(), self.navx.getRoll(), self.navx.getRotation2d().degrees()]
                wpilib.SmartDashboard.putNumberArray('_navx_YPR', ypr)

    def get_pose(self, report=False) -> Pose2d:
        # return the pose of the robot  TODO: update the dashboard here?
        if report:
            pass
            # print(f'attempting to get pose: {self.odometry.getPose()}')
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.
        :param pose: The pose to which to set the odometry.
        """
        self.odometry.resetPosition(
            Rotation2d.fromDegrees(self.get_angle()), pose, *self.get_module_positions())

    def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool, rate_limited: bool, keep_angle:bool=True) -> None:
        """Method to drive the robot using joystick info.
        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        """

        if rate_limited:
            xSpeedCommanded = self.fwd_magLimiter.calculate(xSpeed)
            ySpeedCommanded = self.strafe_magLimiter.calculate(ySpeed)
            rotation_commanded = self.rotLimiter.calculate(rot)
        else:
            xSpeedCommanded = xSpeed
            ySpeedCommanded = ySpeed
            rotation_commanded = rot

        if keep_angle:
            rotation_commanded = self.perform_keep_angle(xSpeed, ySpeed, rot)  # call the 1706 keep angle routine to maintain rotation

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedDelivered = xSpeedCommanded * dc.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeedCommanded * dc.kMaxSpeedMetersPerSecond
        rotDelivered = rotation_commanded * dc.kMaxAngularSpeed

        # probably can stop doing this now
        if constants.k_debugging_messages:
            wpilib.SmartDashboard.putNumberArray('_xyr', [xSpeedDelivered, ySpeedDelivered, rotDelivered])

        # create the swerve state array depending on if we are field relative or not
        swerveModuleStates = dc.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(self.get_angle()),)
            if fieldRelative else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered))

        # normalize wheel speeds so we do not exceed our speed limit
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, dc.kMaxTotalSpeed)
        for state, module in zip(swerveModuleStates, self.swerve_modules):
            module.setDesiredState(state)

        # safety
        #self.drivebase.feed()

    def perform_keep_angle(self, xSpeed, ySpeed, rot):  # update rotation if we are drifting when trying to drive straight
        output = rot  # by default we will return rot unless it needs to be changed
        if math.fabs(rot) > dc.k_inner_deadband:  # we are actually intending to rotate
            self.last_rotation_time = self.keep_angle_timer.get()
        if math.fabs(xSpeed) > dc.k_inner_deadband or math.fabs(ySpeed) > dc.k_inner_deadband:
            self.last_drive_time = self.keep_angle_timer.get()

        self.time_since_rotation = self.keep_angle_timer.get() - self.last_rotation_time
        self.time_since_drive = self.keep_angle_timer.get() - self.last_drive_time

        if self.time_since_rotation < 0.5:  # (update keep_angle until 0.5s after rotate command stops to allow rotate to finish)
            self.keep_angle = self.get_yaw()  # todo: double check SIGN (and units are in degrees)
        elif math.fabs(rot) < dc.k_inner_deadband and self.time_since_drive < 0.25:  # stop keep_angle .25s after you stop driving
            output = self.keep_angle_pid.calculate(-self.get_angle(), self.keep_angle)
            output = output if math.fabs(output) < 0.2 else 0.2 * math.copysign(1, output)  # clamp at 0.2

        return output

    def set_drive_motor_references(self, setpoint, control_type = rev.CANSparkMax.ControlType.kVoltage,
                                    pidSlot=1, arbFeedForward=0):
        # Make sure you've turned the swerve into a tank drive before calling this
        for module in self.swerve_modules:
            module.drivingPIDController.setReference(setpoint, control_type, pidSlot=pidSlot)

    def set_brake_mode(self, mode='brake'):
        if mode == 'brake':
            self.setX()

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        angles = [45, -45, -45, 45]
        # angles = [0, 0, 0, 0]

        for angle, swerve_module in zip(angles, self.swerve_modules):
            swerve_module.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(angle)))

    def setModuleStates(self, desiredStates: typing.Tuple[SwerveModuleState]) -> None:
        """Sets the swerve ModuleStates.
        :param desiredStates: The desired SwerveModule states.
        """
        desiredStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, dc.kMaxTotalSpeed)
        for idx, m in enumerate(self.swerve_modules):
            m.setDesiredState(desiredStates[idx])

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        [m.resetEncoders() for m in self.swerve_modules]

    def zeroHeading(self) -> None:
        """Zeroes the heading of the robot."""
        self.gyro.reset()

    def getHeading(self) -> float:
        """Returns the heading of the robot.
        :returns: the robot's heading in degrees, from -180 to 180
        """
        return Rotation2d.fromDegrees(self.get_angle()).getDegrees()

    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot.
        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (-1.0 if dc.kGyroReversed else 1.0)

    def get_module_positions(self):
        """ CJH-added helper function to clean up some calls above"""
        # note lots of the calls want tuples, so _could_ convert if we really want to
        return [m.getPosition() for m in self.swerve_modules]

    def get_angle(self):  # if necessary reverse the heading for swerve math
        return -self.gyro.getAngle() if dc.kGyroReversed else self.gyro.getAngle()

    def get_raw_angle(self):  # never reversed value for using PIDs on the heading
        return self.gyro.getAngle()

    def get_yaw(self):  # helpful for determining nearest heading parallel to the wall
        return self.gyro.getYaw()

    def get_pitch(self):  # need to calibrate the navx, apparently
        return self.gyro.getPitch() - 4.75

    def reset_gyro(self):  # use this from now on whenever we reset the gyro
        self.gyro.reset()
        self.keep_angle = 0

