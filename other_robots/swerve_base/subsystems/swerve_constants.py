import math
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
from rev import CANSparkMax

class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    kMaxSpeedMetersPerSecond = 1.5# 4.25 is Haochen competition, 4.8 is full out
    kMaxAngularSpeed = 0.25 * math.tau  # radians per second
    kMaxTotalSpeed = math.sqrt(2) *  kMaxAngularSpeed  # sum of angular and rotational, should probably do hypotenuse
    kMagnitudeSlewRate = 5  # hundred percent per second (1 = 100%)
    kRotationalSlewRate = 5  # hundred percent per second (1 = 100%)
    k_inner_deadband = 0.08  # use deadbands for joystick transformations and keepangle calculations
    k_outer_deadband = 0.95
    k_minimum_rotation = kMaxAngularSpeed * k_inner_deadband

    # Chassis configuration - not sure it even matters if we're square
    kTrackWidth = units.inchesToMeters(24.0)  # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(24.0)   # Distance between front and back wheels on robot

    # This is key !  Here is where you get left and right correct
    # It's the minus signs in the 2nd column that swap l/r  - but it can still mess up
    # kinematics gets passed [self.frontLeft, self.frontRight, self.rearLeft, self.rearRight]
    # Front left is X+Y+, Front right is + -, Rear left is - +, Rear right is - -
    # this should be left as the convention, so match the above.  Then take care of turning issues with the
    # INVERSION OF THE TURN OR DRIVE MOTORS

    kModulePositions = [
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),  # i swapped F and B to get the diamond on rotation
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    kGyroReversed = True

    # which motors need to be inverted  - none?
    # code seems to ignore this, so I turned the right wheels around instead, to have billet gears always point right.
    # that is a mistake, and need to rectify this
    k_lf_drive_motor_inverted = False
    k_lb_drive_motor_inverted = False
    k_rf_drive_motor_inverted = False
    k_rb_drive_motor_inverted = False

    k_lf_turn_motor_inverted = False
    k_lb_turn_motor_inverted = False
    k_rf_turn_motor_inverted = False
    k_rb_turn_motor_inverted = False

    # absolute encoder values when wheels facing forward  - 20230322 CJH
    # NOW IN RADIANS to feed right to the AnalogPotentiometer on the module
    k_lf_zero_offset = 1.011 * math.tau * 0.836  # 0.105 rad
    k_rf_zero_offset = 1.011 * math.tau * 0.745  # 4.682 rad   billet gear out on rf
    k_lb_zero_offset = 1.011 * math.tau * 0.723  # 4.531 rad
    k_rb_zero_offset = 1.011 * math.tau * 0.872  # 5.478 rad  billet gear out on rf


    # max absolute encoder value on each wheel  - 20230322 CJH
    # going to stop using this - it's probably 3.3 and it doesn't actually matter much considering our noise level
    # need to figure out where to account for this
    k_analog_encoder_scale_factor = 1.011  #  this thing only hits 0.989 for some reason
    k_lf_filtered_abs_max = 0.989
    k_rf_filtered_abs_max = 0.989
    k_lb_filtered_abs_max = 0.989
    k_rb_filtered_abs_max = 0.989

    # Angular offsets of the modules relative to the chassis in radians  -
    # (CJH: they want to put the abs encoder offsets here and set and forget)
    kFrontLeftChassisAngularOffset = math.tau * k_lf_zero_offset / k_lf_filtered_abs_max  # 5.389
    kFrontRightChassisAngularOffset = math.tau * k_rf_zero_offset / k_rf_filtered_abs_max  # 4.732
    kBackLeftChassisAngularOffset = math.tau * k_lb_zero_offset / k_lb_filtered_abs_max  # 4.611
    kBackRightChassisAngularOffset = math.tau * k_rb_zero_offset / k_rb_filtered_abs_max  # 5.581

    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 21
    kRearLeftDrivingCanId = 23
    kFrontRightDrivingCanId = 25
    kRearRightDrivingCanId = 27

    kFrontLeftTurningCanId = 20
    kRearLeftTurningCanId = 22
    kFrontRightTurningCanId = 24
    kRearRightTurningCanId = 26

    # not used but looks like we may have to use the Rio
    kFrontLeftAbsEncoderPort = 0
    kFrontRightAbsEncoderPort = 1
    kBackLeftAbsEncoderPort = 2
    kBackRightAbsEncoderPort = 3

class NeoMotorConstants:
    kFreeSpeedRpm = 5676

class ModuleConstants:

    # Invert the turning encoder, since the output shaft rotates in the opposite direction of
    # the steering motor in the MAXSwerve Module.
    kTurningEncoderInverted = False # True for absolute encoder, but not built-in

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 4 * 0.0254  #  0.1016  =  four inches
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = 6.75  # From MK4i website, L2  #  From (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0  # meters per second

    k_turning_motor_gear_ratio = 150/7  #  not needed when we switch to absolute encoder of 150/7
    kTurningEncoderPositionFactor = math.tau / k_turning_motor_gear_ratio # radian
    kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0  # radians per second

    kTurningEncoderPositionPIDMinInput = 0  # radian
    kTurningEncoderPositionPIDMaxInput = math.tau  # kTurningEncoderPositionFactor  # radian

    kDrivingP = 0
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps  # CJH tested 3/19/2023, works ok  - 0.2235
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 0.3 #  CJH tested this 3/19/2023  and 0.25 was good
    kTurningI = 0.0
    kTurningD = 0.0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    k_PID_dict_vel = {'kP': 0.0, 'kI': 0.000, 'kD': 0.00, 'kIz': 0.001, 'kFF': kDrivingFF, 'kArbFF':0, 'kMaxOutput': 0.95,
                'kMinOutput': -0.95, 'SM_MaxVel':3, 'SM_MaxAccel':2}  # 180 is 3 m/s and 3m/s/s

    kDrivingMotorIdleMode = CANSparkMax.IdleMode.kBrake
    kTurningMotorIdleMode = CANSparkMax.IdleMode.kCoast  # for now it's easier to move by hand when testing

    kDrivingMotorCurrentLimit = 50  # amp
    kTurningMotorCurrentLimit = 20  # amp

class AutoConstants:
    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 1

    # Constraint for the motion profiled robot angle controller
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    )

