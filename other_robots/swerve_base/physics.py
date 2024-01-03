"""
    CJH Copied from robotpy docs example drivetrain physics, keeping only swerve  2023/03/19

    .. warning:: These drivetrain models are not particularly realistic, and
                 if you are using a tank drive style drivetrain you should use
                 the :class:`.TankModel` instead.

    Based on input from various drive motors, these helper functions
    simulate moving the robot in various ways. Many thanks to
    `Ether <http://www.chiefdelphi.com/forums/member.php?u=34863>`_
    for assistance with the motion equations.

    When specifying the robot speed to the below functions, the following
    may help you determine the approximate speed of your robot:

    * Slow: 4ft/s
    * Typical: 5 to 7ft/s
    * Fast: 8 to 12ft/s

    Obviously, to get the best simulation results, you should try to
    estimate the speed of your robot accurately.

    Here's an example usage of the drivetrains::

        import hal.simulation
        from pyfrc.physics import drivetrains

        class PhysicsEngine:

            def __init__(self, physics_controller):
                self.physics_controller = physics_controller
                self.drivetrain = drivetrains.TwoMotorDrivetrain(deadzone=drivetrains.linear_deadzone(0.2))

                self.l_motor = hal.simulation.PWMSim(1)
                self.r_motor = hal.simulation.PWMSim(2)

            def update_sim(self, now, tm_diff):
                l_motor = self.l_motor.getSpeed()
                r_motor = self.r_motor.getSpeed()

                speeds = self.drivetrain.calculate(l_motor, r_motor)
                self.physics_controller.drive(speeds, tm_diff)

                # optional: compute encoder
                # l_encoder = self.drivetrain.wheelSpeeds.left * tm_diff

    .. versionchanged:: 2020.1.0

       The input speeds and output rotation angles were changed to reflect
       the current WPILib drivetrain/field objects. Wheelbases and default
       speeds all require units.
"""
import math
import typing

import wpilib
from wpimath import units
import wpilib.simulation as simlib
from pyfrc.physics.core import PhysicsEngine, Pose2d
from pyfrc.physics.drivetrains import DeadzoneCallable, linear_deadzone
from wpimath.kinematics import ChassisSpeeds
import wpimath.geometry as geo
import ntcore as nt

import constants


class PhysicsEngine(PhysicsEngine):

    def __init__(self, physics_controller):
        self.physics_controller = physics_controller
        self.field = wpilib.Field2d()

        offset = 16
        self.lf_motor = simlib.PWMSim(21 - offset)
        self.lr_motor = simlib.PWMSim(23-offset)
        self.rf_motor = simlib.PWMSim(25-offset)
        self.rr_motor = simlib.PWMSim(27-offset)

        self.lf_angle = simlib.PWMSim(20-offset)
        self.lr_angle = simlib.PWMSim(22-offset)
        self.rf_angle = simlib.PWMSim(24-offset)
        self.rr_angle = simlib.PWMSim(26-offset)



        # Motor simulation definitions. Each correlates to a motor defined in the drivetrain subsystem.
        # self.lf_motor_spark = simlib.SimDeviceSim('SPARK MAX [21]')  # SparkMAX sim device
        # self.lr_motor_spark = simlib.SimDeviceSim('SPARK MAX [23]')
        # self.rf_motor_spark = simlib.SimDeviceSim('SPARK MAX [25]')  # SparkMAX sim device
        # self.rr_motor_spark = simlib.SimDeviceSim('SPARK MAX [27]')
        # self.lf_angle_spark = simlib.SimDeviceSim('SPARK MAX [20]')
        # self.lr_angle_spark = simlib.SimDeviceSim('SPARK MAX [22]')
        # self.rf_angle_spark = simlib.SimDeviceSim('SPARK MAX [24]')
        # self.rr_angle_spark = simlib.SimDeviceSim('SPARK MAX [26]')
        # self.l_spark_position = self.l_spark.getDouble('Position')  # SparkMAX encoder distance
        # self.r_spark_position = self.r_spark.getDouble('Position')
        # self.l_spark_velocity = self.l_spark.getDouble('Velocity')  # SparkMAX encoder rate
        # self.r_spark_velocity = self.r_spark.getDouble('Velocity')
        # self.l_spark_output = self.l_spark.getDouble('Applied Output')  # SparkMAX controller output
        # self.r_spark_output = self.r_spark.getDouble('Applied Output')

        # better way to set up all the swerve sparks
        self.spark_dict = {}
        # self.spark_ids = [21, 23, 25, 27, 20, 22, 24, 26]
        self.spark_ids = [23, 27, 21, 25, 22, 26, 20, 24]
        self.spark_drives = ['lr_motor_spark', 'rr_motor_spark', 'lf_motor_spark', 'rf_motor_spark']
        self.spark_turns = ['lr_angle_spark', 'rr_angle_spark', 'lf_angle_spark', 'rf_angle_spark']
        self.spark_names = self.spark_drives + self.spark_turns

        simlib.SimDeviceSim.enumerateDevices()

        for idx, (spark_name, can_id) in enumerate(zip(self.spark_names, self.spark_ids)):
            spark = simlib.SimDeviceSim(f'SPARK MAX [{can_id}]')
            position = spark.getDouble('Position')
            velocity = spark.getDouble('Velocity')
            output = spark.getDouble('Applied Output')
            self.spark_dict.update({spark_name: {'controller': spark, 'position': position,
                                                'velocity': velocity, 'output': output}})


        # NavX (SPI interface) - no idea why the "4" is there, seems to be the default name generated by the navx code
        self.navx = simlib.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")

        simlib.SimDeviceSim.enumerateDevices()

        self.x, self.y = constants.k_start_x, constants.k_start_y
        initial_pose = geo.Pose2d(0, 0, geo.Rotation2d())
        self.physics_controller.move_robot(geo.Transform2d(self.x, self.y, 0))

        # self.drivetrain = drivetrains.four_motor_swerve_drivetrain(deadzone=drivetrains.linear_deadzone(0.1))
        # make a pretend Armcam that reports an apriltag and a green reflective target
        self.camera_dict = {'tags': {}, 'green': {}}
        self.armcam_table = nt.NetworkTableInstance.getDefault().getTable('ArmCam')
        self.turretcam_table = nt.NetworkTableInstance.getDefault().getTable('BottomCam')
        self.tag_keys = ['targets', 'green']
        # for key in self.camera_dict.keys():
        #     self.camera_dict[key].update({'targets_entry': self.armcam_table.getEntry(f"/{key}/targets")})
        #     self.camera_dict[key].update({'distance_entry': self.armcam_table.getEntry(f"/{key}/distance")})
        #     self.camera_dict[key].update({'rotation_entry': self.armcam_table.getEntry(f"/{key}/rotation")})
        #     self.camera_dict[key].update({'strafe_entry': self.armcam_table.getEntry(f"/{key}/strafe")})
        for key in ['green']:
            self.camera_dict[key].update({'targets_entry': self.armcam_table.getDoubleTopic(f"{key}/targets").publish()})
            self.camera_dict[key].update({'distance_entry': self.armcam_table.getDoubleTopic(f"{key}/distance").publish()})
            self.camera_dict[key].update({'rotation_entry': self.armcam_table.getDoubleTopic(f"{key}/rotation").publish()})
            self.camera_dict[key].update({'strafe_entry': self.armcam_table.getDoubleTopic(f"{key}/strafe").publish()})
        for key in ['tags']:
            self.camera_dict[key].update({'targets_entry': self.turretcam_table.getDoubleTopic(f"{key}/targets").publish()})
            self.camera_dict[key].update({'distance_entry': self.turretcam_table.getDoubleTopic(f"{key}/distance").publish()})
            self.camera_dict[key].update({'rotation_entry': self.turretcam_table.getDoubleTopic(f"{key}/rotation").publish()})
            self.camera_dict[key].update({'strafe_entry': self.turretcam_table.getDoubleTopic(f"{key}/strafe").publish()})

    def update_sim(self, now, tm_diff):
        velocity_scale, angular_scale = 10, 10  # faking PWM signals in from -1 to 1, multiply by to x get velocities,
        lr_motor = self.lr_motor.getSpeed() * velocity_scale
        rr_motor = self.rr_motor.getSpeed() * velocity_scale
        lf_motor = self.lf_motor.getSpeed() * velocity_scale
        rf_motor = self.rf_motor.getSpeed() * velocity_scale

        lr_angle = self.lr_angle.getSpeed() * angular_scale   # multiply by 10 to get dummy angles from simulated pwms
        rr_angle = self.rr_angle.getSpeed() * angular_scale
        lf_angle = self.lf_angle.getSpeed() * angular_scale
        rf_angle = self.rf_angle.getSpeed() * angular_scale

        motors_and_angles = [lr_motor, rr_motor, lf_motor, rf_motor, lr_angle, rr_angle, lf_angle, rf_angle]

        # update the real sparks so the odometry is correct
        override_sparkmax = False  # can't figure this out yet
        if override_sparkmax:
            speed = 5
            for idx, spark in enumerate(self.spark_drives):
                # self.spark_dict[spark]['output'].set(motors_and_angles[idx])
                self.spark_dict[spark]['velocity'].set(motors_and_angles[idx])
                self.spark_dict[spark]['position'].set(self.spark_dict[spark]['position'].get() + motors_and_angles[idx] * tm_diff)
            for idx, spark in enumerate(self.spark_turns):
                self.spark_dict[spark]['position'].set(-motors_and_angles[idx + 4])
                # self.spark_dict[spark]['output'].set(motors_and_angles[idx] + 4)


        # had to mess with the orders and the vx, vy to get things to sim properly
        speeds = four_motor_swerve_drivetrain(*motors_and_angles, x_wheelbase = 2, y_wheelbase = 2,
        speed=5, deadzone=None,)

        self.physics_controller.drive(speeds, tm_diff)
        self.field.setRobotPose(self.physics_controller.get_pose())
        self.x = self.field.getRobotPose().X()
        self.y = self.field.getRobotPose().Y()
        self.theta = self.field.getRobotPose().rotation().degrees()
        wpilib.SmartDashboard.putNumberArray('sim_pose', [self.x, self.y, self.theta])
        wpilib.SmartDashboard.putNumberArray('drive_pose', [self.x, self.y, self.theta])  # need this for dashboard to update
        # optional: compute encoder
        # l_encoder = self.drivetrain.wheelSpeeds.left * tm_diff

        # Update the navx gyro simulation
        # -> FRC gyros like NavX are positive clockwise, but the returned pose is positive counter-clockwise
        self.navx_yaw.set((self.theta % 360))  # although we should be able to get this from a drivesim

        # update the vision simulation with tags and greens
        self.update_vision()

    def update_vision(self):  # TODO: update these to poses and pose math, and add camera offset from robot center
        locations = {'tags': {'id': 7, 'x': units.inchesToMeters(40.45), 'y': units.inchesToMeters(108.19)},
                     'green': {'id': '7h+', 'x': 0.34, 'y': 3.294}}
        # tag_x, tag_y = units.inchesToMeters(40.45), units.inchesToMeters(108.19)  # 1.02, 2.74 tag 7 is the one in front of blue center
        # green_x, green_y = units.inchesToMeters(0.34), 3.294  # high on right of tag 7

        for key in locations.keys():
            dx = self.x - locations[key]['x']
            dy = self.y - locations[key]['y']
            distance = (dx**2 + dy**2)**0.5
            rotation = (self.theta % 360)   # math.atan2(dy, dx) * 180/math.pi  # figure out what to do about this number,
            strafe = self.y - locations[key]['y']

            self.camera_dict[key]['targets_entry'].set(1)
            self.camera_dict[key]['distance_entry'].set(distance)
            self.camera_dict[key]['rotation_entry'].set(rotation)
            self.camera_dict[key]['strafe_entry'].set(strafe)

def four_motor_swerve_drivetrain(
    lr_motor: float,
    rr_motor: float,
    lf_motor: float,
    rf_motor: float,
    lr_angle: float,
    rr_angle: float,
    lf_angle: float,
    rf_angle: float,
    x_wheelbase=2,
    y_wheelbase=2,
    speed=10,
    deadzone=None,
) -> ChassisSpeeds:
    """
    Four motors that can be rotated in any direction

    If any motors are inverted, then you will need to multiply that motor's
    value by -1.

    :param lr_motor:   Left rear motor value (-1 to 1); 1 is forward
    :param rr_motor:   Right rear motor value (-1 to 1); 1 is forward
    :param lf_motor:   Left front motor value (-1 to 1); 1 is forward
    :param rf_motor:   Right front motor value (-1 to 1); 1 is forward

    :param lr_angle:   Left rear motor angle in degrees (0 to 360 measured clockwise from forward position)
    :param rr_angle:   Right rear motor angle in degrees (0 to 360 measured clockwise from forward position)
    :param lf_angle:   Left front motor angle in degrees (0 to 360 measured clockwise from forward position)
    :param rf_angle:   Right front motor angle in degrees (0 to 360 measured clockwise from forward position)

    :param x_wheelbase: The distance in feet between right and left wheels.
    :param y_wheelbase: The distance in feet between forward and rear wheels.
    :param speed:       Speed of robot in feet per second (see above)
    :param deadzone:    A function that adjusts the output of the motor (see :func:`linear_deadzone`)

    :returns: ChassisSpeeds that can be passed to 'drive'

    .. versionchanged:: 2020.1.0

       The output rotation angle was changed from CW to CCW to reflect the
       current WPILib drivetrain/field objects
    """

    if deadzone:
        lf_motor = deadzone(lf_motor)
        lr_motor = deadzone(lr_motor)
        rf_motor = deadzone(rf_motor)
        rr_motor = deadzone(rr_motor)

    # Calculate speed of each wheel
    lr = lr_motor * speed
    rr = rr_motor * speed
    lf = lf_motor * speed
    rf = rf_motor * speed

    # Calculate angle in radians
    lr_rad = lr_angle
    rr_rad = rr_angle
    lf_rad = lf_angle
    rf_rad = rf_angle

    # Calculate wheelbase radius
    wheelbase_radius = math.hypot(x_wheelbase / 2.0, y_wheelbase / 2.0)

    # Calculates the Vx and Vy components
    # Sin an Cos inverted because forward is 0 on swerve wheels
    Vy = (
        (math.sin(lr_rad) * lr)
        + (math.sin(rr_rad) * rr)
        + (math.sin(lf_rad) * lf)
        + (math.sin(rf_rad) * rf)
    )
    Vx = (
        (math.cos(lr_rad) * lr)
        + (math.cos(rr_rad) * rr)
        + (math.cos(lf_rad) * lf)
        + (math.cos(rf_rad) * rf)
    )

    # Adjusts the angle corresponding to a diameter that is perpendicular to the radius (add or subtract 45deg)
    lr_rad = (lr_rad + (math.pi / 4)) % (2 * math.pi)
    rr_rad = (rr_rad - (math.pi / 4)) % (2 * math.pi)
    lf_rad = (lf_rad - (math.pi / 4)) % (2 * math.pi)
    rf_rad = (rf_rad + (math.pi / 4)) % (2 * math.pi)

    # Finds the rotational velocity by finding the torque and adding them up
    Vw = wheelbase_radius * (
        (math.cos(lr_rad) * -lr)
        + (math.cos(rr_rad) * rr)
        + (math.cos(lf_rad) * -lf)
        + (math.cos(rf_rad) * rf)
    )

    Vx *= 0.25
    Vy *= -0.25  # I cheated on this to swap left and right.  Where is the right place to do this?
    Vw *= 0.25

    wpilib.SmartDashboard.putNumberArray('sim_chassis', [Vx, Vy, Vw])
    return ChassisSpeeds.fromFeet(Vx, Vy, Vw)

