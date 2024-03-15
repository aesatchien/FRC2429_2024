#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# See the notes for the other physics sample
#

import math
import wpilib
import wpilib.simulation as simlib
from pyfrc.physics.core import PhysicsInterface
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState, SwerveModulePosition
import wpimath.geometry as geo
import hal
import rev
from pyfrc.physics.units import units

from subsystems.swerve_constants import DriveConstants as dc
import constants

from robot import MyRobot

import typing
if typing.TYPE_CHECKING:  # not sure how dropbears are getting their classes to work - this will be offseason research
    pass


class PhysicsEngine:
    """
    Simulates our swerve drive - still clunky, we should be able to get our turn motors not from the dash
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """
        self.physics_controller = physics_controller  # must have for simulation
        self.robot = robot

        self.kinematics: SwerveDrive4Kinematics = dc.kDriveKinematics  # our swerve drive kinematics

        # NavX (SPI interface) - no idea why the "4" is there, seems to be the default name generated by the navx code
        self.navx = simlib.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")  # for some reason it seems we have to set Yaw and not Angle
        self.navx_angle = self.navx.getDouble("Angle")

        # TODO: determine if we need these
        self.analogs = [simlib.AnalogInputSim(i) for i in range(4)]
        self.analog_offsets = []

        # create a dictionary so we can refer to the sparks by name and get their relevant parameters
        self.spark_dict = {}
        # kinematics chassis speeds wants them in same order as in original definition - unfortunate ordering
        self.spark_drives = ['lf_drive', 'rf_drive', 'lb_drive', 'rb_drive']
        self.spark_drive_ids = [21, 25, 23, 27]  # keep in this order - based on our kinematics definition
        self.spark_turns = ['lf_turn', 'rf_turn', 'lb_turn', 'rb_turn']
        self.spark_turn_ids = [20, 24, 22, 26]  # keep in this order

        # Other devices
        self.spark_peripherals = ['br_crank', 'bl_crank', 'tr_crank', 'tl_crank', 't_shooter','b_shooter']
        self.spark_peripheral_ids = [6, 7, 8, 9, 10, 11]

        # allow ourselves to access the simdevice's Position, Velocity, Applied Output, etc
        self.spark_names = self.spark_drives + self.spark_turns + self.spark_peripherals
        self.spark_ids = self.spark_drive_ids + self.spark_turn_ids + self.spark_peripheral_ids
        for idx, (spark_name, can_id) in enumerate(zip(self.spark_names, self.spark_ids)):
            spark = simlib.SimDeviceSim(f'SPARK MAX [{can_id}]')
            position = spark.getDouble('Position')
            velocity = spark.getDouble('Velocity')
            output = spark.getDouble('Applied Output')
            self.spark_dict.update({spark_name: {'controller': spark, 'position': position,
                                                 'velocity': velocity, 'output': output}})
        for key, value in self.spark_dict.items():  # see if these make sense
            print(f'{key}: {value}')

        self.distances = [0, 0, 0, 0]

        # sensors

        # set up the initial location of the robot on the field
        self.x, self.y = constants.k_start_x, constants.k_start_y
        initial_pose = geo.Pose2d(0, 0, geo.Rotation2d())
        self.physics_controller.move_robot(geo.Transform2d(self.x, self.y, 0))

        # Create a Mechanism2d display of an Arm
        self.mech2d = wpilib.Mechanism2d(60, 60)
        self.armBase = self.mech2d.getRoot("ArmBase", 30, 30)
        self.chassisBase = self.mech2d.getRoot("chassisBase", 5, 23)
        self.chassis = self.chassisBase.appendLigament(
            "chassis", 28, 0, 12, wpilib.Color8Bit(wpilib.Color.kGray))
        self.crank_arm_mech = self.armBase.appendLigament(
            "Crank Arm Tower", 22, 90, 12, wpilib.Color8Bit(wpilib.Color.kDodgerBlue)
        )
        self.shooter_arm_mech = self.crank_arm_mech.appendLigament(
            "Shooter Arm", 19, 175, 20, wpilib.Color8Bit(wpilib.Color.kYellow)
        )

        wpilib.SmartDashboard.putData("Arm Sim", self.mech2d)

        # self.arm_motor: rev.CANSparkMax = robot.container.crank_arm.motor

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Not sure why this is necessary but it is- LHACK 3/12/24
        simlib.DriverStationSim.setAllianceStationId(hal.AllianceStationID.kBlue2)

        # Simulate the drivetrain (only front motors used because read should be in sync)
        ##lf_motor = self.lf_motor.getSpeed()
        ##rf_motor = self.rf_motor.getSpeed()
        ##transform = self.drivetrain.calculate(lf_motor, rf_motor, tm_diff)
        ##pose = self.physics_controller.move_robot(transform)

        # Update the gyro simulation
        # -> FRC gyros are positive clockwise, but the returned pose is positive
        #    counter-clockwise
        ##self.gyro.setAngle(-pose.rotation().degrees())

        # need to update the positions of the swerve's turning motors - they have no feedback in sim
        # currently desired states from NT because I'm not sure how else to get them
        dash_values = ['lf_target_vel_angle', 'rf_target_vel_angle', 'lb_target_vel_angle', 'rb_target_vel_angle']
        target_angles = [wpilib.SmartDashboard.getNumberArray(dash_value, [0, 0])[1] for dash_value in dash_values]
        for spark_turn, target_angle in zip(self.spark_turns, target_angles):
            self.spark_dict[spark_turn]['position'].set(target_angle)  # this works to update the simulated spark
        wpilib.SmartDashboard.putNumberArray('target_angles', target_angles)

        # send the speeds and positions from the spark sim devices to the fourmotorswervedrivetrain
        module_states = []
        for drive, turn in zip(self.spark_drives, self.spark_turns):
            module_states.append(SwerveModuleState(
                self.spark_dict[drive]['velocity'].value, geo.Rotation2d(self.spark_dict[turn]['position'].value))
            )

        # using our own kinematics to update the chassis speeds
        speeds = self.kinematics.toChassisSpeeds((module_states))

        # update the sim's robot
        self.physics_controller.drive(speeds, tm_diff)

        # send our poses to the dashboard so we can use it with our trackers
        pose = self.physics_controller.get_pose()
        self.x, self.y, self.theta  = pose.X(), pose.Y(), pose.rotation().degrees()

        # attempt to update the real robot's odometry
        self.distances = [pos + tm_diff * self.spark_dict[drive]['velocity'].value for pos, drive in zip(self.distances, self.spark_drives)]
        [self.spark_dict[drive]['position'].set(self.spark_dict[drive]['position'].value + tm_diff * self.spark_dict[drive]['velocity'].value ) for drive in self.spark_drives]

        # TODO - why does this not take care of itself if I just update the simmed SPARK's position?
        swerve_positions = [SwerveModulePosition(distance=dist, angle=m.angle) for m, dist in zip(module_states, self.distances)]
        self.robot.container.drive.odometry.update(pose.rotation(), swerve_positions)

        wpilib.SmartDashboard.putNumberArray('sim_pose', [self.x, self.y, self.theta])
        wpilib.SmartDashboard.putNumberArray('drive_pose', [self.x, self.y, self.theta])  # need this for 2429 python dashboard to update

        crank_angle, crank_velocity = self.spark_dict['bl_crank']['position'].value, self.spark_dict['bl_crank']['velocity'].value
        shooter_arm_angle, shooter_arm_velocity = self.spark_dict['tr_crank']['position'].value, self.spark_dict['tr_crank']['velocity'].value

        # optional: compute encoder
        # l_encoder = self.drivetrain.wheelSpeeds.left * tm_diff

        # Update the navx gyro simulation
        # -> FRC gyros like NavX are positive clockwise, but the returned pose is positive counter-clockwise
        # TODO: do we need to put in whether our gyro is reversed (changes if mounted upside down) ?
        # below should account for resetting the yaw externally when we reset heading - the yaw is NOT the pose
        # really confused about yaw vs angle
        # yaw has the wrong sign, so just never use it except to update the actual navx in the sim
        self.navx_yaw.set(self.navx_yaw.get() - math.degrees(speeds.omega * tm_diff))

        # Update the arm
        voltage = wpilib.simulation.RoboRioSim.getVInVoltage()
        # self.armSim.setInputVoltage(self.arm_motor_sim.getSpeed() * voltage)
        # self.armSim.update(tm_diff)
        # arm_angle = self.armSim.getAngleDegrees()

        # self.armSim.setState(crank_angle, crank_velocity)
        self.crank_arm_mech.setAngle(crank_angle_to_sim(crank_angle))
        self.shooter_arm_mech.setAngle(shooter_angle_to_sim(shooter_arm_angle))

def crank_angle_to_sim(crank_angle):
    # for us 90 is straight up and the angle is positive CW (looking at robot from the right)
    # for sim 90 is up but we are positive CCW - so take 90=90 but we're reversed from there
    return 90 + (90 - math.degrees(crank_angle))  # same as 180 - crank_angle
    pass

def shooter_angle_to_sim(shooter_angle):
    # for the shooter our zero (horizontal or perpendicular to the arm) is the sim's 90
    return 90 - math.degrees(shooter_angle)

