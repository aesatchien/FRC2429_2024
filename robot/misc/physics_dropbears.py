from __future__ import annotations

import math
import typing
import wpilib
import rev

from pyfrc.physics.core import PhysicsInterface
from wpimath.kinematics import SwerveDrive4Kinematics
from wpilib.simulation import SimDeviceSim

from robotcontainer import RobotContainer
from subsystems.swervemodule_2429 import SwerveModule
from subsystems.swerve_constants import DriveConstants as dc
from subsystems.swerve_constants import ModuleConstants as mc

if typing.TYPE_CHECKING:
    from robot import MyRobot


class SimpleSparkMotorSim:
    def __init__(
        self, motor: rev.CANSparkBase, units_per_rev: float, kV: float
    ) -> None:
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.kV = kV  # volt seconds per unit
        self.units_per_rev = units_per_rev

    def update(self, dt: float) -> None:
        voltage = self.sim_state.motor_voltage
        velocity = voltage / self.kV  # units per second
        velocity_rps = velocity * self.units_per_rev
        self.sim_state.set_rotor_velocity(velocity_rps)
        self.sim_state.add_rotor_position(velocity_rps * dt)


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller

        self.kinematics: SwerveDrive4Kinematics = dc.kDriveKinematics
        self.swerve_modules: list[SwerveModule] = RobotContainer.drive.swerve_modules

        # Motors
        self.wheels = [
            SimpleSparkMotorSim(
                module.drivingSparkMax,
                units_per_rev=1 / mc.kDrivingMotorReduction,
                kV=2.7,
            )
            for module in self.swerve_modules
        ]
        self.steer = [
            SimpleSparkMotorSim(
                module.turningSparkMax,
                units_per_rev=1 / mc.k_turning_motor_gear_ratio,
                kV=2.356,
            )
            for module in self.swerve_modules
        ]

        self.imu = SimDeviceSim("navX-Sensor", 4)
        self.imu_yaw = self.imu.getDouble("Yaw")

    def update_sim(self, now: float, tm_diff: float) -> None:
        # Enable the Phoenix6 simulated devices
        # TODO: delete when phoenix6 integrates with wpilib
        # if wpilib.DriverStation.isEnabled():
        #     phoenix6.unmanaged.feed_enable(0.1)

        for wheel in self.wheels:
            wheel.update(tm_diff)
        for steer in self.steer:
            steer.update(tm_diff)

        speeds = self.kinematics.toChassisSpeeds(
            (
                self.swerve_modules[0].get(),
                self.swerve_modules[1].get(),
                self.swerve_modules[2].get(),
                self.swerve_modules[3].get(),
            )
        )

        self.imu_yaw.set(self.imu_yaw.get() - math.degrees(speeds.omega * tm_diff))

        self.physics_controller.drive(speeds, tm_diff)