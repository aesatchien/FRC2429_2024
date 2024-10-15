#!/usr/bin/env python3

import typing

import hal
import wpilib
import commands2
from wpilib.simulation import DriverStationSim
from commands.shooter_toggle import ShooterToggle

import robotcontainer
from robotcontainer import RobotContainer
from subsystems.led import Led
from ntcore import NetworkTableInstance
from subsystems.swerve import Swerve

# import warnings
# warnings.filterwarnings("ignore")


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        self.container.led.set_indicator(Led.Indicator.RAINBOW)
        self.container.drive.set_brake_mode(mode='coast')
        self.container.drive.set_use_apriltags(True)

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""

        self.container.set_start_time()  # putting this after the scheduler is bad
        self.container.drive.set_brake_mode(mode='brake')
        self.container.drive.set_use_apriltags(True)

        self.autonomousCommand = self.container.get_autonomous_command()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # self.container.led.set_indicator(Led.Indicator.POLKA) <- Arshan and Miles's pattern for robot reveal
        self.container.drive.set_brake_mode(mode='brake')
        self.container.led.set_indicator(Led.Indicator.NONE)
        self.container.drive.set_use_apriltags(True)
        self.container.set_start_time()  # putting this after the scheduler is bad

        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

        self.container.shooter.stop_shooter()
        commands2.CommandScheduler.getInstance().schedule(ShooterToggle(self.container, self.container.shooter, force='off'))

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

        #def simulationInit(self):
        # make us blue for the pathplanner  ... does not seem to work
        # hal.initialize(500)
        # DriverStationSim.setAllianceStationId(hal.AllianceStationID.kBlue2)
        # DriverStationSim.notifyNewData()

