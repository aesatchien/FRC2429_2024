import math
import typing

import commands2
import wpilib

from subsystems.indexer import Indexer
from wpilib import SmartDashboard
from commands2.button import CommandXboxController

import constants


class IndexerByJoystick(commands2.Command):

    def __init__(self, container, indexer: Indexer):
        super().__init__()
        self.setName('indexer_by_joystick')
        self.indexer = indexer
        self.container = container
        self.controller: typing.Optional[CommandXboxController] = self.container.driver_command_controller
        self.addRequirements(self.indexer)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

    def execute(self) -> None:
        #max_linear = 1
        joystick = -1 * self.controller.getRightY()
        self.indexer.set_indexer(joystick)

        # if wpilib.RobotBase.isSimulation():
        #     SmartDashboard.putNumber('joystick', joystick)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")