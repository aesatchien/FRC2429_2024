import commands2
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from photonlibpy.photonCamera import PhotonCamera

from subsystems.swerve import Swerve
import constants


class AutoTurn(commands2.Command):  # change the name for your command

    def __init__(self, container, drive: Swerve, target_angle) -> None:
        super().__init__()
        self.setName('Auto Turn')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.target_angle = target_angle
        self.rot_PID_controller = PIDController(4, 0, 0) # Values are taken from Pathplanner's values in swerve.py
        self.addRequirements(drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.rot_PID_controller.reset()
        self.rot_PID_controller.setSetpoint(self.target_angle)
        # self.rot_PID_controller.setTolerance(0.5) #sets the tolerance to 0.5 degrees

    def execute(self) -> None:
        desired_rotation = self.rot_PID_controller.calculate(self.drive.get_angle()) # todo: .get_angle() can return an angle beyond 360 degrees (continuous)--need to check if the PID controller accounts for that.
        self.drive.drive(0, 0, desired_rotation, fieldRelative=False, rate_limited=True)
        
    def isFinished(self) -> bool:
        return self.rot_PID_controller.atSetpoint()

    def end(self, interrupted: bool) -> None:
 
        end_time = self.container.get_enabled_time()

        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")