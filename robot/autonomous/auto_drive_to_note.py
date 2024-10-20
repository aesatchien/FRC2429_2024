import commands2
import math
import wpilib
from subsystems.swerve_constants import DriveConstants as dc
from wpilib import SmartDashboard
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.filter import Debouncer
from wpimath.controller import PIDController
import constants
from subsystems.intake import Intake
from autonomous.drive_swerve_auto_velocity import DriveSwerveAutoVelocity
from subsystems.vision import Vision


class AutoDriveToNote(commands2.CommandBase):
    # 4/3 LHACK A command to automatically drive to a note using PID controllers.  Large part of it is DBP's work actually

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutomateIntake')  # change this to something appropriate for this command
        self.container = container
        self.swerve = self.container.drive
        self.intake = self.container.intake
        self.timer = wpilib.Timer()
        self.forward_pid = PIDController(2, 0, 0) # Negative sign since a positive output (forwards) decreases the positive error
        self.rotation_pid = PIDController(4, 0, 0)
        self.rotation_pid.enableContinuousInput(-180, 180)
        self.forward_pid.setTolerance(0.1)
        self.vision = container.vision

        self.move_forward_time = 1/2  # now unused
        self.minimum_velocity = 0.25
        self.move_forward_velocity = 0.25  # likely never going to be called
        self.reached_ring = False
        self.addRequirements(self.container.drive)  # commandsv2 version of requirements

        self.pid_forward_max = 0.25  # changes from 0.1, was too slow, number is % of max velociy
        self.pid_turning_max = 0.15  # changed from 0.1 at beginning to 0.15 - slightly stronger turn

    def initialize(self) -> None:
        self.forward_pid.setSetpoint(0)
        self.rotation_pid.setSetpoint(0)
        self.counter = -1
        self.done_turning = False
        self.reached_ring = False
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")


    def execute(self) -> None:

        distance = self.vision.get_orange_dist()
        rot = self.vision.get_orange_rotation()

        pid_rotation_output = self.rotation_pid.calculate(rot)
        pid_distance_output = -self.forward_pid.calculate(distance) # Negative since PID assumes negative signal -> less error when going forward (positive signal) -> less error

        if not self.reached_ring:
            if not self.forward_pid.atSetpoint():
                desired_rotation = constants.clamp(pid_rotation_output, -self.pid_turning_max, self.pid_turning_max)
                if (math.fabs(distance) < 0.1):
                    desired_forward = self.minimum_velocity
                else:
                    desired_forward = constants.clamp(pid_distance_output, -self.pid_forward_max, self.pid_forward_max) + 0.2
                self.swerve.drive(xSpeed=desired_forward, ySpeed=0, rot=desired_rotation, fieldRelative=False, rate_limited=True, keep_angle=True)
            else:  # forward pid is at setpoint, transition to pure forwards
                self.reached_ring = True
                self.counter = 0
        else:
            self.container.drive.drive(xSpeed=self.move_forward_velocity, ySpeed= 0, rot= 0,
                                       fieldRelative=False, rate_limited=False, keep_angle=True)
            self.counter += 1

    def isFinished(self) -> bool:
        # return self.counter >= 50 * self.move_forward_time
        return False  # let the button held down kill this command instead of any other end condition

    def end(self, interrupted: bool) -> None:
        # self.intake.stop_intake()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
