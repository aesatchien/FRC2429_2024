import commands2
import wpilib
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
from subsystems.swerve import Swerve
from subsystems.swerve_constants import DriveConstants as dc
import math

class AutoRotateSwerve(commands2.CommandBase):

    def __init__(self, container, drive:Swerve, heading=0, find_closest_heading=True, auto=True) -> None:
        super().__init__()
        self.setName('AutoRotate')
        self.container = container
        self.drive = drive
        self.heading = heading
        self.find_closest_heading = find_closest_heading  # use to choose 0 or 180 based on which is closer
        self.addRequirements(container.drive)

        # Set PID controller so that 15 degrees will cause maximum output of 1.  Max speeds will be handled by time.
        self.heading_controller = PIDController(1 / 15, 0, 0)  # note this does not clamp to ±1 unless you do it yourself
        self.heading_controller.enableContinuousInput(0, 360)  # allow us to wrap around at 360
        self.heading_controller.setTolerance(1) # one degree

        self.auto = auto  # allows for operator to use with minimum velocity

        # should we set up a velocity transition, velocities in m/s
        self.rotate_start_time = 0  # do not confuse with the start time status message
        self.max_velocity = 2  # in m/s, so do not forget to normalize when sent to drive function
        self.slow_velocity = self.max_velocity / 2
        self.decay_rate = 10 #  20 transitions in about 0.25s, 10 is about 0.5 s to transition from high to low
        self.transition_time_center = 0.3  # center time of our transition, in seconds

        self.delta_heading = 0  # enable skipping if delta angle is less than some amount

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.rotate_start_time = wpilib.Timer.getFPGATimestamp()

        if self.find_closest_heading:
            self.heading = 0 if abs(self.drive.get_yaw()) < 90 else 180

        self.delta_heading = abs(self.drive.get_yaw() - self.heading)

        self.print_start_message()
        # setting initial speed to angle so that when we hold button it doesn't rapidly switch between 0 and proper speed
        # self.container.drive.drive(self.container.drive.navx.getPitch()/45, 0, 0, False,False)

    def execute(self) -> None:  # 50 loops per second. (0.02 seconds per loop)
        # should drive robot a max of ~1 m/s when climbing on fully tilted charge station

        current_time = wpilib.Timer.getFPGATimestamp() - self.rotate_start_time
        max_allowed_velocity = self.calculate_maximum_velocity(current_time) if self.auto else self.slow_velocity
        pid_output = self.heading_controller.calculate(self.drive.get_raw_angle(), setpoint=self.heading)
        pid_output = pid_output if abs(pid_output) <= 1 else 1 * math.copysign(1, pid_output)  # clamp at +/- 1
        pid_output = pid_output if abs(pid_output) >= 0.1 else 0.1 * math.copysign(1, pid_output)  # too slow and you get stuck waiting to move
        target_vel = pid_output * max_allowed_velocity  # meters per second
        SmartDashboard.putNumber('_target_vel', target_vel)  # actual m/s target

        # remember to scale the velocity for the drive function - divide input by max
        self.drive.drive(xSpeed=0, ySpeed=0, rot=target_vel/dc.kMaxAngularSpeed, fieldRelative=False, rate_limited=False)
        
    def isFinished(self) -> bool:
        return self.heading_controller.atSetpoint() or self.delta_heading < 5

    def end(self, interrupted: bool) -> None:
        # self.container.drive.setX()  # will not stay this way unless we are in autonomous
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    # use a logit function to transition from fast initial move to slow one at the end, like this:  ------\_____
    # starts at self.max_velocity and decays to self.min_velocity
    def calculate_maximum_velocity(self, elapsed_time):
        return self.slow_velocity + (self.max_velocity - self.slow_velocity) * (1 / (1 + math.exp(-self.decay_rate * (self.transition_time_center - elapsed_time))))
