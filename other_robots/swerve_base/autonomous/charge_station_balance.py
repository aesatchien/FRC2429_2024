import commands2
import wpilib
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
from subsystems.swerve import Swerve
from subsystems.swerve_constants import DriveConstants as dc
import math

class ChargeStationBalance(commands2.CommandBase):

    def __init__(self, container, drive:Swerve, auto=True) -> None:
        super().__init__()
        self.setName('ChargeStationBalance')
        self.container = container
        self.drive = drive
        self.addRequirements(container.drive)

        # Set PID controller so that 15 degrees will cause maximum output of 1.  Max speeds will be handled by time.
        self.pitch_controller = PIDController(1 / 15, 0, 0)  # note this does not clamp to ±1 unless you do it yourself
        self.pitch_controller.enableContinuousInput(0, 360)  # allow us to wrap around at 360
        self.past_angles = [0] * 50  # not implemented yet
        self.auto = auto  # allows for operator to use with minimum velocity

        # set up velocity transition, velocities in m/s
        self.climb_start_time = 0  # do not confuse with the start time status message
        self.max_velocity = 1.5  # in m/s, so do not forget to normalize when sent to drive function
        self.min_velocity = 0.4
        self.decay_rate = 10 #  20 transitions in about 0.25s, 10 is about 0.5 s to transition from high to low
        self.transition_time_center = 0.9  # center time of our transition, in seconds

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.climb_start_time = wpilib.Timer.getFPGATimestamp()
        self.print_start_message()
        # setting initial speed to angle so that when we hold button it doesn't rapidly switch between 0 and proper speed
        # self.container.drive.drive(self.container.drive.navx.getPitch()/45, 0, 0, False,False)

    def execute(self) -> None:  # 50 loops per second. (0.02 seconds per loop)
        # should drive robot a max of ~1 m/s when climbing on fully tilted charge station

        current_time = wpilib.Timer.getFPGATimestamp() - self.climb_start_time
        max_allowed_velocity = self.calculate_maximum_velocity(current_time) if self.auto else self.min_velocity
        pid_output = self.pitch_controller.calculate(self.drive.get_pitch(), setpoint=0)
        target_vel = pid_output * max_allowed_velocity  # meters per second
        SmartDashboard.putNumber('_target_vel', target_vel)  # actual m/s target

        debugging_speed_limit = 1.49   # allow us to set a temporary test limit in m/s
        if math.fabs(target_vel) > debugging_speed_limit:
            target_vel = debugging_speed_limit * math.copysign(1, target_vel)

        # remember to scale the velocity for the drive function - divide input by max
        self.drive.drive(xSpeed=target_vel/dc.kMaxSpeedMetersPerSecond, ySpeed=0, rot=0, fieldRelative=False, rate_limited=False)
        
    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.container.drive.setX()  # will not stay this way unless we are in autonomous
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    # use a logit function to transition from fast initial climb to slow one at the end, like this:  ------\_____
    # starts at self.max_velocity and decays to self.min_velocity
    def calculate_maximum_velocity(self, elapsed_time):
        return self.min_velocity + (self.max_velocity - self.min_velocity) * (1 / (1 + math.exp(-self.decay_rate * (self.transition_time_center - elapsed_time))))
