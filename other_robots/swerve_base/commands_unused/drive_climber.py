import commands2
import wpilib
from wpilib import SmartDashboard
from subsystems.drivetrain import Drivetrain

class DriveClimber(commands2.CommandBase):

    def __init__(self, container, drive, setpoint_velocity=45, setpoint_distance=2, wait_to_finish=True) -> None:
        super().__init__()
        self.setName('DriveClimber')
        self.container = container
        self.drive = drive
        self.setpoint_velocity = setpoint_velocity  # m per minute, so 30 is 0.5 m/s
        self.setpoint_distance = setpoint_distance  # how far to go
        self.wait_to_finish = wait_to_finish  # determine how long we wait to end
        self.tolerance = 0.1  # m tolerance
        self.addRequirements(self.drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.print_start_message()
        self.start_heading = self.drive.navx.getAngle()

        # tell the drive to go to position


    def execute(self) -> None:
        # NOTE - relies on slot 2 having I and IMaxAccum to get us over the slope
        current_angle = self.drive.navx.getAngle() - self.start_heading
        kp_angle = 0.1 # 0.1 V per degrees
        lfeed, rfeed = 0,0  # initialize feeds forward
        if current_angle > 0 :  # we've turned right
            lfeed = kp_angle * current_angle
        elif current_angle < 0:  # we've turned left
            rfeed = - kp_angle * current_angle
        else:
            pass
        self.drive.drive_forwards_vel(self.setpoint_velocity, pidSlot=2, l_feed_forward=lfeed, r_feed_forward=rfeed)
        if isinstance(self.drive, Drivetrain):
            self.drive.feed()

    def isFinished(self) -> bool:
        if wpilib.RobotBase.isSimulation():
            return False
        if self.wait_to_finish:  # wait for the robot to get within some number of meters
            return abs(self.drive.get_pose.X()) < self.tolerance
        else:
            return True

    def end(self, interrupted: bool) -> None:
        self.drive.drive_forwards_vel(0, pidSlot=2)
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")