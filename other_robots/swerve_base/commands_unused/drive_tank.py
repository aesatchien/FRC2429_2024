import commands2
import wpilib
from wpilib import SmartDashboard
from subsystems.drivetrain import Drivetrain

class DriveTank(commands2.CommandBase):

    def __init__(self, container, drive:Drivetrain, setpoint=None, direction=None, wait_to_finish=True) -> None:
        super().__init__()
        self.setName('DriveMove')
        self.container = container
        self.drive = drive
        self.setpoint = setpoint
        self.direction = direction
        self.wait_to_finish = wait_to_finish  # determine how long we wait to end
        self.tolerance = 0.1  # m tolerance
        self.addRequirements(self.drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.print_start_message()
        left_position, right_position = self.drive.get_positions()
        # tell the drive to go to position
        if self.setpoint is not None:
            left_target = left_position + self.setpoint
            right_target = right_position - self.setpoint  # need to check if inverted matters here
            self.drive.smart_motion(left_target, slot=2, feed_forward=0)  # slot 2 has P and I to help
            print(f'Setting drive from ({left_position:.1f},{right_position:.1f}) to {self.setpoint}')

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        self.drive.feed()

    def isFinished(self) -> bool:
        if wpilib.RobotBase.isSimulation():
            return True
        if self.wait_to_finish:  # wait for the arm to get within x mm
            left_position, right_position = self.drive.get_positions()
            return abs(left_position - self.setpoint) < self.tolerance
        else:
            return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")