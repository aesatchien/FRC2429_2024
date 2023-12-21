import commands2
import wpilib
from wpilib import SmartDashboard
from subsystems.drivetrain import Drivetrain
import rev

class DriveClimber(commands2.CommandBase):

    def __init__(self, container, drive:Drivetrain, setpoint_velocity=45, setpoint_distance=2, wait_to_finish=True) -> None:
        super().__init__()
        self.setName('DriveCimber')
        self.container = container
        self.drive = drive
        self.setpoint_velocity = setpoint_velocity  # m per minute, so 30 is 0.5 m/s
        self.setpoint_distance = setpoint_distance  # how far to go
        self.wait_to_finish = wait_to_finish  # determine how long we wait to end
        self.tolerance = 0.1  # m tolerance
        self.addRequirements(self.drive)  # commandsv2 version of requirements
        self.left_position = 0
        self.right_position = 0
        self.start_heading = 0
        self.r_controller = self.drive.spark_PID_controller_right_front
        self.l_controller = self.drive.spark_PID_controller_left_front
        self.control_type = rev.CANSparkMax.ControlType.kSmartVelocity

    def initialize(self) -> None:
        self.print_start_message()
        self.left_position, self.right_position = self.drive.get_positions()
        self.start_heading = self.drive.navx.getAngle()

        # tell the drive to go to position


    def execute(self) -> None:
        # NOTE - relies on slot 2 having I and IMaxAccum to get us over the slope
        current_angle = self.drive.navx.getAngle() - self.start_heading
        self.left_position, self.right_position = self.drive.get_positions()
        kp_angle = 0.1 # 0.1 V per degrees
        lfeed, rfeed = 0,0  # initialize feeds forward
        if current_angle > 0 :  # we've turned right
            lfeed = kp_angle * current_angle
        elif current_angle < 0:  # we've turned left
            rfeed = - kp_angle * current_angle
        else:
            pass
        self.l_controller.setReference(self.setpoint_velocity, self.control_type, pidSlot=2, arbFeedforward=lfeed)
        self.r_controller.setReference(self.setpoint_velocity, self.control_type, pidSlot=2, arbFeedforward=rfeed)
        self.drive.feed()

        wpilib.SmartDashboard.putNumber("left IAccum", self.l_controller.getIAccum())
        wpilib.SmartDashboard.putNumber("right IAccum", self.r_controller.getIAccum())

        wpilib.SmartDashboard.putNumber("left Output", self.drive.spark_neo_right_front.getAppliedOutput())
        wpilib.SmartDashboard.putNumber("left Current", self.drive.spark_neo_right_front.getOutputCurrent())

        wpilib.SmartDashboard.putNumber("right Output", self.drive.spark_neo_left_front.getAppliedOutput())
        wpilib.SmartDashboard.putNumber("right Current", self.drive.spark_neo_left_front.getOutputCurrent())

    def isFinished(self) -> bool:
        if wpilib.RobotBase.isSimulation():
            return False
        if self.wait_to_finish:  # wait for the arm to get within x mm
            left_position, right_position = self.drive.get_positions()
            return abs(left_position - self.setpoint_distance) < self.tolerance
        else:
            return True

    def end(self, interrupted: bool) -> None:
        self.r_controller.setReference(0, self.control_type,2)
        self.l_controller.setReference(0, self.control_type, 2)
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")