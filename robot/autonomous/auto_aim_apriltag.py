import commands2
from wpilib import SmartDashboard
from photonlibpy.photonCamera import PhotonCamera
from autonomous.auto_turn import AutoTurn

from subsystems.swerve import Swerve
import constants


class AutoAimAprilTag(commands2.Command):  # change the name for your command

    def __init__(self, container, drive: Swerve, target_tag_id: int) -> None:
        super().__init__()
        self.setName('Auto aim to AprilTag')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.target_tag_id = target_tag_id
        self.camera = PhotonCamera(constants.k_camera_name) # not sure we need a whole vision subsystem just for 1 command
        self.addRequirements(drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        results = self.camera.getLatestResult()
        targets = results.getTargets()
        self.goodAprilTag = None
        for detection in targets:
            if detection.getFiducialId() == self.target_tag_id:
                self.goodAprilTag = detection
                break

    def isFinished(self) -> bool:
        return self.goodAprilTag is not None

    def end(self, interrupted: bool) -> None:
        AutoTurn(self.goodAprilTag.getYaw()).schedule()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")