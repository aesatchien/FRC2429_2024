import commands2
from wpilib import SmartDashboard
from photonlibpy.photonCamera import PhotonCamera
from autonomous.auto_turn import AutoTurn
from commands.arm_move import ArmMove

from subsystems.swerve import Swerve
import constants
import math

from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal

class AutoAimAprilTag(commands2.Command):  # change the name for your command
    def __init__(self, container, drive: Swerve, upper_crank: UpperCrankArmTrapezoidal, lower_crank: LowerCrankArmTrapezoidal, target_tag_id: int) -> None:
        super().__init__()
        self.setName('Auto aim to AprilTag')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.upper_crank = upper_crank
        self.lower_crank = lower_crank
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

        measurements_recorded = False
        if measurements_recorded and self.goodAprilTag is not None:
            # AutoTurn(self.goodAprilTag.getYaw()).schedule()
            alpha = self.goodAprilTag.getBestCameraToTarget().X()
            beta = self.goodAprilTag.getBestCameraToTarget().Z()

            #constants; will keep here for now, but should be moved to constants file (if we decide to pursue this route for apriltag honing)
            k_h = 0.0 #vertical distance from the center of the apriltag to the bottom of the speaker's opening.
            k_l = 0.0 #vertical distance from the the bottom of the speaker's opening to the midpoint of the speaker's opening
            k_h_prime = 0.0 #vertical distance from the camera to the uppder crank.
            k_width_prime = 0.0 #horizontal distance from the camera to the lower crank.
            k_width = 0.0 #horizontal distance from the speaker's vertical to the midpont of the speaker's opening.
            x = alpha + k_width_prime - k_width
            y = beta + k_h + k_l - k_h_prime
            phi = math.atan2(y/x)

            AutoTurn(self.goodAprilTag.getYaw()).schedule() #face/center onto the apriltag
            ArmMove(self.lower_crank, degrees=90, absolute=True).andThen(ArmMove(self.upper_crank, degrees=math.degrees(phi), absolute=True)).schedule() #angle the crank towards the speaker's opening
            #todo: unsure if something will break if we move both upper and lower crank at the same time. But if we can, we should because that's faster.
            #todo: is "vertically straight" 90 degrees or 0 degrees?
        else:
            print("No AprilTag found")
            SmartDashboard.putString("alert", "No AprilTag found")
            #flash LEDs red (?) to indicate no april tag found.

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")