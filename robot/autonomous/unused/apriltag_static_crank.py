#J.S., 3/7/24
# Purpose: using the location of the apriltag, the robot should move to a specific location at which it can shoot the note into the speaker.
    # - ideally, this can be used in both teleop and auto.
# Pros:
    # - unlike the "auto_aim_apriltag.py" file (where the robot's (x,y) position is stationary but the crank moves), the crank is stationary.
        # - by reducing fast movement of the crank, we reduce the chance of the belt slipping or the robot tipping.
        # - also, we do not sacrifice speed because the robot can move fast (if not faster) than the crank.
# Problems:
    # - currently does not get frames from the camera. Must initialize camera to grab raw frames.
# Note-to-self:
    # - A homology is a 3x3 matrix that uses the current coordinate of points in a frame to "predict" the location of these coordinates in the next frame, given these points physically line on a plane.


import commands2
from wpilib import SmartDashboard

import robotpy_apriltag as ra
from photonlibpy.photonCamera import PhotonCamera

from autonomous.auto_turn import AutoTurn
from commands.arm_move import ArmMove
from commands.shooter_toggle import ShooterToggle
from autonomous.pathplannermaker import PathPlannerConfiguration

from subsystems.swerve import Swerve
import constants
import math
import wpimath.geometry as geo
from wpimath import objectToRobotPose

from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal
from subsystems.shooter import Shooter

class AutoAimAprilTag(commands2.Command):  # change the name for your command
    def __init__(self, container, drive: Swerve, shooter: Shooter, target_tag_id: int) -> None:
        super().__init__()
        self.setName('Static Crank Apriltag Aiming')  # change this to something appropriate for this command

        # set up an apriltag detector
        camera = 'c920' #should be made a constant.
        self.detector = ra.AprilTagDetector()
        self.detector.addFamily('tag36h11')

        # need to calculate this based on camera and resolution
        if camera == 'lifecam':
            config = ra.AprilTagPoseEstimator.Config(tagSize=0.1524, fx=342.3, fy=335.1, cx=320/2, cy=240/2) # calibration values used to convert 3D points to 2D points with minimal warping/error.
        elif camera == 'c920':
            config = ra.AprilTagPoseEstimator.Config(tagSize=0.1524, fx=439.5, fy=439.9, cx=640/2, cy=360/2) # logitech at 640x360
        else:
            raise ValueError('invalid camera')
        self.estimator = ra.AprilTagPoseEstimator(config)  # 6 inches is 0.15m

        self.container = container
        self.drive = drive
        self.shooter = shooter

        self.target_tag_id = target_tag_id
        # self.camera = PhotonCamera(constants.k_camera_name) # not sure we need a whole vision subsystem just for 1 command
        self.addRequirements(drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        
        # get 8-bit grayscale image from camera process

        frame = None
        tags = self.detector.detect(frame)
        tags = [tag for tag in tags if tag.getHamming()< 2 and tag.getDecisionMargin() > 20]

        self.target_tag = None
        for detection in tags:
            if detection.getId() == self.target_tag_id:
                self.target_tag = detection
                break

    def isFinished(self) -> bool:
        return self.target_tag is not None

    def end(self, interrupted: bool) -> None:
        pose = self.estimator.estimate(self.target_tag)
        k_gap_y = 0.4 # distance (y) from aprriltag we want to be at for shooting. 0.4 is a random number.
        k_gap_x = 0.4 # distance (x) from apriltag we want to be at for shooting. 0.4 is a random number.
        target_pos = {"x": pose.X() - k_gap_x, "y": pose.Y() - k_gap_y, "rotation": -pose.rotation().Z()}

        path = PathPlannerConfiguration().on_the_fly_path(self.drive, target_pos, 0).andThen(ShooterToggle(self.container, self.shooter, wait_for_spinup=True)).schedule()
        # todo: lock crank in place (?) Or does the crank automatically brake when stopped...(?)
        # todo: shoot note after robot in place.

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")