import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, DriverStation
from ntcore import NetworkTableInstance

import constants


class Vision(SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Vision')
        self.counter = 0
        self.inst = NetworkTableInstance.getDefault()
        self.photonvision_table = self.inst.getTable("photonvision/Microsoft_LifeCam_HD-3000")
        self.photonvision_yaw = self.photonvision_table.getDoubleTopic("targetYaw").subscribe(0)
        self.photonvision_has_target = self.photonvision_table.getBooleanTopic("hasTarget").subscribe(False)
        self.photonvision_target_pose = self.photonvision_table.getDoubleTopic("targetPose").subscribe(0)

    def periodic(self) -> None:
        self.counter += 1
        if self.counter % 10 == 0:
            tag_acquired = self.photonvision_has_target.get()

            wpilib.SmartDashboard.putNumber(keyName='pv_yaw', value=self.photonvision_yaw.get())
            wpilib.SmartDashboard.putBoolean(keyName='pv_has_target', value=tag_acquired)
            wpilib.SmartDashboard.putNumber(keyName='pv_target_pose', value=self.photonvision_target_pose.get())