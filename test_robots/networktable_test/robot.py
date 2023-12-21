#!/usr/bin/env python3

import wpilib
import photonvision
import ntcore
from wpilib import Spark
from wpimath.filter import MedianFilter
import rev

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.counter = 0
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable("datatable")
        self.counter_pub = self.table.getDoubleTopic("counter").publish()
        self.number_pub = self.table.getDoubleTopic("number").publish()
        self.photonvision_table = self.inst.getTable("photonvision/Microsoft_LifeCam_HD-3000")
        self.photonvision_yaw = self.photonvision_table.getDoubleTopic("targetYaw").subscribe(0)
        self.number_sub = self.table.getDoubleTopic("number").subscribe(11)
        self.number_pub.set(12)

    def teleopPeriodic(self) -> None:
        pass

    def robotPeriodic(self) -> None:
        self.counter += 1
        if self.counter % 10 == 0:
            wpilib.SmartDashboard.putNumber(keyName='counter', value=self.counter)
            wpilib.SmartDashboard.putNumber(keyName='yaw', value=self.photonvision_yaw.get())
            wpilib.SmartDashboard.putNumber(keyName='number_read', value=self.number_sub.get())

            self.counter_pub.set(self.counter)

    def clamp(self, x):
        return min(1, max(-1, x))

if __name__ == "__main__":
    wpilib.run(MyRobot)


#!/usr/bin/env python3

import ntcore
import wpilib


class EasyNetworkTableExample(wpilib.TimedRobot):
    def robotInit(self) -> None:
        # Get the default instance of NetworkTables that was created automatically
        # when the robot program starts
        inst = ntcore.NetworkTableInstance.getDefault()

        # Get the table within that instance that contains the data. There can
        # be as many tables as you like and exist to make it easier to organize
        # your data. In this case, it's a table called datatable.
        table = inst.getTable("datatable")

        # Start publishing topics within that table that correspond to the X and Y values
        # for some operation in your program.
        # The topic names are actually "/datatable/x" and "/datatable/y".
        self.xPub = table.getDoubleTopic("x").publish()
        self.yPub = table.getDoubleTopic("y").publish()

        self.x = 0
        self.y = 0

    def teleopPeriodic(self) -> None:
        # Publish values that are constantly increasing.
        self.xPub.set(self.x)
        self.yPub.set(self.y)
        self.x += 0.05
        self.y += 1.0