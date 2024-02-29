#!/usr/bin/env python3
""" LHACK testing a servo for what I think is the climber """

import wpilib

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.servo0 = wpilib.Servo(0)
        self.servo1 = wpilib.Servo(1)
    
    def teleopInit(self) -> None:
        self.servo0.setAngle(0)
        self.servo1.setAngle(0)
        self.counter = 0

    def teleopPeriodic(self) -> None:
        if self.counter % 100 > 50:
            self.servo0.setAngle(0)
            self.servo1.setAngle(0)
            print('0 degrees')
        else:
            print('110 degrees')
            self.servo0.setAngle(90)
            self.servo1.setAngle(90)
        self.counter += 1
