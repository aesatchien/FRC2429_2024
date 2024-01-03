"""
2023 2015 CJH
Took this from 6367's more elegant approach to tricking the code to use PWMs for the drive motors if we are in
simulation mode and the real CANSparkMax otherwise
"""

import wpilib


if wpilib.RobotBase.isSimulation():

    class SparkMaxRelativeEncoder:
        def __init__(self) -> None:
            self._velocity = 0

        def getVelocity(self):
            return self._velocity

    class CANSparkMax(wpilib.Spark):
        def __init__(self, channel: int, ignored) -> None:
            super().__init__(channel)
            self._encoder = SparkMaxRelativeEncoder()

        def getEncoder(self):
            return self._encoder

        def setIdleMode(self, mode):
            pass

else:
    import rev

    CANSparkMax = rev.CANSparkMax
