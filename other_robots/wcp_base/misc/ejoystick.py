import wpilib


class EnhancedJoystick(wpilib.Joystick):
    """
    Enhanced Joystick math stolen from Team 2481's 2018 robot
    basically sets a middle zone where the inputs are squared and go back to linear outside this zone

    """

    s = 0.57
    m = 1.340508
    t = 0.381021
    b = -0.340508
    sTwist = 0.75
    mTwist = 1.56538
    tTwist = 0.54176
    bTwist = -0.56538

    def getEnhY(self) -> float:
        y = super().getY()
        scale = 1
        if y < -self.t:
            return scale * (self.m * y - self.b)

        if y > -self.t and y < self.t:
            return scale * (1 / (pow(self.s, 2.0)) * pow(y, 3.0))
        else:
            return scale * (self.m * y + self.b)

    def getEnhTwist(self):
        z = super().getZ()
        scale = 1

        if z < -self.tTwist:
            return scale * (self.mTwist * z - self.bTwist)

        if z > -self.tTwist and z < self.tTwist:
            return scale * (1 / (pow(self.sTwist, 2.0)) * pow(z, 3.0))
        else:
            return scale * (self.mTwist * z + self.bTwist)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import wpilib.simulation

    ds = wpilib.simulation.DriverStationSim()
    ds.setJoystickAxisCount(0, 3)

    stick = EnhancedJoystick(0)

    x = []
    y = []

    for v in range(-10, 11, 1):
        v /= 10.0
        x.append(v)
        ds.setJoystickAxis(0, 1, v)
        y.append(stick.getEnhY())
        # ds.setJoystickAxis(0, 2, v)
        # y.append(stick.getEnhTwist())

    plt.plot(x, y)
    plt.show()
