from commands2 import TrapezoidProfileSubsystem
from wpimath.trajectory import TrapezoidProfile

class Crank(TrapezoidProfileSubsystem):
    def __init__(self):
        super().__init__(
            TrapezoidProfile.Constraints(
                constn
            )
        )