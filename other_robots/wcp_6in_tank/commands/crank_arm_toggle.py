import commands2
from wpilib import SmartDashboard


class CrankArmToggle(commands2.Command):

    SmartDashboard.putNumber('set crank arm rpm', 2500)

    def __init__(self, container, crank_arm, power=0.1, force=None) -> None:
        super().__init__()
        self.setName('CrankArmToggle')
        self.crank_arm = crank_arm
        self.container = container
        self.power = power
        self.force = force
        self.addRequirements(crank_arm)

    def initialize(self) -> None:
        rpm = SmartDashboard.getNumber('set crank arm rpm', 2500)


        if self.force == 'on':
            self.crank_arm.set_crank_arm(self.power)
        elif self.force == 'off':
            self.crank_arm.stop_crank_arm()
        else:
            self.crank_arm.toggle_crank_arm(self.power)

        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Firing {self.getName()} with force={self.force} at {self.start_time} s **", flush=True)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return False  # going to use this as a whileTrue

    def end(self, interrupted:bool) -> None:
        self.crank_arm.stop_crank_arm()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
