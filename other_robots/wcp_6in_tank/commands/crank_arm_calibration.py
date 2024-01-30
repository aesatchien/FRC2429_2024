import commands2
from wpilib import SmartDashboard
import rev
from subsystems.crank_arm import CrankArm


class CrankArmCalibration(commands2.Command):

    def __init__(self, container, crank_arm:CrankArm) -> None:
        super().__init__()
        self.setName('CrankArmCalibration')
        self.container = container
        self.crank_arm = crank_arm
        self.addRequirements(self.crank_arm)  # commandsv2 version of requirements

    def runsWhenDisabled(self):  # ok to run when disabled - override the base method
        return True
    
    def initialize(self) -> None:
        self.print_start_message()
        absolute_angle = self.crank_arm.abs_encoder.getPosition()
        self.crank_arm.set_encoder_position(angle=absolute_angle)  # now we have a new maximum

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        pass

    def isFinished(self) -> bool:
        # Stop when we hit the limit switch or too much current
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s at {self.crank_arm.sparkmax_encoder.getPosition():.1f} after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")