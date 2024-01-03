import commands2
from wpilib import SmartDashboard
from subsystems.arm import Arm


class ArmMoveByTurret(commands2.CommandBase):  # change the name for your command
    # LHACK, moves arm to mid or hi based on turret position
    def __init__(self, container, arm: Arm, turret, wait_to_finish = True) -> None:
        # is there really no better way to do this than to create an entire command?
        super().__init__()
        self.setName('ArmMoveByTurret')
        self.wait_to_finish = wait_to_finish
        self.container = container
        self.arm = arm
        self.turret = turret
        self.tolerance = 10

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        scoring_on_hi = self.turret.get_angle() > 90 or self.turret.get_angle() < 270
        if scoring_on_hi: self.arm_setpoint = self.arm.positions['full']
        else: self.arm_setpoint = self.arm.positions['middle']
        self.arm.set_arm_extension(self.arm_setpoint)
        

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return abs(self.arm_setpoint - self.arm.get_extension) < self.tolerance
        else:
            return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")