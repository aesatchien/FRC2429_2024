import commands2
from wpilib import SmartDashboard
from subsystems.elevator import Elevator


class ElevatorMoveByTurret(commands2.CommandBase):
    # LHACK, moves elevator to mid or hi based on turret position
    def __init__(self, container, elevator: Elevator, turret, wait_to_finish=True) -> None:
        super().__init__()
        self.setName('ElevatorMoveByTurret')
        self.wait_to_finish = wait_to_finish
        self.container = container
        self.elevator = elevator
        self.turret = turret
        self.tolerance = 10

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        scoring_on_hi = self.turret.get_angle() > 90 or self.turret.get_angle() < 270
        if scoring_on_hi: self.elevator_setpoint = self.elevator.positions['top']
        else: self.elevator_setpoint = self.elevator.positions['low']
        self.elevator.set_elevator_height(self.elevator_setpoint)
        

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return abs(self.elevator_setpoint - self.elevator.get_height) < self.tolerance
        else:
            return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")