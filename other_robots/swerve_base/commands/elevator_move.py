import commands2
from wpilib import SmartDashboard
from subsystems.elevator import Elevator
from wpilib import DoubleSolenoid

class ElevatorMove(commands2.CommandBase):

    def __init__(self, container, elevator:Elevator, setpoint=None, direction=None, wait_to_finish=True,
                 drive_controls=False, enable_skip=False, decide_by_turret=False) -> None:
        super().__init__()
        self.setName('Elevator Move')
        self.container = container
        self.elevator = elevator
        self.setpoint = setpoint
        self.direction = direction
        self.tolerance = 10
        self.wait_to_finish = wait_to_finish  # determine how long we wait to end
        self.drive_controls = drive_controls
        self.enable_skip = enable_skip
        self.decide_by_turret = decide_by_turret

        self.addRequirements(self.elevator)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.print_start_message()
        position = self.elevator.get_height()

        positions = list(self.elevator.positions.values())  # default list of setpoints

        # only allow certain setpoints depending on if we have a game piece
        if self.drive_controls:
            if self.container.pneumatics.get_manipulator_state() == DoubleSolenoid.Value.kForward:
                positions = list(self.elevator.positions_open.values())
            else:
                positions = list(self.elevator.positions_close.values())

        # allow us to autonomously decide on how high to go based on turret position
        if self.decide_by_turret:
            turret_angle = self.container.turret.get_angle()
            if turret_angle > -20 and turret_angle < 120: # we are scoring mid or low
                self.setpoint = self.elevator.positions_close['low']  #  {'top': 950, 'low': 650}
            else:  # we are scoring high
                self.setpoint = self.elevator.positions_close['top']

        # tell the elevator to go to position
        if self.setpoint is None:
            if self.direction == 'up':
                allowed_positions = [x for x in sorted(positions) if x > position + 10 ]
                print(allowed_positions)
                temp_setpoint = sorted(allowed_positions)[0] if len(allowed_positions) > 0 else position

                if self.enable_skip and self.elevator.is_moving and len(allowed_positions) > 1:
                    temp_setpoint = sorted(allowed_positions)[1]
            else:
                allowed_positions = [x for x in sorted(positions) if x < position -10]
                print(allowed_positions)
                temp_setpoint = sorted(allowed_positions)[-1] if len(allowed_positions) > 0 else position

                if self.enable_skip and self.elevator.is_moving and len(allowed_positions) > 1:
                    temp_setpoint = sorted(allowed_positions)[-2]

            self.elevator.set_elevator_height(height=temp_setpoint, mode='smartmotion')
            print(f'Setting elevator from {position:.0f} to {temp_setpoint}')
        else:
            self.elevator.set_elevator_height(height=self.setpoint, mode='smartmotion')
            print(f'Setting elevator from {position:.0f} to {self.setpoint}')

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:  # wait for the elevator to get within x mm
            return abs(self.elevator.get_height() - self.setpoint) < self.tolerance
        else:
            return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s at {self.elevator.sparkmax_encoder.getPosition():.1f} after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")