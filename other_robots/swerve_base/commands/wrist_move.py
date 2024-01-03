import commands2
from wpilib import SmartDashboard
from subsystems.wrist import Wrist

class WristMove(commands2.CommandBase):

    def __init__(self, container, wrist:Wrist, setpoint=None, direction=None, wait_to_finish=True) -> None:
        super().__init__()
        self.setName('Wrist Move')
        self.container = container
        self.wrist = wrist
        self.setpoint = setpoint
        self.direction = direction
        self.tolerance = 3
        self.wait_to_finish = wait_to_finish  # determine how long we wait to end

        self.addRequirements(self.wrist)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.print_start_message()
        position = self.wrist.get_angle()

        elevator_height = self.container.elevator.get_height()
        ground_thresh = 200
        wrist_positions = list(self.wrist.positions.values())

        if elevator_height > ground_thresh:
            wrist_positions.remove(self.wrist.positions['floor'])

        # tell the elevator to go to position
        if self.setpoint is None:
            if self.direction == 'up':
                allowed_positions = [x for x in sorted(wrist_positions) if x > position + self.tolerance]
                print(allowed_positions)
                temp_setpoint = sorted(allowed_positions)[0] if len(allowed_positions) > 0 else position

                # skip presets when on ground. precision needed for scoring though
                if elevator_height < ground_thresh and self.wrist.is_moving and len(allowed_positions) > 1:
                    temp_setpoint = sorted(allowed_positions)[1]
            else:
                allowed_positions = [x for x in sorted(wrist_positions) if x < position - self.tolerance]
                print(allowed_positions)
                temp_setpoint = sorted(allowed_positions)[-1] if len(allowed_positions) > 0 else position

                # skip presets when on ground. precision needed for scoring though
                if elevator_height < ground_thresh and self.wrist.is_moving and len(allowed_positions) > 1:
                    temp_setpoint = sorted(allowed_positions)[-2]

            self.wrist.set_wrist_angle(angle=temp_setpoint, mode='smartmotion')
            print(f'Setting wrist from {position:.0f} to {temp_setpoint} - is_moving={self.wrist.is_moving}')
        else:
            self.wrist.set_wrist_angle(angle=self.setpoint, mode='smartmotion')
            print(f'Setting wrist from {position:.0f} to {self.setpoint}')

        self.wrist.is_moving = True  # try to raise a flag that lets us know we're in motion

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:  # wait for the wrist to get within x degrees
            return abs(self.wrist.get_angle() - self.setpoint) < self.tolerance
        else:
            return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s at {self.wrist.sparkmax_encoder.getPosition():.1f} after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")