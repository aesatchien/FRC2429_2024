import commands2
from wpilib import SmartDashboard
from subsystems.pneumatics import Pneumatics
from subsystems.led import Led


class ManipulatorAutoGrab(commands2.CommandBase):

    def __init__(self, container, pneumatics: Pneumatics) -> None:
        super().__init__()
        self.setName('ManipulatorAutoGrab')
        self.container = container
        self.pneumatics = pneumatics
        self.has_game_piece = False
        self.counter = 0
        self.timeout = 0.5
        self.addRequirements(pneumatics)

    def initialize(self) -> None:
        # ensure manipulator is open during initialization
        self.has_game_piece = False
        self.counter = 0
        self.pneumatics.set_manipulator_piston(position='open')
        print(round(self.container.get_enabled_time(), 2))
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        target_distance = self.pneumatics.get_target_distance()
        grab_distance = 200 # mm

        if not self.has_game_piece and target_distance <= grab_distance and target_distance >= 50:
            self.pneumatics.set_manipulator_piston(position='close')
            self.has_game_piece = True

        if self.has_game_piece:
            self.counter += 1

            if self.counter / 50 >= self.timeout:
                # start flashing so driver knows to leave
                self.container.led.set_indicator(Led.Indicator.PICKUP_COMPLETE)

    def isFinished(self) -> bool:
        # whileHeld
        return False

    def end(self, interrupted: bool) -> None:
        self.container.led.set_indicator(Led.Indicator.NONE)

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
