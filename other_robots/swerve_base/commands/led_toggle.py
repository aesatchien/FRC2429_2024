import commands2
from wpilib import SmartDashboard
from subsystems.led import Led


class LedToggle(commands2.CommandBase):

    counter = 0

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('LedToggle')
        self.container = container
        self.addRequirements(container.led)
        self.modes = [
            'cone',
            'cube',
        ]

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Firing {self.getName()}  at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        self.counter += 1
        active_mode = self.modes[self.counter % len(self.modes)]
        self.container.game_piece_mode = active_mode

        if active_mode == 'cone':
            self.container.led.set_mode(Led.Mode.CONE)
        elif active_mode == 'cube':
            self.container.led.set_mode(Led.Mode.CUBE)

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")


