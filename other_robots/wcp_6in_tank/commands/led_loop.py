import commands2
from wpilib import SmartDashboard
from subsystems.led import Led


class LedLoop(commands2.CommandBase):

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('LedLoop')
        self.container = container
        self.addRequirements(container.led)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Firing {self.getName()}  at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        selected = self.container.led_modes.getSelected()

        if selected != 'NONE':
            self.container.led.set_mode(selected)
        else:
            if self.container.game_piece_mode == 'cube':
                self.container.led.set_mode(Led.Mode.CUBE)
            elif self.container.game_piece_mode == 'cone':
                self.container.led.set_mode(Led.Mode.CONE)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")


