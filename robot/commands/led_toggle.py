import commands2
from wpilib import SmartDashboard
from subsystems.led import Led


class LedToggle(commands2.Command):

    counter = 0

    def __init__(self, container, force = None) -> None:
        super().__init__()
        self.setName('LedToggle')
        self.container = container
        self.force = force
        self.addRequirements(container.led)
        self.modes = [
            'ring',
            'cube',
        ]
        self.indicators = [
            self.container.led.indicator.READY_SHOOT,
            self.container.led.indicator.AMP,
            self.container.led.indicator.INTAKE,
            self.container.led.indicator.PICKUP_COMPLETE,
            self.container.led.indicator.RAINBOW,
            self.container.led.indicator.INTAKE_ON,
            self.container.led.indicator.NONE,
            self.container.led.indicator.POLKA
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
        active_mode = self.indicators[self.counter % len(self.indicators)]
        # active_mode = self.modes[self.counter % len(self.modes)]
        self.container.game_piece_mode = active_mode

        # if active_mode == 'PICKUP_COMPLETE':
        #   self.container.led.set_mode(Led.Indicator.PICKUP_COMPLETE)
        # elif active_mode == 'READY_SHOOT':
        #   self.container.led.set_mode(Led.Indicator.READY_SHOOT)
        #
        # self.container.led.set_indicator(active_mode)
        # print(f"** Light mode: {active_mode}**",flush=True)
        # SmartDashboard.putString(f"alert", f"** Light mode: {active_mode}  **")

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")


