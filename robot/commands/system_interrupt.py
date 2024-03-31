# "feed()" is not a method of the Swerve class and will cause an error if used on a Swerve object.

import commands2
from wpilib import SmartDashboard

class SystemInterrupt(commands2.Command):  # change the name for your command

    def __init__(self, container, system, duration) -> None:
        super().__init__()
        self.setName('SystemInterrupt')  # change this to something appropriate for this command
        self.container = container
        self.system = system
        self.duration = duration
        self.addRequirements(self.system)
        self.start_time = 0

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self):
        pass
    def isFinished(self) -> bool:
        return self.container.get_enabled_time() - self.start_time > self.duration
    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_message = False
        if print_message:
            print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")