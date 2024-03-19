import commands2
from wpilib import SmartDashboard
from subsystems.climber import Climber


class ToggleClimbServos(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, climber) -> None:
        super().__init__()
        self.setName('Toggle climb servos')  # change this to something appropriate for this command
        self.container = container
        self.climber = climber
        self.addRequirements(self.climber)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.servo_state = self.climber.toggle_climber_servos()
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message=False
        if print_end_message:
            print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s.  Are the servos open? {self.servo_state} **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s. Are the servos open? {self.servo_state}**")
