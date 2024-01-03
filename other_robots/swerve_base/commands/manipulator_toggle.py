import commands2
from wpilib import SmartDashboard
from subsystems.pneumatics import Pneumatics

class ManipulatorToggle(commands2.CommandBase):
    # Takes {open, close, or None} to {open, close or toggle} the manipulator

    def __init__(self, container, pneumatics:Pneumatics, force=None) -> None:
        super().__init__()
        self.setName('ManipulatorToggle')
        self.container = container
        self.pneumatics = pneumatics
        #self.addRequirements(pneumatics)  # commandsv2 version of requirements
        self.force = force

    def initialize(self) -> None:
        if self.force == 'open':
            self.pneumatics.set_manipulator_piston(position='open')
        elif self.force == 'close':
            self.pneumatics.set_manipulator_piston(position='close')
        else:
            self.pneumatics.toggle_manipulator()

        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Firing {self.getName()} with force={self.force} at {self.start_time} s **", flush=True)
        # SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:  
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        # print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

        
