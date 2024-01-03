import commands2

class CompressorToggle(commands2.CommandBase):

    def __init__(self, container, pneumatics, force=None) -> None:
        super().__init__()
        self.setName('CompressorToggle')
        self.pneumatics = pneumatics
        self.container = container
        self.force = force

        self.addRequirements(pneumatics)  # commandsv2 version of requirements

    def initialize(self) -> None:
        if self.force == "start":
            self.pneumatics.start_compressor()
        elif self.force == "stop":
            self.pneumatics.stop_compressor()
        else:
            self.pneumatics.toggle_compressor()
        
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        # SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")


    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:  
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        # print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

        
