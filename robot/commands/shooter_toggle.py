# give ourselves three possible actions for the shooter - stop, set, and cycle through a list (for testing)

import commands2
from wpilib import SmartDashboard

from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal

class ShooterToggle(commands2.Command):

    def __init__(self, container, shooter, rpm=3000, auto_amp_slowdown=False, upper_crank: UpperCrankArmTrapezoidal=None, force=None) -> None:
        super().__init__()
        self.setName('ShooterToggle')
        self.container = container
        self.shooter = shooter
        self.upper_crank = upper_crank
        self.rpm = rpm
        self.force = force
        self.auto_amp_slowdown = auto_amp_slowdown
        self.addRequirements(shooter)  # commandsv2 version of requirements

    def initialize(self) -> None:
        # give ourselves three possible actions
        rpm = self.rpm if (not self.auto_amp_slowdown) or (self.upper_crank.get_angle()) < 0 else 2000

        print(f"!! Delivering {rpm} rpm to the shooter !!")

        if self.force == 'on':
            self.shooter.set_flywheel(rpm)
        elif self.force == 'off':
            self.shooter.stop_shooter()
        else:
            self.shooter.toggle_shooter(rpm)
        
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
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

        
