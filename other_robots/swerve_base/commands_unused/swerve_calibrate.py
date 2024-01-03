import commands2
from wpilib import SmartDashboard

import constants


class SwerveCalibrate(commands2.CommandBase):

    def __init__(self, container, swerve, samples=50) -> None:
        super().__init__()
        self.setName('SwerveCalibrate')
        self.container = container
        self.swerve = swerve
        self.samples = samples
        self.data = [ [0] * self.samples for m in self.swerve.swerve_modules ]
        self.counter = 0

        self.addRequirements(self.swerve)  # commandsv2 version of requirements

    def runsWhenDisabled(self):  # ok to run when disabled - override the base method
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.data = [ [0] * self.samples for m in self.swerve.swerve_modules ]
        self.counter = 0
        SmartDashboard.putBoolean('swerve_initialized', False)

    def execute(self) -> None:
        for idx,  m in enumerate(self.swerve.swerve_modules):
            measurement = m.get_turn_encoder()
            (self.data[idx])[self.counter % self.samples] = measurement
        self.counter += 1

    def isFinished(self) -> bool:
        return self.counter > self.samples

    def end(self, interrupted: bool) -> None:
        final_values = [0] * 4
        for idx, m in enumerate(self.swerve.swerve_modules):
            average_encoder_values = sum(self.data[idx]) / self.samples

            # nothing to do now but print the values - now that we have absolute encoders
            # previously was used for calibrating the relative encoders on the spark max

            final_values[idx] = round(average_encoder_values, 4)

        print(f"Average encoder values is {final_values}")
        # print(self.data)
        #print(f'set swerve sparkmax encoders using {average_encoder_value} to {calibrated_angle}')

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putBoolean('swerve_initialized', True)