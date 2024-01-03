import commands2
from wpilib import SmartDashboard


class TurretInitialize(commands2.CommandBase):

    def __init__(self, container, turret, samples=25) -> None:
        super().__init__()
        self.setName('TurretInitialize')
        self.container = container
        self.turret = turret
        self.samples = samples
        self.data = [0] * self.samples
        self.counter = 0

        self.addRequirements(self.turret)  # commandsv2 version of requirements

    def runsWhenDisabled(self):  # ok to run when disabled - override the base method
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.data = [0] * self.samples
        self.counter = 0
        SmartDashboard.putBoolean('turret_initialized', False)

    def execute(self) -> None:
        self.data[self.counter % self.samples] = self.turret.analog_abs_encoder.getDistance()
        self.counter += 1

    def isFinished(self) -> bool:
        return self.counter >= self.samples

    def end(self, interrupted: bool) -> None:

        average_encoder_value = sum(self.data) / self.samples
        print(f"Average encoder value is {average_encoder_value}")
        zero_offset = 98  # 84  # 83, 23 at 180
        calibrated_angle = average_encoder_value - zero_offset # was 57, now 83  # our absolute encoder's offset from our zero is 57
        if calibrated_angle > self.turret.max_angle:  # keep us within -90 to 270, but should never happen if used correctly
            calibrated_angle = calibrated_angle - 360
            msg = f'average encoder value was {average_encoder_value} - {zero_offset} and is HIGHER than max of {self.turret.max_angle}'
        elif calibrated_angle < self.turret.min_angle:  # what if we then get greater than our max allowed?
            calibrated_angle = calibrated_angle + 360
            msg = f'average encoder value was {average_encoder_value} - {zero_offset} and is LOWER than min of {self.turret.min_angle}'
        else:
            msg = f'average encoder value was {average_encoder_value} - {zero_offset} and is within allowed range'

        self.turret.sparkmax_encoder.setPosition(calibrated_angle)
        print(msg)
        print(f'set turret sparkmax encoder using {average_encoder_value} - {zero_offset} to {calibrated_angle}')

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putBoolean('turret_initialized', True)