import commands2
import rev
from wpilib import SmartDashboard

class GenericDrive(commands2.CommandBase):

    def __init__(self, container, subsystem, max_velocity, axis=None, invert_axis=False, control_type='voltage', input_type='stick', direction=1) -> None:
        super().__init__()
        self.setName('Generic Drive')
        self.container = container
        self.subsystem = subsystem
        self.max_velocity = max_velocity
        self.control_type = control_type
        self.axis = axis
        self.invert_axis = invert_axis
        self.input_type = input_type
        self.direction = direction

        self.scale = 0.5

        # Elevator, Turret, Wrist, and Arm all have pid_controller
        self.controller = self.subsystem.pid_controller

        self.addRequirements(self.subsystem)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.print_start_message()
        if self.control_type == 'voltage':
            if self.subsystem == self.container.elevator:
                self.scale = 0.35
            elif self.subsystem == self.container.turret:
                self.scale = 0.2
            elif self.subsystem == self.container.arm:
                self.scale = 0.9
            elif self.subsystem == self.container.wrist:
                self.scale = 0.65
        elif self.control_type == 'velocity':
            if self.subsystem == self.container.wrist:
                self.scale = 1


    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        # get stick value and invert if necessary (if using Y-axis)
        if self.input_type == 'stick':
            stick = self.container.co_driver_controller.getRawAxis(self.axis)

        elif self.input_type == 'dpad':
            stick = 0.5 * self.direction

        if self.invert_axis:
            stick *= -1

        velocity = stick * self.max_velocity * self.scale

        if self.control_type == 'voltage':  # do not confuse this with velocity!!!
            self.controller.setReference(12*stick*self.scale, rev.CANSparkMax.ControlType.kVoltage)
        elif self.control_type == 'velocity':
            self.controller.setReference(velocity, rev.CANSparkMax.ControlType.kVelocity, pidSlot=0)

    def isFinished(self) -> bool:
        return False  # this should be managed by the whileHeld()

    def end(self, interrupted: bool) -> None:
        self.controller.setReference(0, rev.CANSparkMax.ControlType.kVelocity, pidSlot=0)

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")
