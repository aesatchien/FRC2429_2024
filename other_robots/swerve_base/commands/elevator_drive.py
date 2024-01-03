import commands2
from wpilib import SmartDashboard
from subsystems.elevator import Elevator
from commands2.button import JoystickButton
import rev
import constants

class ElevatorDrive(commands2.CommandBase):

    def __init__(self, container, elevator:Elevator, button:JoystickButton) -> None:
        super().__init__()
        self.setName('Elevator Drive')
        self.container = container
        self.elevator = elevator
        self.joystick_button = button

        self.addRequirements(self.elevator)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.print_start_message()

    def execute(self) -> None:  # nothing to do, the sparkmax is doing all the work
        stick = -self.container.co_driver_controller.getRawAxis(constants.k_controller_elevator_axis)
        velocity = stick  *  1000 #  mm/minute to meters per minute
        self.elevator.pid_controller.setReference(velocity, rev.CANSparkMax.ControlType.kSmartVelocity, pidSlot=0)

    def isFinished(self) -> bool:
        return False  # this should be managed by the whileHeld()

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s at {self.elevator.sparkmax_encoder.getPosition():.1f} after {end_time - self.start_time:.1f} s **")
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    def print_start_message(self):
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")