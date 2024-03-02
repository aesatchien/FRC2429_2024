import commands2
import wpilib
from wpilib import SmartDashboard

import constants
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal


class CalibrateLowerCrankByLimitSwitch(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, lower_crank: LowerCrankArmTrapezoidal) -> None:
        super().__init__()
        self.setName('Sample Name')  # change this to something appropriate for this command
        self.container = container
        self.lower_crank = lower_crank
        self.addRequirements(self.lower_crank)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.limit_reached = False
        self.lower_crank.disable_arm()
        self.lower_crank.set_voltage(-1)

    def execute(self) -> None:
        if not wpilib.RobotBase.isReal():
            print(f"!! We're in a simulation, setting lower crank encoder position to {self.lower_crank.get_angle()} + 0.01 !!")
            self.lower_crank.set_encoder_position(self.lower_crank.get_angle() - 0.01)
        else:
            print("!! We're out of the matrix, not setting no encoders !!")
        pass

    def isFinished(self) -> bool:
        # DIO is true by default so we have to negate it
        self.limit_reached = ((not self.lower_crank.get_limit_switch_state()) or
                              self.lower_crank.get_current() > constants.k_lower_crank_current_where_jammed)
        print(f"limit reached? {self.limit_reached} because have we reached the limit? {not self.lower_crank.get_limit_switch_state()} or the current is {self.lower_crank.get_current()}")
        return self.limit_reached

    def end(self, interrupted: bool) -> None:
        if self.limit_reached:
            self.lower_crank.set_encoder_position(constants.k_lower_crank_position_when_limit_switch_true)
        self.lower_crank.enable_arm()
        print()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")