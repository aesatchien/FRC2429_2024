import math
import commands2
from wpilib import SmartDashboard

import constants
from subsystems.upper_crank_trapezoid import UpperCrankArmTrapezoidal
from subsystems.lower_crank_trapezoid import LowerCrankArmTrapezoidal

counter = 0

class ArmCycle(commands2.Command):
    def __init__(self, container, upper_crank: UpperCrankArmTrapezoidal, lower_crank: LowerCrankArmTrapezoidal, direction=None, desired_position : str=None) -> None:
        super().__init__()
        self.setName('ArmCycle')
        self.container = container
        self.upper_crank = upper_crank
        self.lower_crank = lower_crank
        self.direction = direction
        self.desired_position = desired_position
        self.addRequirements(self.upper_crank)
        self.addRequirements(self.lower_crank)

        self.crank_presets = {
            'intake': {'upper': math.radians(-80), 'lower': math.radians(70)},
            'shoot': {'upper': math.radians(-55), 'lower': math.radians(90)},
            'shoot2': {'upper': math.radians(-35), 'lower': math.radians(90)},
            'amp': {'upper': math.radians(50), 'lower': math.radians(100)},
            'trap': {'upper': math.radians(110), 'lower': math.radians(110)},
        }

    def initialize(self) -> None:
        global counter
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)

        if self.direction is None or self.direction == 'up':
            counter += 1

        elif self.direction == 'down':
            counter -= 1
            if counter < 0:
                counter = len(self.crank_presets)-1

        if self.direction is None:
            self.active_mode = self.desired_position
        else:
            self.active_mode = list(self.crank_presets.keys())[counter % len(self.crank_presets)]

        self.container.arm_mode = self.active_mode

        self.upper_desired_position = self.crank_presets[self.active_mode]['upper']
        self.lower_desired_position = self.crank_presets[self.active_mode]['lower']

        print("\n" + f"** Started {self.getName()} at {self.start_time} with mode {self.active_mode} counter {counter} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s with mode {self.active_mode} counter {counter}**")

    def execute(self) -> None:
        print(self.active_mode)
        if self.lower_crank.get_angle() < constants.k_min_lower_crank_angle_where_deploying_upper_crank_safe_rad:
            # Fold the upper crank since the lower crank is too low
            self.upper_crank.set_goal(self.crank_presets['intake']['upper'])
        else:
            self.upper_crank.set_goal(self.upper_desired_position)

        if self.upper_crank.get_angle() > constants.k_max_upper_crank_where_retracting_lower_crank_safe_rad:
            # Keep the lower crank up till the upper crank folds
            if self.active_mode == 'amp':
                print("setting lower crank goal to " + str(math.degrees(self.crank_presets['amp']['lower'])))
                self.lower_crank.set_goal(self.crank_presets['amp']['lower'])
            else:
                self.lower_crank.set_goal(self.crank_presets['shoot']['lower'])
                print("setting loewr crank goal to 90 deg")
        else:
            self.lower_crank.set_goal(self.lower_desired_position)

    def isFinished(self) -> bool:
        return abs(self.lower_crank.get_angle() - self.lower_desired_position) < 0.1 and abs(self.upper_crank.get_angle() - self.upper_desired_position) < 0.1

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
