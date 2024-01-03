import commands2
from wpilib import SmartDashboard
from wpimath.filter import MedianFilter

from subsystems.pneumatics import Pneumatics
from subsystems.wrist import Wrist

class ToggleGroundPickup(commands2.CommandBase):
    def __init__(self, container, pneumatics: Pneumatics, wrist: Wrist, button: int, target_distance=254, timeout=2) -> None:
        super().__init__()
        self.setName('ToggleGroundPickup')

        self.container = container
        self.penumatics = pneumatics
        self.wrist = wrist
        
        self.game_piece_presets = {
            'cone': 0,
            'cube': wrist.positions['floor'],
        }

        self.distance_sensor = pneumatics.target_distance_sensor
        self.median_filter = MedianFilter(5)
        self.target_distance = target_distance

        self.button = button
        self.timeout = timeout

        #ASSUMPTIONS:
        # Turret is already zeroed
        # Elevator is already down
        # Wrist already stowed
        # Driver is driving up to the cone.

    def initialize(self) -> None:
        # open manipulator
        self.pneumatics.set_manipulator_piston(position='open')
        
        # deploy wrist
        self.wrist_setpoint = self.game_piece_presets[self.container.game_piece_mode]
        self.wrist.set_wrist_angle(angle=self.wrist_setpoint)

        self.canceled = False
        self.counter = 0


    def execute(self) -> None: #50 times a second
        distance = self.median_filter.calculate(self.distance_sensor.getRange())

        if  distance <= self.target_distance:
            self.counter += 1
            self.pneumatics.set_manipulator_piston(position='close')
            #not sure what the stop condition for this is...(I would rather avoid a timer)

        self.canceled = self.container.co_driver_controller.getRawButtonPressed(self.button)
        if self.canceled:
            self.pneumatics.set_manipulator_piston(position='close')

    def isFinished(self) -> bool:
        stop = self.counter >= self.timeout * 50 or self.canceled
        return stop
        # or emergency button pressed


    def end(self, interrupted: bool) -> None:
        # stow wrist
        self.wrist.set_wrist_angle(angle=self.wrist.positions['stow'])
        
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f'** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **')

    def print_start_message(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")
