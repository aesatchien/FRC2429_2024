"""
pneumatic subsystem
Shares the following with networktables:  manipulator_open, close_loop_enable
manipulator needs to reliably open and close on a double solenoid
"""
import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, Solenoid, Compressor, AnalogInput, DoubleSolenoid
from playingwithfusion import TimeOfFlight
import constants


class Pneumatics(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Pneumatics')
        self.counter = 10  # offset the periodics

        # rev version
        self.hub_type = 'rev'
        if self.hub_type == 'rev':
            rev_can_id = 15
            self.hub = wpilib.PneumaticHub(rev_can_id)  # need to figure out the REV ecosystem  it was not allowing us to actuate 20130225
            self.manipulator_piston = self.hub.makeDoubleSolenoid(constants.k_manipulator_open_port, constants.k_manipulator_closed_port)
            self.compressor = Compressor(rev_can_id, wpilib.PneumaticsModuleType.REVPH)
        # ctre version
        else:
            self.compressor = Compressor(0, wpilib.PneumaticsModuleType.CTREPCM)
            self.manipulator_piston = DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM,
                        constants.k_manipulator_open_port, constants.k_manipulator_closed_port)

        # self.pressure_sensor = AnalogInput(0)  # may skip this unless we really want one
        self.close_loop_enable = True

        competition = constants.k_competition_mode  # set this in constants
        if competition:
            self.start_compressor()
        else:
            self.stop_compressor()

        # Decide on init piston position - always closed because we will hold a game piece
        self.manipulator_closed = True
        self.manipulator_piston.set(DoubleSolenoid.Value.kReverse)  # must initialize, or no toggling

        # time of flight sensor
        self.target_distance_sensor = TimeOfFlight(constants.k_manipulator_timeofflight)
        self.target_distance_sensor.setRangingMode(TimeOfFlight.RangingMode.kShort, sampleTime=50)

        SmartDashboard.putBoolean('manipulator_closed', self.manipulator_closed)
        SmartDashboard.putBoolean('compressor_close_loop', self.close_loop_enable)

    # --------   MANIPULATOR CODE   --------------
    def set_manipulator_piston(self, position='open'):
        if position == 'open':
            self.manipulator_closed = False
            self.manipulator_piston.set(DoubleSolenoid.Value.kForward)
        elif position == 'close':
            self.manipulator_closed = True
            self.manipulator_piston.set(DoubleSolenoid.Value.kReverse)
        SmartDashboard.putBoolean('manipulator_closed', self.manipulator_closed)

    def toggle_manipulator(self):
        self.manipulator_piston.toggle()
        self.manipulator_closed = not self.manipulator_closed
        SmartDashboard.putBoolean('manipulator_closed', self.manipulator_closed)

    def get_manipulator_state(self):
        return self.manipulator_piston.get()


    # --------   COMPRESSOR CODE   --------------
    def stop_compressor(self):
        self.compressor.disable()   # should now be disable, as of 2022
        self.close_loop_enable = False
        SmartDashboard.putBoolean('compressor_close_loop', self.close_loop_enable)

    def start_compressor(self):
        self.compressor.enableDigital()  # setClosedLoopControl(True) is no longer a call in 2022
        self.close_loop_enable = True
        SmartDashboard.putBoolean('compressor_close_loop', self.close_loop_enable)

    def toggle_compressor(self):
        if self.close_loop_enable:
            self.stop_compressor()
        else:
            self.start_compressor()

    def get_target_distance(self):
        return self.target_distance_sensor.getRange()

    def periodic(self) -> None:
        
        self.counter += 1
        if self.counter % 25 == 1:
            # the compressor turns itself off and on, so we have to ask it its state
            SmartDashboard.putBoolean('compressor_state', self.compressor.enabled())
            SmartDashboard.putNumber('target distance', self.target_distance_sensor.getRange())
            # SmartDashboard.putNumber('pressure', self.get_analog_pressure())
            # todo: integrate pressure sensor into compressor class




        





  
