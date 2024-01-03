import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, DriverStation
from ntcore import NetworkTableInstance

import constants


class Vision(SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Vision')
        self.counter = 0
        self.ntinst = NetworkTableInstance.getDefault()

        self.pole_targets = 0
        self.pole_distance = 0
        self.pole_rotation = 0

        self.cone_targets = 0
        self.cone_distance = 0
        self.cone_rotation = 0

        self.cube_targets = 0
        self.cube_distance = 0
        self.cube_rotation = 0

        self.camera_dict = {'green': {}, 'tags': {}, 'yellow': {}, 'purple': {}}
        self.camera_values = {}

        self.armcam_table = NetworkTableInstance.getDefault().getTable('ArmCam')
        self.turretcam_table = NetworkTableInstance.getDefault().getTable('BottomCam')
        # print(f'Armcam keys: {self.armcam_table.getKeys()}')

        #self.armcam_table.putBoolean('training', False)
        #self.armcam_table.putString('training_color', 'yellow')
        #self.training_chooser = wpilib.SendableChooser()
        #[self.training_chooser.addOption(c, c) for c in self.camera_dict.keys()]
        # wpilib.SmartDashboard.putData('\Armcam\training_choices', self.training_chooser)


        self.relay = wpilib.Relay(0, direction=wpilib.Relay.Direction.kForwardOnly)
        self.relay.set(wpilib.Relay.Value.kForward)
        self.relay_state = True

        SmartDashboard.putBoolean('relay_state', self.relay_state)

        for key in self.camera_dict.keys():  # colors on top
            self.camera_dict[key].update({'targets_entry': self.armcam_table.getDoubleTopic(f"{key}/targets").subscribe(0)})
            self.camera_dict[key].update({'distance_entry': self.armcam_table.getDoubleTopic(f"{key}/distance").subscribe(0)})
            self.camera_dict[key].update({'strafe_entry': self.armcam_table.getDoubleTopic(f"{key}/strafe").subscribe(0)})
            self.camera_dict[key].update({'rotation_entry': self.armcam_table.getDoubleTopic(f"{key}/rotation").subscribe(0)})
            self.camera_values[key] = {}
            self.camera_values[key].update({'targets': 0})
            self.camera_values[key].update({'distance': 0})
            self.camera_values[key].update({'rotation': 0})
            self.camera_values[key].update({'strafe': 0})

        for key in ['tags']:  # tag from bottom
            self.camera_dict[key].update({'targets_entry': self.turretcam_table.getDoubleTopic(f"{key}/targets").subscribe(0)})
            self.camera_dict[key].update({'distance_entry': self.turretcam_table.getDoubleTopic(f"{key}/distance").subscribe(0)})
            self.camera_dict[key].update({'strafe_entry': self.turretcam_table.getDoubleTopic(f"{key}/strafe").subscribe(0)})
            self.camera_dict[key].update({'rotation_entry': self.turretcam_table.getDoubleTopic(f"{key}/rotation").subscribe(0)})

            self.camera_values[key] = {}
            self.camera_values[key].update({'targets': 0})
            self.camera_values[key].update({'distance': 0})
            self.camera_values[key].update({'rotation': 0})
            self.camera_values[key].update({'strafe': 0})

        # print(self.camera_dict)
        # print(self.camera_values)

    def set_relay(self, state):
        if state:
            self.relay.set(wpilib.Relay.Value.kForward)
            self.relay_state = True
        else:
            self.relay.set(wpilib.Relay.Value.kOff)
            self.relay_state = False
        SmartDashboard.putBoolean('relay_state', self.relay_state)

    def target_available(self, target):
        if target == 'tags':
            return self.camera_dict['tags']['targets_entry'].get() > 0
        elif target == 'green':
            self.camera_dict['green']['targets_entry'].get() > 0

    def get_tag_strafe(self):
        tag_available = self.camera_dict['tags']['targets_entry'].get() > 0
        if tag_available > 0:
            return self.camera_dict['tags']['strafe_entry'].get()
        else:
            return 0  # it would do this anyway because it defaults to zero

    def get_green_strafe(self):
        green_available = self.camera_dict['green']['targets_entry'].get() > 0
        if green_available > 0:
            return self.camera_dict['green']['strafe_entry'].get()
        else:
            return 0  # it would do this anyway because it defaults to zero
        
    def get_tag_dist(self):
        tag_available = self.camera_dict['tags']['targets_entry'].get() > 0
        if tag_available > 0:
            return self.camera_dict['tags']['distance_entry'].get()
        else:
            return 0
        
    def get_green_dist(self):
        green_available = self.camera_dict['green']['targets_entry'].get > 0
        if green_available > 0:
            return self.camera_dict['tags']['distance_entry'].get()
        else:
            return 0


    def periodic(self) -> None:
        self.counter += 1

        # update x times a second
        if self.counter % 20 == 0:
            if wpilib.RobotBase.isSimulation():
                SmartDashboard.putNumber('match_time', wpilib.Timer.getFPGATimestamp())
            else:
                SmartDashboard.putNumber('match_time', DriverStation.getMatchTime())

            for key in self.camera_dict.keys():
                self.camera_values[key]['targets'] = self.camera_dict[key]['targets_entry'].get()
                self.camera_values[key]['distance'] = self.camera_dict[key]['distance_entry'].get()
                self.camera_values[key]['rotation'] = self.camera_dict[key]['rotation_entry'].get()
                self.camera_values[key]['strafe'] = self.camera_dict[key]['strafe_entry'].get()

            wpilib.SmartDashboard.putBoolean('green_targets_exist', self.camera_dict['green']['targets_entry'].get() > 0)
            wpilib.SmartDashboard.putBoolean('tag_targets_exist', self.camera_dict['tags']['targets_entry'].get() > 0)
            wpilib.SmartDashboard.putNumber('tag_strafe', self.camera_dict['tags']['strafe_entry'].get())

            # update pole values separately
            #self.pole_targets = self.camera_dict['green']['targets_entry'].getDouble(0)
            #self.pole_distance = self.camera_dict['green']['distance_entry'].getDouble(0)
            #self.pole_rotation = self.camera_dict['green']['rotation_entry'].getDouble(0)

            self.set_relay(SmartDashboard.getBoolean('relay_state', False))