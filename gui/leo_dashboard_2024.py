from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer
from ntcore import NetworkTableInstance
import datetime
# from datetime import datetime
class Ui(QtWidgets.QMainWindow):
    # set the root dir for the project, knowing we're one deep
    root_dir = Path('.').absolute()  # set this to be in the root, not a child, so no .parent. May need to change this.
    png_dir = root_dir / 'png'
    save_dir = root_dir / 'save'

    # -------------------  INIT  --------------------------
    def __init__(self):
        super(Ui, self).__init__()
        # trick to inherit all the UI elements from the design file  - DO NOT CODE THE LAYOUT!
        uic.loadUi('layout_2024.ui', self)  # if this isn't in the directory, you got no program

        # set up network tables - TODO need to really see which of these is necessary
        self.ntinst = NetworkTableInstance.getDefault()
        self.servers = ["10.24.29.2", "127.0.0.1"] #  "roboRIO-2429-FRC.local"]  # need to add the USB one here
        self.ntinst.startClient4(identity=f'PyQt Dashboard {datetime.today().strftime("%H%M%S")}')
        self.server_index = 0  # manually do a round-robin later
        # self.ntinst.setServer("127.0.0.1",0)
        #self.ntinst.setServer(servers=self.servers)  # does not seem to work in round-robin in 2023 code
        self.ntinst.setServerTeam(2429)
        self.connected = self.ntinst.isConnected()

        # self.sorted_tree = None  # keep a global list of all the nt addresses
        # self.autonomous_list = []  # set up an autonomous list

        # self.previous_frames = 0
        self.widget_dict = {}
        # self.command_dict = {}

        self.initialize_widgets()

        self.qt_button_swap_sim.clicked.connect(self.increment_server)
        self.qt_button_save_practice_data.connect(self.save_practice_data)

        self.refresh_time = 40  # milliseconds before refreshing
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_widgets)
        self.timer.start(self.refresh_time)
        self.lap_times = []

    def initialize_widgets(self):
        self.widget_dict = {
            'qlabel_lap_time': {'widget': self.qlabel_lap_time, 'nt': '/SmartDashboard/lap_number', 'command': None},
            'qlabel_upper_crank_angle': {'widget': self.qlabel_upper_crank_angle, 'nt':'/SmartDashboard/upper_arm_degrees', 'command': None},
            'qlabel_lower_crank_angle': {'widget': self.qlabel_lower_crank_angle, 'nt': '/SmartDashboard/crank_arm_degrees', 'command': None},
        }

    def update_widgets(self):
        if self.ntist.isConnected():
            self.qlabel_nt_connected.setPalette()

    def increment_server(self):  # changes to next server in server list - TODO - figure our how to make this immediate
        current_server = self.servers[self.server_index]
        self.server_index = (self.server_index + 1) % len(self.servers)
        self.ntinst.setServer(server_name=self.servers[self.server_index])

        print(f'Changed server from {current_server} to {self.servers[self.server_index]}')
        self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Changed server from {current_server} to {self.servers[self.server_index]} ... wait 5s')

    def save_practice_data(self):

        if len(self.lap_times) == 0: return  # no point in saving nothing
        practice_data_filepath = QtWidgets.QFileDialog.getSaveFileName(self, 'Save practice data',
                                                             '.', "CSV (*.csv)")[0]
        print(practice_data_filepath)
        mode = 'a' if os.path.isfile(practice_data_filepath) else 'w'
        with open(practice_data_filepath, mode, newline='') as practice_data:
            writer = csv.writer(practice_data)
            for lap in self.lap_times:
                writer.writerow([lap])