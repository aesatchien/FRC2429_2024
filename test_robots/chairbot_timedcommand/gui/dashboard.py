from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QBrush, QPen, QColor
from ntcore import NetworkTableInstance
from pathlib import Path
from datetime import datetime
import os
import csv
from wpimath.filter import Debouncer
import math
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
        uic.loadUi('dashboard.ui', self)  # if this isn't in the directory, you got no program

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


        self.counter = 1

        self.robot_control_mode = 'remote'

        self.refresh_time = 21  # milliseconds before refreshing
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_widgets)
        self.timer.start(self.refresh_time)

        self.show()

    def initialize_widgets(self):
        self.qbutton_swap_sim.clicked.connect(self.increment_server)
        self.qbutton_control_mode_switch.clicked.connect(self.switch_control_modes)
        self.qslider_translation.sliderReleased.connect(lambda value: self.ntinst.getEntry('datatable/thrust_limit').setValue((value * 9 / 1000) + 0.1))
        self.qslider_twist.sliderReleased.connect(lambda value: self.ntinst.getEntry('datatable/twist_limit').setValue((value * 9 / 1000) + 0.1))
    def switch_control_modes(self):
        if self.robot_control_mode == 'remote':
            self.ntinst.getEntry('datatable/control_mode').setString('onboard')
            self.robot_control_mode = 'onboard'
            self.qbutton_control_mode_switch.setText('switch to remote control')
        else:
            self.ntinst.getEntry('datatable/control_mode').setString('remote')
            self.robot_control_mode = 'remote'
            self.qbutton_control_mode_switch.setText('switch to onboard control')

    def update_widgets(self):
        self.counter += 1

        if self.ntinst.isConnected():
            self.qlabel_nt_connected.setStyleSheet('color: rgb(50, 255, 50)')
            self.qlabel_nt_connected.setText(f'nt connected to {self.servers[self.server_index]}')
            # self.setStyleSheet('background-color: rgb(27, 27, 27)')
        else:
            # todo: fix.  This is broken in the main dashboard and mine- it never goes back to being disconnected.
            self.qlabel_nt_connected.setStyleSheet('color: red')
            self.qlabel_nt_connected.setText('nt disconnected')
            # self.setStyleSheet('background-color: rgb(100, 27, 27)')

        self.qlabel_translation_percent.setText(f'{self.ntinst.getEntry("datatable/thrust_limit").getDouble(-999) * 100:.0f}%')
        self.qlabel_twist_percent.setText(f'{self.ntinst.getEntry("datatable/twist_limit").getDouble(-999) * 100:.0f}%')

    def increment_server(self):  # changes to next server in server list - TODO - figure our how to make this immediate
        current_server = self.servers[self.server_index]
        self.server_index = (self.server_index + 1) % len(self.servers)
        self.ntinst.setServer(server_name=self.servers[self.server_index])

        print(f'Changed server from {current_server} to {self.servers[self.server_index]}')
        # self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Changed server from {current_server} to {self.servers[self.server_index]} ... wait 5s')

if __name__ == "__main__":
    import sys

    # attempt to set high dpi scaling if it is detected - possible fix for high-def laptop and destop displays
    # if hasattr(QtCore.Qt, 'AA_EnableHighDpiScaling'):
    #     QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QtWidgets.QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    # compensate for dpi scaling way up above with the setAttribute calls - don't really need this now
    screen = app.screens()[0]
    dpi_logical = int(screen.logicalDotsPerInchX())
    if dpi_logical > 96:  # 150% scaling on AVIT North, vs 96 for unscaled
        print(f"We're on a scaled screen: logical dpi is {dpi_logical}")
    else:
        print(f"We're not on a scaled screen: logical dpi is {dpi_logical}")

    ui = Ui()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        print('Still has garbage collection issues. Closing.')
