from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QBrush, QPen
from ntcore import NetworkTableInstance
from pathlib import Path
from datetime import datetime
import os
import csv
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
        uic.loadUi('leo_layout_2024.ui', self)  # if this isn't in the directory, you got no program

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
        self.refresh_time = 21  # milliseconds before refreshing
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_widgets)
        self.timer.start(self.refresh_time)
        self.lap_times = []
        self.robot_prev_crashed = False
        self.robot_crashes_current_lap = 0

        self.show()

    def initialize_widgets(self):
        # Fill widget_dict with widgets that directly correspond to some networktables entry
        # less confusing but more typing than the main dashboard's method
        '''
        self.widget_dict = {
            'qlabel_lap_time': {'widget': self.qlabel_lap_time, 'entry': self.ntinst.getEntry('/SmartDashboard/latest_lap_time'), 'command': None,
                                'updater': lambda: self.qlabel_lap_time.setText(f'time: {self.ntist.getEntry('/SmartDashboard/latest_lap_time')}')},
            'qlabel_upper_crank_angle': {'widget': self.qlabel_upper_crank_angle, 'entry': self.ntinst.getEntry('/SmartDashboard/upper_arm_degrees'), 'command': None},
            'qlabel_lower_Crank_angle': {'widget': self.qlabel_lower_Crank_angle, 'entry': self.ntinst.getEntry('/SmartDashboard/crank_arm_degrees'), 'command': None}
        }
        '''
        self.qbutton_swap_sim.clicked.connect(self.increment_server)
        self.qbutton_save_practice_data.clicked.connect(self.save_practice_data)
        self.qgraphicsscene = QtWidgets.QGraphicsScene(0, 0, 8.2, 16.4)
        self.qgraphicsscene.setBackgroundBrush(Qt.gray)
        self.robot_rect = self.qgraphicsscene.addRect(0, 0, 1, 1)
        self.robot_rect.setBrush(QBrush(Qt.red))
        self.robot_rect.setPen(QPen(Qt.black))
        self.qgraphicsview_field.setScene(self.qgraphicsscene)
        self.qgraphicsview_field.rotate(-90) # -90 for blue alliance, 90 for red. TODO: add intelligent switching based on alliance


    def update_widgets(self):
        self.counter += 1

        if self.ntinst.isConnected():
            self.qlabel_nt_connected.setStyleSheet('color: green')
            self.qlabel_nt_connected.setText(f'nt connected to {self.servers[self.server_index]}')
        else:
            # todo: fix.  This is broken in the main dashboard and mine- it never goes back to being disconnected.
            self.qlabel_nt_connected.setStyleSheet('color: red')
            self.qlabel_nt_connected.setText('nt disconnected')

        # ------ LAP TAB ------

        latest_lap_time = self.ntinst.getEntry("/SmartDashboard/latest_lap_time").getDouble(-1)
        prev_lap_time = -1 if len(self.lap_times) == 0 else self.lap_times[-1]

        if self.ntinst.getEntry('/SmartDashboard/robot_crashed').getBoolean(False) and not self.robot_prev_crashed:
            self.robot_crashes_current_lap += 1
            self.robot_prev_crashed = True
        elif self.robot_prev_crashed and not self.ntinst.getEntry('/SmartDashboard/robot_crashed').getBoolean(False):
            self.robot_prev_crashed = False

        if latest_lap_time != prev_lap_time and latest_lap_time >= 0: # to exclude -1, which could be reached by a robot disconnect
            print(f'New latest lap: {latest_lap_time}s')
            self.lap_times.append(latest_lap_time)

            self.qlabel_lap_number.setText(f'lap {len(self.lap_times)}')
            self.qlabel_lap_time.setText(f'time: {latest_lap_time:.1f}')
            self.qlabel_lap_crashes.setText(f'crashes: {self.robot_crashes_current_lap}')
            if self.robot_crashes_current_lap == 0:
                self.qlabel_lap_crashes.setStyleSheet('color: rgb(15, 254, 24)')
            else:
                self.qlabel_lap_crashes.setStyleSheet('color: rgb(255, 170, 255)')

            self.robot_crashes_current_lap = 0

        # ------ DEFAULT TAB ------
        self.qlabel_lower_crank_angle.setText(f'{self.ntinst.getEntry("/SmartDashboard/upper_arm_degrees").getDouble(-999):.1f}')
        self.qlabel_upper_crank_angle.setText(f'{self.ntinst.getEntry("/SmartDashboard/crank_arm_degrees").getDouble(-999):.1f}')

        print(f"bot x: {self.ntinst.getEntry('/SmartDashboard/drive_pose').getDoubleArray([-999, -999, -999])[0]}\n" +
              f"bot y: {self.ntinst.getEntry('/SmartDashboard/drive_pose').getDoubleArray([-999, -999, -999])[1]}")

        # print(f"type of mystery array thing: {type(self.ntinst.getEntry('/SmartDashboard/drive_pose').getDoubleArray(-999))}")

        self.qgraphicsview_field.fitInView(self.qgraphicsscene.sceneRect(), Qt.KeepAspectRatio)
        self.robot_rect.setX((self.ntinst.getEntry('/SmartDashboard/drive_pose').getDoubleArray([-999, -999, -999])[0] + 3) * 1/2)
        self.robot_rect.setY((-self.ntinst.getEntry('/SmartDashboard/drive_pose').getDoubleArray([-999, -999, -999])[1] + 8.2) * 2)
        self.robot_rect.setRotation(self.ntinst.getEntry('/SmartDashboard/drive_pose').getDoubleArray([-999, -999, -999])[2])

        # self.robot_rect.setX(100*math.sin(self.counter/40))
        # self.robot_rect.setY(100*math.cos(self.counter/40))

    def increment_server(self):  # changes to next server in server list - TODO - figure our how to make this immediate
        current_server = self.servers[self.server_index]
        self.server_index = (self.server_index + 1) % len(self.servers)
        self.ntinst.setServer(server_name=self.servers[self.server_index])

        print(f'Changed server from {current_server} to {self.servers[self.server_index]}')
        # self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Changed server from {current_server} to {self.servers[self.server_index]} ... wait 5s')

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
