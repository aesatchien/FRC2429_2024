# pyqt example for teaching GUI development for FRC dashboard
# make sure to pip install pyqt5 pyqt5-tools

# print(f'Loading Modules ...', flush=True)
import time
from datetime import datetime
from pathlib import Path
import urllib.request
import cv2
import numpy as np

from PyQt5 import QtCore, QtGui, QtWidgets, uic
from PyQt5.QtCore import Qt, QTimer, QEvent, QThread, QObject, pyqtSignal
#from PyQt5.QtWidgets import  QApplication, QTreeWidget, QTreeWidgetItem

import qlabel2

from ntcore import NetworkTableType, NetworkTableInstance

#print(f'Initializing GUI ...', flush=True)


# Worker class for the video thread
class CameraWorker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)

    def __init__(self, qtgui):
        super().__init__()
        self.qtgui = qtgui

    def stop(self):
        self.running = False

    def run(self):
        """Blocking task that may take a long time to run"""
        self.running = True
        while self.running:
            if self.qtgui.qradiobutton_autoswitch.isChecked():  # auto determine which camera to show
                # shooter_on = self.qtgui.widget_dict['qlabel_shooter_indicator']['entry'].getBoolean(False)
                elevator_low = self.qtgui.widget_dict['qlcd_elevator_height']['entry'].getDouble(100) < 100
                url = self.qtgui.camera_dict['ShooterCam'] if elevator_low else self.qtgui.camera_dict['BallCam']
            else:
                url = self.qtgui.camera_dict[self.qtgui.qcombobox_cameras.currentText()]  # figure out which url we want
            # stream = urllib.request.urlopen('http://10.24.29.12:1187/stream.mjpg')
            # get and display image
            cap = cv2.VideoCapture(url)
            if not cap.isOpened():
                print("Cannot open stream")
                return
            try:
                ret, frame = cap.read()
                pixmap = self.qtgui.convert_cv_qt(frame, self.qtgui.qlabel_camera_view)
                self.qtgui.qlabel_camera_view.setPixmap(pixmap)
                #self.qtgui.qlabel_camera_view.repaint()  # do not repaint in the thread.  the main loop takes care of that.
                # self.progress.emit(1)
            except Exception as e:
                print(f'cv error: {e}')
                # should I stop the thread here? or just let the user fix it by restarting manually?
        self.finished.emit()

class Ui(QtWidgets.QMainWindow):
    # set the root dir for the project, knowing we're one deep
    root_dir = Path('.').absolute()  # set this to be in the root, not a child, so no .parent. May need to change this.
    png_dir = root_dir / 'png'
    save_dir = root_dir / 'save'

    # -------------------  INIT  --------------------------
    def __init__(self):
        super(Ui, self).__init__()
        # trick to inherit all the UI elements from the design file  - DO NOT CODE THE LAYOUT!
        uic.loadUi('layout_2023.ui', self)  # if this isn't in the directory, you got no program

        # set up network tables
        self.ntinst = NetworkTableInstance.getDefault()
        self.servers = ["10.24.29.2", "127.0.0.1"] #  "roboRIO-2429-FRC.local"]  # need to add the USB one here
        #self.ntinst.startClient3(identity=f'PyQt Dashboard NT3 {datetime.today().strftime("%H%M%S")}')
        self.ntinst.startClient4(identity=f'PyQt Dashboard NT4 {datetime.today().strftime("%H%M%S")}')
        self.server_index = 0  # manually do a round-robin later
        # self.ntinst.setServer("127.0.0.1",0)
        #self.ntinst.setServer(servers=self.servers)  # does not seem to work in round-robin in 2023 code
        self.ntinst.setServerTeam(2429)
        self.connected = self.ntinst.isConnected()
        self.sorted_tree = None  # keep a global list of all the nt addresses
        self.autonomous_list = []  # set up an autonomous list

        self.refresh_time = 50  # milliseconds before refreshing
        self.widget_dict = {}
        self.command_dict = {}
        self.camera_enabled = False
        self.thread = None
        self.camera_dict = {'BallCam': 'http://10.24.29.12:1186/stream.mjpg',
                            'ShooterCam': 'http://10.24.29.12:1187/stream.mjpg',
                            'Raw Balls': 'http://10.24.29.12:1181/stream.mjpg',
                            'Raw Shooter': 'http://10.24.29.12:1182/stream.mjpg'}

        self.initialize_widgets()
        #QTimer.singleShot(2000, self.initialize_widgets())  # wait 2s for NT to initialize

        # all of your setup code goes here - linking buttons to functions, etc (move to seperate funciton if too long)

        # menu items
        self.qaction_show_hide.triggered.connect(self.toggle_network_tables)  # show/hide networktables
        self.qaction_refresh.triggered.connect(self.refresh_tree)

        # widget customization
        #self.qlistwidget_commands.setStyleSheet("QListView::item:selected{background-color: rgb(255,255,255);color: rgb(0,0,0);}")
        self.qlistwidget_commands.clicked.connect(self.command_list_clicked)
        self.qcombobox_autonomous_routines.currentTextChanged.connect(self.update_routines)
        self.qt_text_entry_filter.textChanged.connect(self.filter_nt_keys_combo)
        self.qcombobox_nt_keys.currentTextChanged.connect(self.update_selected_key)
        self.qt_tree_widget_nt.clicked.connect(self.qt_tree_widget_nt_clicked)

        self.qt_text_entry_filter.installEventFilter(self)
        self.qt_text_new_value.installEventFilter(self)

        # button connections
        self.qt_button_set_key.clicked.connect(self.update_key)
        self.qt_button_test.clicked.connect(self.test)
        # self.qt_button_camera_enable.clicked.connect(lambda _: setattr(self, 'camera_enabled', not self.camera_enabled))
        self.qt_button_camera_enable.clicked.connect(self.toggle_camera_thread)

        # hide networktables
        self.qt_tree_widget_nt.hide()

        # at the end of init, you need to show yourself
        self.show()

        # set up thef refresh
        self.counter = 1
        self.previous_time = time.time()

        # set up the refresh
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_widgets)
        self.timer.start(self.refresh_time)


        # if you need to print out the list of children
        # children = [(child.objectName()) for child in self.findChildren(QtWidgets.QWidget) if child.objectName()]
        # children.sort()
        # for child in children:
        #    print(child)

    # ------------------- FUNCTIONS, MISC FOR NOW  --------------------------

    def check_url(self, url):
        try:
            code = urllib.request.urlopen(url, timeout=0.1).getcode()
            print(f'return code is {code}')
            if code == 200:
                return True
        except Exception as e:
            print(f'Failed: {e}')
        return False

    def update_selected_key(self):
        x = self.ntinst.getEntry(self.qcombobox_nt_keys.currentText()).getValue()
        if x is not None:
            self.qt_text_current_value.setPlainText(str(x.value()))

    def convert_cv_qt(self, cv_img, qlabel):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(qlabel.width(), qlabel.height(), Qt.KeepAspectRatio)
        return QtGui.QPixmap.fromImage(p)

    def toggle_camera_thread(self):
        # ToDo: check to see if the thread is running, then start again

        # check if server is running
        if self.check_url(self.camera_dict['BallCam']) or self.check_url('ShooterCam'):
            if self.thread is None:  # first time through we need to make the thread
                self.thread = QThread()  # create a QThread object
                self.worker = CameraWorker(qtgui=self)  # create a CameraWorker object, pass it the main gui
                self.worker.moveToThread(self.thread)  # move the worker to the thread
                # connect signals and slots
                self.thread.started.connect(self.worker.run)
                self.worker.finished.connect(self.thread.quit)
                #self.worker.finished.connect(self.worker.deleteLater)
                #self.thread.finished.connect(self.thread.deleteLater)
                # self.worker.progress.connect(self.reportProgress)
                self.thread.start()  # start the thread
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Starting camera thread')
            elif self.thread.isRunning():
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Interrupting existing running camera thread')
                self.worker.stop()

                # quit or exit?  neither seems to do anything  TODO: add signals so this works correctly
               # self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Restarting existing running camera thread')
                self.thread.quit()
                self.thread.exit()
                # ToDo: figure out why we can't restart here but we can on the next iteration
                #self.thread.start()
            else:  # thread died for some reason
                self.thread.start()
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Restarting existing stopped camera thread')

        else:  # no valid servers
            if self.thread is not None:
                self.worker.stop()
                self.thread.quit()
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Terminating camera thread: no valid camera servers')
            else:
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: No valid camera servers, unable to start thread')

        """        if self.camera_enabled:
            if self.qradiobutton_autoswitch.isChecked():  # auto determine which camera to show
                shooter_on = self.widget_dict['qlabel_shooter_indicator']['entry'].getBoolean(False)
                url = self.camera_dict['ShooterCam'] if shooter_on else self.camera_dict['BallCam']
            else:
                url = self.camera_dict[self.qcombobox_cameras.currentText()]  # figure out which url we want
            # stream = urllib.request.urlopen('http://10.24.29.12:1187/stream.mjpg')
            # get and display image
            cap = cv2.VideoCapture(url)
            if not cap.isOpened():
                print("Cannot open stream")
                return
            ret, frame = cap.read()
            pixmap = self.convert_cv_qt(frame, self.qlabel_camera_view)
            self.qlabel_camera_view.setPixmap(pixmap)
            self.qlabel_camera_view.repaint()"""

    def test(self):  # test function for checking new signals
        # print('Test was called', flush=True)
        current_server = self.servers[self.server_index]
        self.server_index = (self.server_index + 1 ) % len(self.servers)
        self.ntinst.setServer(server_name=self.servers[self.server_index])
        print(f'Changed server from {current_server} to {self.servers[self.server_index]}')


    def filter_nt_keys_combo(self):  # used to simplify the nt key list combo box entries
        if self.sorted_tree is not None:
            self.qcombobox_nt_keys.clear()
            filter = self.qt_text_entry_filter.toPlainText()
            filtered_keys = [key for key in self.sorted_tree if filter in key]
            self.qcombobox_nt_keys.addItems(filtered_keys)

    def update_key(self):  # used to modify an nt key value from the TUNING tab
        key = self.qcombobox_nt_keys.currentText()
        entry = self.ntinst.getEntry(key)
        entry_type = entry.getType()
        new_val = self.qt_text_new_value.toPlainText()
        print(f'Update key was called on {key}, which is a {entry_type}.  Setting it to {new_val}', flush=True)
        try:
            #t = QtWidgets.QPlainTextEdit()
            if entry_type == NetworkTableType.kDouble:
                new_val = float(new_val)
                entry.setDouble(new_val)
            elif entry_type == NetworkTableType.kString:
                entry.setString(new_val)
            elif entry_type == NetworkTableType.kBoolean:
                new_val = eval(new_val)
                entry.setBoolean(new_val)
            else:
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: {key} type {entry_type} not in [double, bool, string]')
        except Exception as e:
            self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Error occurred in setting {key} - {e}')
        self.qt_text_new_value.clear()
        self.refresh_tree()


    # update the autonomous routines - ToDo make this general for any chooser (pass the widget to currentTextChanged)
    def update_routines(self, text):
        key = self.widget_dict['qcombobox_autonomous_routines']['selected']
        self.ntinst.getEntry(key).setString(text)
        self.ntinst.flush()
        # print(f'Set NT value to {text}', flush=True)

    # tie commands to clicking labels - had to promote the labels to a class that has a click
    # Todo - make the commands dictionary and this use the same code - it's a bit redundant
    def label_click(self, label):
        # print(f"Running command to {label} {self.widget_dict[label]['command']}")
        toggled_state = not self.widget_dict[label]['command_entry'].getBoolean(True)
        print(f'You clicked {label} whose command is currently {not toggled_state}.  Firing command...', flush=True)
        self.widget_dict[label]['command_entry'].setBoolean(toggled_state)

    # ------------------- INITIALIZING WIDGETS --------------------------
    # set up appropriate entries for all the widgets we care about
    def initialize_widgets(self):
        self.widget_dict = {
        'qcombobox_autonomous_routines': {'widget':self.qcombobox_autonomous_routines, 'nt':r'/SmartDashboard/autonomous routines/options', 'command':None,
                                          'selected': r'/SmartDashboard/autonomous routines/selected'},
        'qlabel_arm_calibration_indicator': {'widget': self.qlabel_arm_calibration_indicator,
                                           'nt': '/SmartDashboard/ArmCalibration/running', 'command': '/SmartDashboard/ArmCalibration/running'},
        'qlabel_arm_retract_indicator': {'widget': self.qlabel_arm_retract_indicator, 'nt': '/SmartDashboard/ArmMoveDown/running',
                                               'command': '/SmartDashboard/ArmMoveDown/running'},
        'qlabel_arm_extend_indicator': {'widget': self.qlabel_arm_extend_indicator, 'nt': '/SmartDashboard/ArmMoveUp/running',
                                             'command': '/SmartDashboard/ArmMoveUp/running'},
        'qlabel_camera_view': {'widget':self.qlabel_camera_view, 'nt':None, 'command':None},
        'qlabel_compressor_indicator': {'widget':self.qlabel_compressor_indicator, 'nt':'/SmartDashboard/compressor_state', 'command': None},
        'qlabel_compressor_enabled_indicator': {'widget': self.qlabel_compressor_enabled_indicator, 'nt': '/SmartDashboard/compressor_close_loop',
                                                'command': '/SmartDashboard/CompressorToggle/running'},
        'qlabel_elevator_down_indicator': {'widget': self.qlabel_elevator_down_indicator,
                                              'nt': '/SmartDashboard/ElevatorMoveDown/running', 'command': '/SmartDashboard/ElevatorMoveDown/running'},
        'qlabel_elevator_up_indicator': {'widget': self.qlabel_elevator_up_indicator,
                                              'nt': '/SmartDashboard/ElevatorMoveUp/running', 'command': '/SmartDashboard/ElevatorMoveUp/running'},
        'qlabel_manipulator_closed_indicator': {'widget': self.qlabel_manipulator_closed_indicator, 'nt': '/SmartDashboard/manipulator_closed', 'command': '/SmartDashboard/ManipulatorToggle/running'},
        'qlabel_upper_pickup_indicator': {'widget':self.qlabel_upper_pickup_indicator, 'nt':'/SmartDashboard/UpperStationPickup/running', 'command': '/SmartDashboard/UpperStationPickup/running'},
        'qlabel_matchtime': {'widget': self.qlabel_matchtime, 'nt': '/SmartDashboard/match_time', 'command': None},
        'qlabel_nt_connected': {'widget': self.qlabel_nt_connected, 'nt': None, 'command': None},
        'qlabel_safe_carry_indicator': {'widget': self.qlabel_safe_carry_indicator, 'nt': '/SmartDashboard/SafeCarry/running', 'command': '/SmartDashboard/SafeCarry/running'},
        'qlabel_score_from_stow_indicator': {'widget': self.qlabel_score_from_stow_indicator, 'nt': '/SmartDashboard/ScoreFromStow/running', 'command': '/SmartDashboard/ScoreFromStow/running'},
        'qlabel_turret_calibration_indicator': {'widget': self.qlabel_turret_calibration_indicator, 'nt': '/SmartDashboard/TurretInitialize/running',
                                    'command': '/SmartDashboard/TurretInitialize/running'},
        'qlabel_turret_down_indicator': {'widget': self.qlabel_turret_down_indicator,
                                            'nt': '/SmartDashboard/TurretMoveDown/running',
                                            'command': '/SmartDashboard/TurretMoveDown/running'},
        'qlabel_turret_up_indicator': {'widget': self.qlabel_turret_up_indicator,
                                          'nt': '/SmartDashboard/TurretMoveUp/running',
                                          'command': '/SmartDashboard/TurretMoveUp/running'},
        'qlabel_wrist_calibration_indicator': {'widget': self.qlabel_wrist_calibration_indicator, 'nt': '/SmartDashboard/WristCalibration/running',
                                         'command': '/SmartDashboard/WristCalibration/running'},
        'qlabel_wrist_down_indicator': {'widget': self.qlabel_wrist_down_indicator, 'nt': '/SmartDashboard/WristMoveDown/running',
                                               'command': '/SmartDashboard/WristMoveDown/running'},
        'qlabel_wrist_up_indicator': {'widget': self.qlabel_wrist_up_indicator, 'nt': '/SmartDashboard/WristMoveUp/running',
                                             'command': '/SmartDashboard/WristMoveUp/running'},
        'qlcd_turret_angle': {'widget':self.qlcd_turret_angle, 'nt':'/SmartDashboard/turret_angle', 'command': None},
        'qlcd_elevator_height': {'widget': self.qlcd_elevator_height, 'nt': '/SmartDashboard/elevator_height', 'command': None},
        'qlcd_arm_extension': {'widget':self.qlcd_arm_extension, 'nt':'/SmartDashboard/arm_extension', 'command': None},
        'qlcd_wrist_angle': {'widget': self.qlcd_wrist_angle, 'nt': '/SmartDashboard/wrist_angle', 'command': None},
        'hub_targets': {'widget': None, 'nt': '/BallCam//green/targets', 'command': None},
        'hub_rotation': {'widget': None, 'nt': '/BallCam//green/rotation', 'command': None},
        'hub_distance': {'widget': None, 'nt': '/BallCam//green/distance', 'command': None},
        'drive_pose': {'widget': None, 'nt': '/SmartDashboard/drive_pose', 'command': None},
        }

        # get all the entries and add them to the dictionary
        for key, d in self.widget_dict.items():
            if d['nt'] is not None:
                d.update({'entry':self.ntinst.getEntry(d['nt'])})
            else:
                d.update({'entry': None})
            if d['command'] is not None:
                d.update({'command_entry': self.ntinst.getEntry(d['command'])})
                # assign a command clicked to it
                d['widget'].clicked.connect(lambda label=key: self.label_click(label))
            else:
                d.update({'command_entry': None})
            print(f'Widget {key}: {d}')

        for key, item in self.camera_dict.items():
            self.qcombobox_cameras.addItem(key)

    def update_widgets(self):
        """ Main function which is looped to update the GUI with NT values"""
        style_on = "border: 7px; border-radius: 7px; background-color:rgb(80, 235, 0); color:rgb(0, 0, 0);"
        style_off = "border: 7px; border-radius: 7px; background-color:rgb(220, 0, 0); color:rgb(200, 200, 200);"
        style_high = "border: 7px; border-radius: 15px; background-color:rgb(80, 235, 0); color:rgb(0, 0, 0);"
        style_low = "border: 7px; border-radius: 15px; background-color:rgb(0, 20, 255); color:rgb(255, 255, 255);"

        # update the connection indicator
        style = style_on if self.ntinst.isConnected() else style_off
        self.widget_dict['qlabel_nt_connected']['widget'].setStyleSheet(style)

        # update all labels tied to NT entries
        for key, d in self.widget_dict.items():
            if d['entry'] is not None:
                if 'indicator' in key:
                    #  print(f'Indicator: {key}')
                    style = style_on if d['entry'].getBoolean(False) else style_off
                    d['widget'].setStyleSheet(style)
                elif 'lcd' in key:
                    #  print(f'LCD: {key}')
                    value = int(d['entry'].getDouble(0))
                    d['widget'].display(str(value))
                elif 'combo' in key:  # ToDo: need a simpler way to update the combo boxes
                    new_list = d['entry'].getStringArray([])
                    if new_list != self.autonomous_list:
                        d['widget'].blockSignals(True)  # don't call updates on this one
                        d['widget'].clear()
                        d['widget'].addItems(new_list)
                        d['widget'].blockSignals(False)
                        self.autonomous_list = new_list
                    selected_routine = self.ntinst.getEntry(d['selected']).getString('')
                    if selected_routine != d['widget'].currentText():
                        d['widget'].blockSignals(True)  # don't call updates on this one
                        d['widget'].setCurrentText(selected_routine)
                        d['widget'].blockSignals(False)
                elif 'time' in key:

                    match_time = d['entry'].getDouble(0)
                    d['widget'].setText(str(int(match_time)))
                    if match_time < 30:
                        d['widget'].setText(f'* {int(match_time)} *')
                        d['widget'].setStyleSheet(style_low)
                    else:
                        d['widget'].setText(str(int(match_time)))
                        d['widget'].setStyleSheet(style_high)
                else:
                    pass
                    # print(f'Skipping: {key}')

        # update the commands list
        green = QtGui.QColor(227, 255, 227)
        white = QtGui.QColor(255, 255, 255)
        for ix, (key, d) in enumerate(self.command_dict.items()):
            bg_color = green if d['entry'].getBoolean(True) else white
            self.qlistwidget_commands.item(ix).setBackground(bg_color)

        # update the ball position on the hub target image
        hub_targets = self.widget_dict['hub_targets']['entry'].getDouble(0)
        hub_rotation = self.widget_dict['hub_rotation']['entry'].getDouble(0) - 5
        hub_distance = self.widget_dict['hub_distance']['entry'].getDouble(0)
        # print(f'hub_targets: {hub_targets} {hub_rotation:2.2f} {hub_distance:2.2f}', end='\r')

        if hub_targets > 0:
            # shooter_rpm = self.widget_dict['qlcd_shooter_rpm']['entry'].getDouble(0)
            shooter_rpm = 2000
            shooter_distance = shooter_rpm * 0.00075
            center_offset = shooter_distance * -np.sin(hub_rotation * 3.14159 / 180)
            x = 205 + center_offset * (380 / 1.2) # 380 px per 1.2 m
            y = 190

            self.qlabel_ball.move(int(x), int(y))
            if self.qlabel_ball.isHidden():
                self.qlabel_ball.show()
        else:
            #self.qlabel_ball.move(0, 0)
            if not self.qlabel_ball.isHidden():
                self.qlabel_ball.hide()

        # update the pose
        width, height = self.qgroupbox_field.width(), self.qgroupbox_field.height()
        bot_width, bot_height = self.qlabel_robot.width(), self.qlabel_robot.height()
        x_lim, y_lim = 16.4, 8.2
        drive_pose = self.widget_dict['drive_pose']['entry'].getDoubleArray([0,0,0])
        self.qlabel_robot.move(int(-bot_width/2 + width * drive_pose[0] / x_lim ), int(-bot_height/2 + height * (1 - drive_pose[1] / y_lim)))
        ## print(f'Pose X:{drive_pose[0]:2.2f} Pose Y:{drive_pose[1]:2.2f} Pose R:{drive_pose[2]:2.2f}', end='\r', flush=True)

        self.counter += 1
        if self.counter % 100 == 0:  # display an FPS
            current_time = time.time()
            msg = f'Current Updates/s : {100/(current_time - self.previous_time):.1f}'
            self.statusBar().showMessage(msg)
            self.previous_time = current_time

    def qt_tree_widget_nt_clicked(self, item):
        # send the clicked item from the tree to the filter for the nt selction combo box
        # print(f' Item clicked is: {item.data()}', flush=True)
        self.qt_text_entry_filter.clear()
        self.qt_text_entry_filter.setPlainText(item.data())

    def command_list_clicked(self, item):
        # shortcut where we click the command list, fire off (or end) the command
        cell_content = item.data()
        toggled_state = not self.command_dict[cell_content]['entry'].getBoolean(True)
        print(f'You clicked {cell_content} which is currently {not toggled_state}.  Firing command...', flush=True)
        self.command_dict[cell_content]['entry'].setBoolean(toggled_state)

    # -------------------  UPDATING NETWORK TABLES DISPLAY --------------------------
    def toggle_network_tables(self):
        # tree = QtWidgets.QTreeWidget
        if self.qt_tree_widget_nt.isHidden():
            self.refresh_tree()
            self.qt_tree_widget_nt.show()
        else:
            self.qt_tree_widget_nt.hide()

    def report_nt_status(self):
        id, ip = self.ntinst.getConnections()[0].remote_id, self.ntinst.getConnections()[0].remote_ip
        self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: NT status: id={id}, ip={ip}')

    def refresh_tree(self):
        """  Read networktables and update tree and combo widgets
        """
        self.connected = self.ntinst.isConnected()
        if self.connected:
            self.report_nt_status()
            self.qt_tree_widget_nt.clear()
            entries = self.ntinst.getEntries('/', types=0)
            self.sorted_tree = sorted([e.getName() for e in entries])

            # update the dropdown combo box with all keys
            self.filter_nt_keys_combo()
            # self.qcombobox_nt_keys.clear()
            # self.qcombobox_nt_keys.addItems(self.sorted_tree)

            # generate the dictionary - some magic I found on the internet
            nt_dict = {}
            levels = [s[1:].split('/') for s in self.sorted_tree]
            for path in levels:
                current_level = nt_dict
                for part in path:
                    if part not in current_level:
                        current_level[part] = {}
                    current_level = current_level[part]

            self.qlistwidget_commands.clear()
            for item in self.sorted_tree:
                # print(item)
                if 'running' in item:  # quick test of the list view for commands
                    # print(f'Command found: {item}')
                    command_name = item.split('/')[2]
                    self.qlistwidget_commands.addItem(command_name)
                    self.command_dict.update({command_name: {'nt':item, 'entry': self.ntinst.getEntry(item)}})

                entry_value = self.ntinst.getEntry(item).getValue()
                value = entry_value.value()
                age = int(time.time() - entry_value.last_change()/1E6)
                levels = item[1:].split('/')
                if len(levels) == 2:
                    nt_dict[levels[0]][levels[1]] = value, age
                elif len(levels) == 3:
                    nt_dict[levels[0]][levels[1]][levels[2]] = value, age
                elif len(levels) == 4:
                    nt_dict[levels[0]][levels[1]][levels[2]][levels[3]] = value, age

            self.fill_item(self.qt_tree_widget_nt.invisibleRootItem(), nt_dict)
            self.qt_tree_widget_nt.resizeColumnToContents(0)
            self.qt_tree_widget_nt.setColumnWidth(1, 100)
        else:
            self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Unable to connect to server')

    # -------------------  HELPER FUNCTIONS FOR THE DICTIONARIES AND WIDGETS --------------------------
    def eventFilter(self, obj, event):
        if (obj is self.qt_text_entry_filter or obj is self.qt_text_new_value) and event.type() ==  QEvent.KeyPress:
            if event.key() in (Qt.Key_Return, Qt.Key_Enter):
                return True
        return super().eventFilter(obj, event)

    def depth(self, d):
        if isinstance(d, dict):
            return 1 + (max(map(self.depth, d.values())) if d else 0)
        return 0

    ## helper functions for filling the NT tree widget
    def fill_item(self, widget, value):
        if value is None:
            # keep recursing until nothing is passed
            return
        elif isinstance(value, dict) and self.depth(value) > 1:
            for key, val in sorted(value.items()):
                self.new_item(parent=widget, text=str(key), val=val)
        elif isinstance(value, dict):
            # now we actually add the bottom level item
            #self.new_item(parent=widget, text=str(value))
            for key, val in sorted(value.items()):
                child = QtWidgets.QTreeWidgetItem([str(key), str(val[0]), str(val[1])])
                self.fill_item(child, val)
                widget.addChild(child)
        else:
            pass

    def new_item(self, parent, text, val=None):
        if val is None:
            child = QtWidgets.QTreeWidgetItem([text, 'noval'])
        else:
            if isinstance(val,dict):
                child = QtWidgets.QTreeWidgetItem([text])
            else:
                child = QtWidgets.QTreeWidgetItem([text, str(val[0]), str(val[1])])
        self.fill_item(child, val)
        parent.addChild(child)
        child.setExpanded(True)



# -------------------  MAIN --------------------------
if __name__ == "__main__":
    import sys

    # attempt to set high dpi scaling if it is detected - possible fix for high-def laptop and destop displays
    # if hasattr(QtCore.Qt, 'AA_EnableHighDpiScaling'):
    #     QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    if hasattr(QtCore.Qt, 'AA_UseHighDpiPixmaps'):
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

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
