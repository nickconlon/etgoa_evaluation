from PyQt5.QtWidgets import QMainWindow, QApplication
import PyQt5.QtCore as QtCore
import PyQt5.QtGui as QtGui
import sys
from datetime import datetime

from base_interface.mission_management import MissionManager
from base_interface.control_modes import ControlModeState
from mission_control.mission_control import MissionControl
from base_interface.ui import Ui_MainWindow
from analysis.data_recorder import DataRecorder
from base_interface.battery_model import Battery


class BaseInterface(QMainWindow, Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)
        self.img_path = 'base_interface/mission_area.png'
        self.mission_control = MissionControl()
        self.data_recorder = DataRecorder()
        self.battery_model = Battery()
        self.mission_manager = MissionManager(self.img_path)

        #################
        # Setup the System Connections
        self.robot_connected = False
        self.gps_connected = False
        self.sensor1_connected = False
        self.sensor2_connected = False
        self.ui_connected = False

        #################
        # Setup the robot state
        self.mission_mode = ControlModeState(ControlModeState.planning)  # planning or executing
        self.control_mode = ControlModeState(ControlModeState.stopped)  # stopped or driving

        #################
        # Setup the Mission Control Communications panel
        self.send_mission_control_udpate_button.clicked.connect(self.send_mission_control_update_callback)
        self.send_mission_control_udpate_button.clicked.connect(
            lambda: self.splash_of_color(self.send_mission_control_udpate_button))
        self.request_mission_control_help_button.clicked.connect(self.request_mission_control_help_callback)
        self.request_mission_control_help_button.clicked.connect(
            lambda: self.splash_of_color(self.request_mission_control_help_button))

        #################
        # Setup the Mission Planning Panel
        self.poi_selected = None
        self.accept_poi_order_button.clicked.connect(lambda: self.splash_of_color(self.accept_poi_order_button))
        self.accept_poi_order_button.clicked.connect(self.accept_poi_callback)
        self.poi_selection.currentTextChanged.connect(self.select_poi_callback)

        #################
        # Setup the Robot Control panel
        self.drive_mode_button.clicked.connect(lambda: self.update_control_mode_state(ControlModeState.drive))
        self.stop_mode_button.clicked.connect(lambda: self.update_control_mode_state(ControlModeState.stopped))

        self.robot_power_slider.valueChanged.connect(self.update_robot_power_callback)
        self.robot_battery_slider.valueChanged.connect(self.update_robot_battery_callback)
        self.robot_gps_dial.valueChanged.connect(self.update_robot_gps_frequency_callback)

        #################
        # Setup the Telemetry Panel
        self.update_control_mode_state(self.control_mode.state)
        self.update_mission_mode_state(self.mission_mode.state)
        self.battery_remaining = 100
        self.battery_number = 0
        self.power_number = 0
        self.gps_frequency = 0
        self.velocity = 0.0
        self.heading = 0.0
        self.position = [0.0, 0.0, 0.0]
        self.time_start = datetime.now()

        self.telemetry_updater = QtCore.QTimer()
        self.telemetry_updater.timeout.connect(self.periodic_update)
        self.telemetry_updater.setInterval(500)
        self.telemetry_updater.start()

    def periodic_update(self):
        self.update_control_loa_mode_text()
        self.update_mission_mode_text()
        self.update_time_text()
        self.update_battery_text()
        self.update_velocity_text()
        self.update_heading_text()
        self.update_position_text()
        self.update_connections()
        self.data_recorder.add_row(*self.position, self.heading, self.velocity,
                                   self.control_mode, self.mission_mode,
                                   self.battery_number, self.battery_remaining,
                                   self.power_number, self.gps_frequency, -1, -1,
                                   datetime.now() - self.time_start)
        self.update_map()

    def update_map(self):
        img = self.mission_manager.get_overlay_image(
            *self.mission_manager.projector.equirectangular_projection(self.position[0], self.position[1]))
        self.update_map_display(img)

    def update_connections(self):
        self.robot_connected_indicator.setStyleSheet(
            'background-color: {}'.format('green' if self.robot_connected else 'red'))
        self.gps_connected_indicator.setStyleSheet(
            'background-color: {}'.format('green' if self.gps_connected else 'red'))
        self.sensor1_connected_indicator.setStyleSheet(
            'background-color: {}'.format('green' if self.sensor1_connected else 'red'))
        self.sensor2_connected_indicator.setStyleSheet(
            'background-color: {}'.format('green' if self.sensor2_connected else 'red'))
        self.ui_connected_indicator.setStyleSheet(
            'background-color: {}'.format('green' if self.ui_connected else 'red'))

    def update_control_loa_mode_text(self):
        text = '{}'.format(self.control_mode)
        self.state_text.setText(text)

    def update_mission_mode_text(self):
        text = '{}'.format(self.mission_mode)
        self.mode_text.setText(text)

    def update_LOA_mode_text(self):
        text = '{}'.format(self.mission_mode)
        self.mode_text.setText(text)

    def update_position_text(self):
        self.latitude_label.setText('Lat: {:.10f}'.format(self.position[0]))
        self.longitude_label.setText('Lon: {:.10f}'.format(self.position[1]))
        self.altitude_label.setText('Alt:  {:.10f}'.format(self.position[2]))

    def update_heading_text(self):
        heading = self.heading
        if 0 <= heading < 22.5:
            desc = 'North'
        elif 22.5 <= heading < 67.5:
            desc = 'North East'
        elif 67.5 <= heading < 112.5:
            desc = 'East'
        elif 112.5 <= heading < 157.5:
            desc = 'South East'
        elif 157.5 <= heading < 202.5:
            desc = 'South'
        elif 202.5 <= heading < 247.5:
            desc = 'South West'
        elif 247.5 <= heading < 292.5:
            desc = 'West'
        elif 292.5 <= heading < 337.5:
            desc = 'North West'
        elif 337.5 <= heading < 360:
            desc = 'North'
        else:
            desc = 'Error'
        text = '{:03.0f} degrees | {}'.format(heading, desc)
        self.heading_text.setText(text)

    def update_velocity_text(self):
        text = '{:.2f} m/s'.format(self.velocity)
        self.velocity_text.setText(text)

    def update_battery_text(self):
        text = '{}%'.format(self.battery_remaining)
        self.battery_text.setText(text)

    def update_time_text(self):
        mission_time = datetime.now() - self.time_start
        now = datetime.now()
        mins = mission_time.total_seconds() // 60
        secs = mission_time.total_seconds() % 60
        textm = "{}".format(str(int(mins)).rjust(2, "0"))
        texts = "{}".format(str(int(secs)).rjust(2, "0"))
        text = '{}:{} into {} | {} MT'.format(textm, texts, self.mission_mode, now.strftime("%H:%M:%S"))
        self.time_text.setText(text)

    def update_robot_power_callback(self):
        val = self.robot_power_slider.value()
        self.robot_power_lcd.display(val)

    def update_robot_battery_callback(self):
        val = self.robot_battery_slider.value()
        self.robot_battery_lcd.display(val)

    def update_robot_gps_frequency_callback(self):
        val = self.robot_gps_dial.value()
        self.robot_gps_lcd.display(val)

    def update_control_mode_state(self, new_state):
        self.control_mode.state = new_state
        if self.control_mode.state == ControlModeState.stopped:
            self.stop_mode_button.setStyleSheet('background-color: green')
            self.drive_mode_button.setStyleSheet('background-color: light gray')
        elif self.control_mode.state == ControlModeState.drive:
            self.stop_mode_button.setStyleSheet('background-color: light gray')
            self.drive_mode_button.setStyleSheet('background-color: green')
        self.update_control_loa_mode_text()

    def update_mission_mode_state(self, new_state):
        self.mission_mode.state = new_state
        self.update_mission_mode_text()

    def select_poi_callback(self):
        self.poi_selected = self.poi_selection.currentText()
        self.mission_manager.plan_known_poi(
            *self.mission_manager.projector.equirectangular_projection(
                self.position[0], self.position[1]),
            self.poi_selected)
        # get POI position
        # get robot position
        # start splash of POI box
        # thread -> self.run_competency_assessment()
        # end splash of POI box
        # start splash of competency box
        # thread -> self.run_competency_assessment()
        # end splash of competency box

    def accept_poi_callback(self):
        self.mission_mode.state = ControlModeState.execution

    def update_mission_control_text(self, text):
        self.splash_of_color(self.mission_control_update_text, color='red', timeout=1000)
        self.mission_control_update_text.setPlainText(text)

    def send_mission_control_update_callback(self):
        text = self.mission_control_update_text.toPlainText()
        self.mission_control_update_text.clear()
        print(text)

    def request_mission_control_help_callback(self):
        print('asking for some help.')

    def run_competency_assessment(self):
        print('running competency assessment')

    def update_competency_assessment(self):
        self.splash_of_color(self.competency_assessment_frame)

    def update_map_display(self, img):
        height, width, channel = img.shape
        bytesPerLine = 3 * width
        qImg = QtGui.QImage(img, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
        self.label_21.setPixmap(QtGui.QPixmap(qImg))

    def splash_of_color(self, obj, color='green', timeout=500):
        style_sheet = obj.styleSheet()
        obj.setStyleSheet('background-color: {}'.format(color))
        if timeout > 0:
            QtCore.QTimer.singleShot(timeout, lambda: obj.setStyleSheet(style_sheet))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    MainWindow = BaseInterface()
    MainWindow.show()
    app.exec_()
