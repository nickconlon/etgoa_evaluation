import base64
import numpy as np
import traceback
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtGui import QPixmap, QImage
import PyQt5.QtCore as QtCore
import sys
from datetime import datetime
from .test_ui import Ui_MainWindow


class MissionControl:
    OKAY = 'okay'
    GPS = 'gps'
    BATTERY = 'battery'
    OBSTACLE = 'obstacle'
    EVERYTHING = 'everything'

    def __init__(self):
        self.help_requests = {"": None,
                              "I need help diagnosing everything": self.EVERYTHING,
                              "I need help diagnosing robot position issues": self.GPS,
                              "I need help diagnosing robot battery issues": self.BATTERY,
                              "I need help with an obstacle blocking the robot": self.OBSTACLE}

        self.help_responses = {
            self.OKAY: '{} looks fine to us',
            self.GPS: 'Please follow this procedure:\n\nInstruct the robot to stop. Next, set the GPS to frequency 34. Then instruct the robot continue.',
            self.BATTERY: 'Please follow this procedure:\n\nInstruct the robot to stop. Next, switch to backup battery 3. Then set power to 75. Then instruct the robot continue.',
            self.OBSTACLE: 'Please follow this procedure:\n\nInstruct the robot to stop. Next, switch to manual control and drive the robot around the obstacle and to the next waypoint. Then return control to the robot'}

        self.current_issue = self.BATTERY

    def get_response(self, issue):
        if self.current_issue == issue:
            return self.help_responses[issue]
        else:
            return self.help_responses[self.OKAY].format(issue)


class ControlModeState:
    drive = 0
    stopped = 1
    planning = 2
    execution = 3
    assessing = 4
    automatic = 5
    manual = 6

    descriptions = ['Driving', 'Stopped', 'Planning', 'Execution', 'Assessing', 'Automatic', 'Manual']

    def __init__(self, mode):
        self.state = mode

    def __str__(self):
        return self.descriptions[self.state]


class BaseInterface(QMainWindow, Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)
        self.mission_control = MissionControl()

        #################
        # Setup the System Connection panel
        self.robot_connected_indicator.setStyleSheet('background-color: red')
        self.gps_connected_indicator.setStyleSheet('background-color: red')
        self.sensor1_connected_indicator.setStyleSheet('background-color: red')
        self.sensor2_connected_indicator.setStyleSheet('background-color: red')
        self.ui_connected_indicator.setStyleSheet('background-color: red')

        #################
        # Setup the robot state
        self.mission_mode = ControlModeState(ControlModeState.planning)  # planning or executing
        self.control_mode = ControlModeState(ControlModeState.stopped)  # stopped or driving
        self.loa_mode = ControlModeState(ControlModeState.manual)  # manual or autonomy

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
        self.POIs_selected = None
        self.select_poi_order_button.clicked.connect(self.poi_select_callback)
        self.select_poi_order_button.clicked.connect(lambda: self.splash_of_color(self.select_poi_order_button))
        self.accept_poi_order_button.clicked.connect(self.accept_poi_callback)
        self.accept_poi_order_button.clicked.connect(lambda: self.splash_of_color(self.accept_poi_order_button))
        self.proceed_to_execution_button.clicked.connect(self.proceed_to_execution_callback)
        self.proceed_to_execution_button.clicked.connect(lambda: self.splash_of_color(self.proceed_to_execution_button))

        #################
        # Setup the Robot Control panel
        self.teleop_forward_button.clicked.connect(self.teleop_forward_callback)
        self.teleop_forward_button.clicked.connect(lambda: self.splash_of_color(self.teleop_forward_button))
        self.teleop_back_button.clicked.connect(self.teleop_back_callback)
        self.teleop_back_button.clicked.connect(lambda: self.splash_of_color(self.teleop_back_button))
        self.teleop_left_button.clicked.connect(self.teleop_left_callback)
        self.teleop_left_button.clicked.connect(lambda: self.splash_of_color(self.teleop_left_button))
        self.teleop_right_button.clicked.connect(self.teleop_right_callback)
        self.teleop_right_button.clicked.connect(lambda: self.splash_of_color(self.teleop_right_button))
        self.drive_mode_button.clicked.connect(lambda: self.update_control_mode_state(ControlModeState.drive))
        self.stop_mode_button.clicked.connect(lambda: self.update_control_mode_state(ControlModeState.stopped))
        self.manual_mode_button.clicked.connect(lambda: self.update_loa_mode_state(ControlModeState.manual))
        self.automatic_mode_button.clicked.connect(lambda: self.update_loa_mode_state(ControlModeState.automatic))

        self.robot_power_slider.valueChanged.connect(self.update_robot_power_callback)
        self.robot_battery_slider.valueChanged.connect(self.update_robot_battery_callback)
        self.robot_gps_dial.valueChanged.connect(self.update_robot_gps_frequency_callback)

        #################
        # Setup the Telemetry Panel
        self.update_control_mode_state(self.control_mode.state)
        self.update_mission_mode_state(self.mission_mode.state)
        self.update_loa_mode_state(self.loa_mode.state)
        self.battery_remaining = 100
        self.velocity = 0.0
        self.heading = 0.0
        self.position = [0.0, 0.0, 0.0]
        self.time_start = datetime.now()

        self.telemetry_updater = QtCore.QTimer()
        self.telemetry_updater.timeout.connect(self.update_telemetry)
        self.telemetry_updater.setInterval(500)
        self.telemetry_updater.start()

    def update_telemetry(self):
        self.update_control_loa_mode_text()
        self.update_mission_mode_text()
        self.update_time_text()
        self.update_battery_text()
        self.update_velocity_text()
        self.update_heading_text()
        self.update_position_text()

    def update_control_loa_mode_text(self):
        loa_mode = ''
        if self.control_mode.state != ControlModeState.stopped:
            loa_mode = self.loa_mode
        text = '{} {}'.format(self.control_mode, loa_mode)
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
        secs = mission_time.total_seconds()
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

    def update_loa_mode_state(self, new_state):
        self.loa_mode.state = new_state
        if self.loa_mode.state == ControlModeState.manual:
            self.manual_mode_button.setStyleSheet('background-color: green')
            self.automatic_mode_button.setStyleSheet('background-color: light gray')
        elif self.loa_mode.state == ControlModeState.automatic:
            self.manual_mode_button.setStyleSheet('background-color: light gray')
            self.automatic_mode_button.setStyleSheet('background-color: green')

    def update_mission_mode_state(self, new_state):
        self.mission_mode.state = new_state
        self.update_mission_mode_text()

    def poi_select_callback(self):
        self.POIs_selected = self.poi_selection.currentText()
        print(self.POIs_selected)
        self.run_competency_assessment()

    def accept_poi_callback(self):
        text = 'POIs selected:\n    {}'.format(self.POIs_selected)
        self.accept_poi_order_text.setText(text)
        print(text)

    def proceed_to_execution_callback(self):
        self.mission_mode.state = ControlModeState.execution

    def update_mission_control_text(self, text):
        self.splash_of_color(self.mission_control_update_prompt, color='red', timeout=1000)
        self.mission_control_update_prompt.setText(text)

    def send_mission_control_update_callback(self):
        text = self.mission_control_update_text.toPlainText()
        self.mission_control_update_text.clear()
        print(text)

    def request_mission_control_help_callback(self):
        print('asking for some help.')

    def teleop_forward_callback(self):
        print('teleoperation forward')

    def teleop_back_callback(self):
        print('teleoperation backward')

    def teleop_left_callback(self):
        print('teleoperation left')

    def teleop_right_callback(self):
        print('teleoperation right')

    def run_competency_assessment(self):
        print('running competency assessment')

    def update_competency_assessment(self):
        self.splash_of_color(self.competency_assessment_frame)

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
