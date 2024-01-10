import time
import traceback

from PyQt5.QtWidgets import QMainWindow, QApplication, QMessageBox
import PyQt5.QtCore as QtCore
import PyQt5.QtGui as QtGui
import sys
from datetime import datetime
import numpy as np

from base_interface.mission_management import MissionManager
from base_interface.control_modes import ControlModeState
from mission_control.mission_control import MissionControl
from base_interface.ui import Ui_MainWindow
from analysis.data_recorder import DataRecorder
from base_interface.battery_model import Battery
from motion_planning.projections import Projector, PointOfInterest, get_heading
from famsec import goa, rollout, et_goa
from base_interface.settings import Settings


class BaseInterface(QMainWindow, Ui_MainWindow):
    COND_TELEMENTRY = 'TELEM'
    COND_GOA = 'GOA'
    COND_ETGOA = 'ET-GOA'

    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)
        settings = Settings('./settings.yaml')
        settings.read()
        self.cohrint_logo_img_path = settings.logo_path
        self.mission_area_img_path = settings.map_path
        self.rollout_path = settings.rollout_path
        self.condition = settings.condition
        self.record_path = settings.record_path
        self.data_recorder = DataRecorder(self.condition, self.record_path.format(''))
        self.projector = Projector(settings.lat_center, settings.lon_center)
        self.projector.setup()
        self.mission_manager = MissionManager(self.mission_area_img_path, self.projector,
                                              settings.obstacles)

        #################
        # Setup the System Connections
        print('Setting up connections')
        self.robot_connected = False
        self.gps_connected = False
        self.sensor1_connected = False
        self.sensor2_connected = False
        self.ui_connected = False

        #################
        # Setup the robot state
        print('Setting up state')
        self.mission_mode = ControlModeState(ControlModeState.planning)  # planning or executing
        self.control_mode = ControlModeState(ControlModeState.stopped)  # stopped or driving
        self.mission_phase = ControlModeState(ControlModeState.phase_mission_planning)

        #################
        # Setup the Mission Planning Panel
        print('Setting up Mission Planning')
        self.poi_selected = None
        self.poi_accepted = None
        self.accept_poi_button.clicked.connect(self.accept_poi_callback)
        self.plan_poi_button.clicked.connect(self.plan_poi_callback)
        self.plan_poi_button.clicked.connect(self.start_competency_assessment)
        self.poi_selection.currentTextChanged.connect(self.select_poi_callback)

        #################
        # Setup the Telemetry Panel
        print('Setting up telemetry')
        self.battery_model = Battery()
        self.update_control_mode_state(self.control_mode.state)
        self.update_mission_mode_state(self.mission_mode.state)
        self.power_number = 50
        self.gps_frequency = 68
        self.velocity = 0.0
        self.heading = 0.0
        self.position = PointOfInterest()
        self.time_start = datetime.now()
        self.telemetry_updater = QtCore.QTimer()
        self.telemetry_updater.timeout.connect(self.periodic_update)
        self.telemetry_updater.setInterval(500)
        self.telemetry_updater.start()
        self.battery_updater = QtCore.QTimer()
        self.battery_updater.timeout.connect(lambda: self.battery_model.decrement(self.velocity))
        self.battery_updater.setInterval(1000)
        self.battery_updater.start()

        #################
        # Setup the Mission Control Panel
        print('Setting up Mission Control')
        self.mission_control = MissionControl(self.gps_frequency,
                                              self.power_number,
                                              self.battery_model.battery_level,
                                              100, 100, 5)
        self.request_mission_control_help_button.clicked.connect(
            self.request_mission_control_help_callback)
        self.request_mission_control_help_button.clicked.connect(
            lambda: self.splash_of_color(self.request_mission_control_help_button))

        #################
        # Setup the Robot Control panel
        print('Setting up robot control')
        self.drive_mode_button.clicked.connect(
            lambda: self.update_control_mode_state(ControlModeState.drive))
        self.drive_mode_button.setDisabled(True)
        self.stop_mode_button.clicked.connect(
            lambda: self.update_control_mode_state(ControlModeState.stopped))
        self.robot_power_slider.valueChanged.connect(self.update_robot_power_callback)
        self.robot_battery_slider.valueChanged.connect(self.update_robot_battery_callback)
        self.robot_gps_dial.valueChanged.connect(self.update_robot_gps_frequency_callback)
        self.robot_power_slider.setValue(self.power_number)
        self.robot_battery_slider.setValue(self.battery_model.get_number())
        self.robot_gps_dial.setValue(self.gps_frequency)

        #################
        # Setup the Competency Assessment Panel
        self.etgoa = et_goa.et_goa(min_stds=settings.et_goa_stds)
        if self.condition == self.COND_GOA or self.condition == self.COND_ETGOA:
            print('Setting up Competency Assessment')
            self.etgoa.set_pred_paths([self.rollout_path.format(i) for i in range(10)])
            self.rollout_thread = None
            self.et_goa_threshold = settings.et_goa_threshold
        else:
            self.competency_assessment_frame.hide()
        self.mqa = [0] * 3  # [x, y, v]
        self.goa = [0] * 5  # []

        #################
        # Setup the questionnaire prompts
        self.surveys = {0: False, 1: False, 2: False}
        self.survey_prompt(0)

    def periodic_update(self):
        """
        Updater for most periodic updates on the UI

        :return:
        """
        try:
            self.update_control_mode_text()
            self.update_mission_mode_text()
            self.update_time_text()
            self.update_battery_text()
            self.update_velocity_text()
            self.update_heading_text()
            self.update_position_text()
            self.update_connections()
            self.data_recorder.add_row(self.position.x, self.position.y, self.position.z,
                                       self.heading, self.velocity,
                                       self.control_mode, self.mission_mode,
                                       self.battery_model.get_number(),
                                       self.battery_model.get_level(),
                                       self.power_number, self.gps_frequency,
                                       'N/A', 'N/A',
                                       "_".join(["{:.2f}".format(x) for x in self.goa]),
                                       "_".join(["{:.2f}".format(x) for x in self.mqa]),
                                       (datetime.now() - self.time_start).total_seconds())
            self.update_map()
            self.update_et_goa()
        except Exception as e:
            traceback.print_exc()

    def navigation_complete(self):
        self.update_control_mode_state(ControlModeState.stopped)
        self.update_mission_mode_state(ControlModeState.planning)

    def update_map(self):
        """
        Updater for the map
        :return:
        """
        try:
            complete = self.mission_manager.update_progress(self.position.x, self.position.y)
            if complete:
                self.navigation_complete()
            img = self.mission_manager.get_overlay_image(self.position.x, self.position.y)
            height, width, channel = img.shape
            bytesPerLine = 3 * width
            qImg = QtGui.QImage(img, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.label_21.setPixmap(QtGui.QPixmap(qImg))
        except Exception as e:
            traceback.print_exc()

    def update_connections(self):
        """
        Updater for connection status

        :return:
        """
        try:
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
        except Exception as e:
            traceback.print_exc()

    def update_control_mode_text(self):
        """
        Updater for control mode

        :return:
        """
        try:
            text = '{}'.format(self.control_mode)
            self.state_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_mission_mode_text(self):
        """
        Updater for mission mode

        :return:
        """
        try:
            text = '{}'.format(self.mission_mode)
            self.mode_text.setText(text)
            if self.mission_mode.state == ControlModeState.planning:
                self.drive_mode_button.setDisabled(True)
            elif self.mission_mode.state == ControlModeState.execution:
                self.drive_mode_button.setEnabled(True)
        except Exception as e:
            traceback.print_exc()

    def update_phase_text(self):
        pass

    def update_position_text(self):
        """
        Updater for position

        :return:
        """
        try:
            self.latitude_label.setText('x: {:.10f}'.format(self.position.x))
            self.longitude_label.setText('y: {:.10f}'.format(self.position.y))
            self.altitude_label.setText('z:  {:.10f}'.format(self.position.z))
        except Exception as e:
            traceback.print_exc()

    def update_heading_text(self):
        """
        Updater for heading

        :return:
        """
        try:
            text = '{:03.0f} degrees | {}'.format(self.heading, get_heading(self.heading))
            self.heading_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_velocity_text(self):
        """
        Updater for velocity

        :return:
        """
        try:
            text = '{:.2f} m/s'.format(self.velocity)
            self.velocity_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_battery_text(self):
        """
        Updater for battery level

        :return:
        """
        try:
            text = '{:.2f}%'.format(self.battery_model.get_level())
            self.battery_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_time_text(self):
        """
        Updater for time

        :return:
        """
        try:
            mission_time = datetime.now() - self.time_start
            now = datetime.now()
            mins = mission_time.total_seconds() // 60
            secs = mission_time.total_seconds() % 60
            textm = "{}".format(str(int(mins)).rjust(2, "0"))
            texts = "{}".format(str(int(secs)).rjust(2, "0"))
            text = '{}:{} | {} MT'.format(textm, texts,  now.strftime("%H:%M:%S"))
            self.time_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_robot_power_callback(self):
        """
        Updater for robot power LCD

        :return:
        """
        try:
            val = self.robot_power_slider.value()
            self.robot_power_lcd.display(val)
            self.power_number = val
            self.check_anomaly_strategy()
        except Exception as e:
            traceback.print_exc()

    def update_robot_battery_callback(self):
        """
        Updater for robot battery LCD

        :return:
        """
        try:
            val = self.robot_battery_slider.value()
            self.robot_battery_lcd.display(val)
            self.battery_model.swap_battery(val)
            self.check_anomaly_strategy()
        except Exception as e:
            traceback.print_exc()

    def update_robot_gps_frequency_callback(self):
        """
        Updater for robot GPS LCD

        :return:
        """
        try:
            val = self.robot_gps_dial.value()
            self.robot_gps_lcd.display(val)
            self.gps_frequency = val
            self.check_anomaly_strategy()
        except Exception as e:
            traceback.print_exc()

    def check_anomaly_strategy(self):
        """
        Check that the strategy fixed the anomaly

        :return:
        """
        if self.mission_control:
            text = self.mission_control.check_strategy(self.power_number, self.gps_frequency,
                                                       self.battery_model.get_number())
            if text != "":
                self.update_mission_control_text(text, 'green')
                self.enable_buttons()
                # delete the old stuff here
                self.plan_poi_callback()
                self.start_competency_assessment()

    def update_control_mode_state(self, new_state):
        """
        Updater for control mode changes

        :param new_state:
        :return:
        """
        try:
            self.control_mode.state = new_state
            if self.control_mode.state == ControlModeState.stopped:
                self.stop_mode_button.setStyleSheet('background-color: green')
                self.drive_mode_button.setStyleSheet('background-color: light gray')
            elif self.control_mode.state == ControlModeState.drive:
                self.stop_mode_button.setStyleSheet('background-color: light gray')
                self.drive_mode_button.setStyleSheet('background-color: green')
            self.update_control_mode_text()
        except Exception as e:
            traceback.print_exc()

    def update_mission_mode_state(self, new_state):
        """
        Updater for mission mode changes

        :param new_state:
        :return:
        """
        try:
            self.mission_mode.state = new_state
            self.update_mission_mode_text()
        except Exception as e:
            traceback.print_exc()

    def select_poi_callback(self):
        """
        Callback for selecting a POI

        :return:
        """
        try:
            if self.poi_selection.currentText() != "Select POI":
                self.poi_selected = self.poi_selection.currentText()
                print('Selected POI: ', self.poi_selected)
                self.accept_poi_button.setStyleSheet('background-color: light grey')
                self.update_mission_control_text("")
        except Exception as e:
            traceback.print_exc()

    def plan_poi_callback(self):
        """
        Callback to generate a waypoint plan to the selected POI

        :return:
        """

        if self.poi_selected:
            print('Planning route to POI: ', self.poi_selected)
            self.splash_of_color(self.frame_2)
            self.mission_manager.plan_known_poi(self.position.x, self.position.y, self.poi_selected)

    def accept_poi_callback(self):
        """
        Callback to accept a POI

        :return:
        """
        try:
            if self.poi_selected:
                print('Accepted POI: ', self.poi_selected)
                self.poi_accepted = self.poi_selected
                self.accept_poi_button.setStyleSheet('background-color: green')
                self.survey_prompt(1)
                self.mission_phase.state = ControlModeState.phase_mission_execution
                self.mission_mode.state = ControlModeState.execution
        except Exception as e:
            traceback.print_exc()

    def update_mission_control_text(self, text, color=None):
        """
        Updater for new mission control text

        :param text:
        :param color:
        :return:
        """
        try:
            if color is not None:
                self.splash_of_color(self.mission_control_update_text, color=color, timeout=1000)
            self.mission_control_update_text.setPlainText(text)
        except Exception as e:
            traceback.print_exc()

    def request_mission_control_help_callback(self):
        """
        Callback when help is requested from Mission Control

        :return:
        """
        print('requesting help')
        self.accept_poi_button.setStyleSheet('background-color: light grey')
        text = self.mission_control.get_response(self.mission_control.BATTERY)
        self.update_mission_control_text(text, 'red')
        self.update_mission_mode_state(ControlModeState.planning)
        print('stopping waypoint follower')
        self.stop_mode_button.click()
        time.sleep(0.1)
        self.disable_buttons()

    def start_competency_assessment(self):
        """
        Callback to start a GOA competency assessment

        :return:
        """
        try:
            if self.condition == self.COND_ETGOA or self.condition == self.COND_GOA:
                if self.mission_manager.has_plan():
                    self.mission_mode.state = ControlModeState.assessing
                    self.splash_of_color(self.competency_assessment_frame, color='light grey',
                                         timeout=0)
                    self.disable_buttons()
                    labels = [self.label_6, self.label_7, self.label_8, self.label_15,
                              self.label_16]
                    for label in labels:
                        label.setStyleSheet('background-color: {}; color: black'.format('green'))
                        label.setText("{}".format('Computing...'))
                    plan = self.mission_manager.current_plan
                    self.splash_of_color(self.competency_assessment_frame, timeout=0)
                    self.rollout_thread = rollout.RolloutThread()
                    self.rollout_thread.pose = [self.position.x, self.position.y, self.position.z]
                    self.rollout_thread.orientation = [0, 0, np.deg2rad(self.heading), 0]
                    self.rollout_thread.waypoints = plan
                    self.rollout_thread.known_obstacles = {}
                    self.rollout_thread.goal = plan[-1]
                    print('driving to', plan[-1])
                    self.rollout_thread.finished.connect(self.finish_competency_assessment)
                    self.rollout_thread.start()
        except Exception as e:
            traceback.print_exc()

    def finish_competency_assessment(self, goa_ret):
        """
        Callback when a GOA competency assessment is completed

        :param goa_ret:
        :return:
        """
        try:
            labels = [self.label_6, self.label_7, self.label_8, self.label_15, self.label_16]
            goas = []
            for gg, label in zip(goa_ret.items(), labels):
                outcome = gg[1]
                goas.append(outcome)
                label.setStyleSheet(
                    'background-color: {}; color: black'.format(goa.semantic_label_color(outcome)))
                label.setText("{}".format(goa.semantic_label_text(outcome)))
            self.goa = goas
            if self.condition == self.COND_ETGOA:
                self.etgoa.preprocess()
            self.enable_buttons()
            if self.mission_phase.state == ControlModeState.phase_mission_planning:
                self.mission_mode.state = ControlModeState.planning
            elif self.mission_phase.state == ControlModeState.phase_mission_execution:
                self.mission_mode.state = ControlModeState.execution
                self.accept_poi_button.click()
            self.splash_of_color(self.competency_assessment_frame, color='light grey', timeout=0)
        except Exception as e:
            traceback.print_exc()

    def update_et_goa(self):
        """
        Updater for the ET-GOA algorithm

        :return:
        """
        try:
            if (self.control_mode.state == ControlModeState.drive
                    and self.etgoa.has_data() and self.condition == self.COND_ETGOA):
                t = (datetime.now() - self.time_start).total_seconds()
                self.etgoa.set_start_time(t)
                px, py, pv = self.position.x, self.position.y, self.velocity
                si = self.etgoa.get_si(px, py, pv, t)
                self.mqa = si
                if np.min(self.mqa) <= self.et_goa_threshold:
                    print('Should trigger a stop + reassessment')
                    # mean speed
                    # mean battery drain
                    # newly observed obstacles
            # TODO need an in-mission assmt state so we don't hit the below case when ET-GOA triggers
            if self.control_mode.state == ControlModeState.stopped and self.condition == self.COND_ETGOA:
                self.etgoa.forget_start_time()
                self.mqa = [0] * 3
        except Exception as e:
            traceback.print_exc()

    def disable_buttons(self):
        """
        Disable control and communications buttons during processing

        :return:
        """
        try:
            self.drive_mode_button.setDisabled(True)
            self.poi_selection.setDisabled(True)
            self.plan_poi_button.setDisabled(True)
            #self.stop_mode_button.setDisabled(True)
            self.accept_poi_button.setDisabled(True)
            self.request_mission_control_help_button.setDisabled(True)
            self.mission_control_update_text.setDisabled(True)
        except Exception as e:
            traceback.print_exc()

    def enable_buttons(self):
        """
        Enable control and communications buttons during processing

        :return:
        """
        try:
            if self.poi_accepted:
                self.drive_mode_button.setEnabled(True)
                self.stop_mode_button.setEnabled(True)

            self.poi_selection.setEnabled(True)
            self.plan_poi_button.setEnabled(True)
            self.accept_poi_button.setEnabled(True)
            self.request_mission_control_help_button.setEnabled(True)
            self.mission_control_update_text.setEnabled(True)
        except Exception as e:
            traceback.print_exc()

    def survey_prompt(self, survey):
        if not self.surveys[survey]:
            self.surveys[survey] = True
            msg = QMessageBox()
            msg.setWindowTitle("Questionnaire Request")
            msg.setText("Please complete questionnaire {} and press OK when done".format(survey))
            x = msg.exec_()

    def splash_of_color(self, obj, color='green', timeout=500):
        """
        Change the color of a panel

        :param obj:
        :param color:
        :param timeout:
        :return:
        """
        try:
            style_sheet = obj.styleSheet()
            obj.setStyleSheet('background-color: {}'.format(color))
            if timeout > 0:
                QtCore.QTimer.singleShot(timeout, lambda: obj.setStyleSheet(style_sheet))
        except Exception as e:
            traceback.print_exc()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    MainWindow = BaseInterface()
    MainWindow.show()
    app.exec_()
