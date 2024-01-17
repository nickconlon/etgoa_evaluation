import time
import traceback
from collections import deque

from PyQt5.QtWidgets import QMainWindow, QApplication
import PyQt5.QtCore as QtCore
import PyQt5.QtGui as QtGui
import sys
from datetime import datetime
import numpy as np
import os

from motion_planning.mission_management import MissionManager
from base_interface.control_modes import ControlModeState
from mission_control.mission_control import MissionControl
from base_interface.ui import Ui_MainWindow
from analysis.data_recorder import PrimaryTaskRecorder, SurveyRecorder
from motion_planning.projections import Projector, PointOfInterest, get_heading
from famsec import goa, rollout, et_goa
from base_interface.settings import Settings
from base_interface.survey_popup import run_survey_popup_online, run_survey_popup_offline


class BaseInterface(QMainWindow, Ui_MainWindow):
    COND_TELEMENTRY = 'TELEM'
    COND_GOA = 'GOA'
    COND_ETGOA = 'ET-GOA'

    def __init__(self, settings_path):
        QMainWindow.__init__(self)
        self.setupUi(self)
        settings = Settings(settings_path)
        settings.read()
        self.cohrint_logo_img_path = settings.logo_path
        self.mission_area_img_path = settings.map_path
        self.rollout_path = settings.rollout_path
        self.condition = settings.condition
        fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_primary_{}.csv'.format(
            settings.condition)
        self.data_recorder = PrimaryTaskRecorder(self.condition,
                                                 os.path.join(settings.record_path, fname))
        fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_trust_survey_{}.csv'.format(
            settings.condition)
        self.survey_recorder = SurveyRecorder(os.path.join(settings.record_path, fname))
        self.projector = Projector(settings.lat_center, settings.lon_center)
        self.projector.setup()
        self.mission_manager = MissionManager(self.mission_area_img_path, self.projector,
                                              settings.obstructions,
                                              settings.hazards,
                                              settings.power_draws)
        self.obstructions = settings.obstructions
        self.hazards = settings.hazards
        self.power_draws = settings.power_draws
        self.batt_drain_rate = settings.batt_drain_rate

        self.mission_time = 0.0
        self.update_rate = 0.5  # seconds
        self.poi_accepted_workaround = False

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
        self.battery_level = 100
        self.battery_number = 0
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
        self.telemetry_updater.setInterval(int(self.update_rate * 1000))
        self.telemetry_updater.start()

        self.mean_velocity = deque(maxlen=10)
        self.mean_battery = deque(maxlen=10)

        #################
        # Setup the Mission Control Panel
        print('Setting up Mission Control')
        self.mission_control = MissionControl(self.gps_frequency,
                                              self.power_number,
                                              self.battery_level,
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
        self.robot_battery_slider.setValue(self.battery_number)
        self.robot_gps_dial.setValue(self.gps_frequency)
        self.robot_battery_slider.setDisabled(True)
        self.robot_gps_dial.setDisabled(True)
        self.robot_power_slider.setDisabled(True)

        #################
        # Setup the Competency Assessment Panel
        self.etgoa = et_goa.et_goa(min_stds=settings.et_goa_stds)
        if self.condition == self.COND_GOA or self.condition == self.COND_ETGOA:
            print('Setting up Competency Assessment')
            self.etgoa.set_pred_paths([self.rollout_path.format(i) for i in range(10)])
            self.rollout_thread = None
            self.et_goa_threshold = settings.et_goa_threshold
        else:
            labels = [self.label_27, self.label_6, self.label_7, self.label_8, self.label_15,
                      self.label_16]
            [label.hide() for label in labels]
        self.mqa = [0] * len(settings.et_goa_stds)
        self.goa = [0] * 5  # []

        #################
        # Setup the questionnaire prompts
        self.surveys = {1: False, 2: False, 3: False} if settings.show_surveys else {1: True,
                                                                                     2: True,
                                                                                     3: True}
        self.survey_prompt(1)

    def periodic_update(self):
        """
        Updater for most periodic updates on the UI

        :return:
        """
        try:
            self.activate_buttons()
            self.update_mission_time()
            self.update_battery_level()
            self.update_connections()

            self.update_control_mode_text()
            self.update_mission_mode_text()
            self.update_time_text()
            self.update_battery_text()
            self.update_velocity_text()
            self.update_heading_text()
            self.update_position_text()

            self.data_recorder.add_row(self.position.x, self.position.y, self.position.z,
                                       self.heading, self.velocity,
                                       self.control_mode, self.mission_mode,
                                       self.battery_number, self.battery_level,
                                       self.power_number, self.gps_frequency,
                                       'N/A', 'N/A',
                                       "_".join(["{:.2f}".format(x) for x in self.goa]),
                                       "_".join(["{:.2f}".format(x) for x in self.mqa]),
                                       self.mission_time)
            self.update_map()
            self.update_et_goa()
        except Exception as e:
            traceback.print_exc()

    def navigation_complete(self):
        self.update_control_mode_state(ControlModeState.stopped)
        self.update_mission_mode_state(ControlModeState.planning)
        if self.mission_phase.state == ControlModeState.phase_mission_execution:
            self.mission_phase.state = ControlModeState.phase_mission_complete
            self.survey_prompt(3)

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

    def update_battery_level(self):
        """
        Updater for the battery level -  default is to lose 0.25 % at each 1/2 second timestep

        :return:
        """
        try:
            if self.control_mode.state == ControlModeState.drive:
                dt = self.update_rate
                drain_rate = self.batt_drain_rate
                for pd in self.power_draws:
                    # Change the battery draw rate %/second based
                    px, py = self.position.x, self.position.y
                    rx, ry = pd.center
                    x_err, y_err = rx - px, ry - py
                    dh = np.sqrt((x_err ** 2) + (y_err ** 2))
                    if dh <= pd.axis[0]:
                        drain_rate = pd.data
                prev_battery = self.battery_level
                new_battery = float(np.maximum(self.battery_level - dt * drain_rate, 0.0))
                self.mean_battery.append(abs(prev_battery - new_battery) / dt)
                self.battery_level = new_battery
        except Exception as e:
            traceback.print_exc()

    def update_battery_text(self):
        """
        Updater for battery level

        :return:
        """
        try:
            text = '{:.2f}%'.format(self.battery_level)
            self.battery_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_mission_time(self):
        try:
            self.mission_time = (datetime.now() - self.time_start).total_seconds()
        except Exception as e:
            traceback.print_exc()

    def update_time_text(self):
        """
        Updater for time

        :return:
        """
        try:
            now = datetime.now()
            mins = self.mission_time // 60
            secs = self.mission_time % 60
            textm = "{}".format(str(int(mins)).rjust(2, "0"))
            texts = "{}".format(str(int(secs)).rjust(2, "0"))
            text = '{}:{} | {} MT'.format(textm, texts, now.strftime("%H:%M:%S"))
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
            self.battery_number = int(val)
            self.battery_level = 100
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
                                                       self.battery_number)
            if text != "":
                self.robot_battery_slider.setDisabled(True)
                self.robot_gps_dial.setDisabled(True)
                self.robot_power_slider.setDisabled(True)
                self.update_mission_control_text(text, 'green')
                # delete the old stuff here
                # TODO add distance function to Obstacle class -> Obstacle.dist(rx, ry) -> float
                # TODO each obstacle name is unique int id
                # TODO add is_visible field to Obstacle class -> map only shows visible obstacles
                # publish list[int] of active obstacles -> mission manager keep track of this
                to_remove = []
                for o in self.mission_manager.power_draws:
                    pass
                for o in self.mission_manager.hazards:
                    pass
                for o in self.mission_manager.obstructions:
                    pass
                # remove obstacle from mission manager
                # send cancel request to waypoint planner
                if self.condition == self.COND_ETGOA:
                    self.start_competency_assessment()
                else:
                    self.finish_competency_assessment(None)

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
            self.mission_manager.plan_known_poi(self.position.x, self.position.y, self.poi_selected,
                                                tofrom=True)

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
                self.survey_prompt(2)
                if self.mission_phase.state == ControlModeState.phase_mission_planning:
                    self.time_start = datetime.now()  # reset time for execution phase
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
        self.robot_battery_slider.setEnabled(True)
        self.robot_gps_dial.setEnabled(True)
        self.robot_power_slider.setEnabled(True)
        self.accept_poi_button.setStyleSheet('background-color: light grey')
        text = self.mission_control.get_response(self.mission_control.BATTERY)
        self.update_mission_control_text(text, 'red')
        self.update_mission_mode_state(ControlModeState.planning)
        print('stopping waypoint follower')
        self.stop_mode_button.click()
        time.sleep(0.1)

    def start_competency_assessment(self):
        """
        Callback to start a GOA competency assessment

        :return:
        """
        try:
            if self.condition == self.COND_ETGOA or self.condition == self.COND_GOA:
                if self.mission_manager.has_plan():
                    # Maybe add back the current (x,y) location to the beginning of the plan?
                    self.mission_mode.state = ControlModeState.assessing
                    self.splash_of_color(self.competency_assessment_frame, color='light grey',
                                         timeout=0)
                    labels = [self.label_6, self.label_7, self.label_8, self.label_15,
                              self.label_16]
                    for label in labels:
                        label.setStyleSheet('background-color: {}; color: black'.format('green'))
                        label.setText("{}".format('Computing...'))
                    plan = self.mission_manager.get_plan()
                    self.splash_of_color(self.competency_assessment_frame, timeout=0)
                    self.rollout_thread = rollout.RolloutThread()
                    self.rollout_thread.pose = [self.position.x, self.position.y, self.position.z]
                    self.rollout_thread.orientation = [0, 0, np.deg2rad(self.heading), 0]
                    self.rollout_thread.waypoints = plan
                    self.rollout_thread.known_obstacles = {}
                    self.rollout_thread.goal = plan[-1]
                    self.rollout_thread.battery = self.battery_level
                    if self.mission_phase.state == ControlModeState.phase_mission_planning:
                        # During mission planning, parameter values are at their baseline
                        self.rollout_thread.velocity_rate = 1.0
                        self.rollout_thread.battery_rate = self.batt_drain_rate
                        self.rollout_thread.time_offset = 0.0
                    else:
                        # During mission execution, parameter values may have changed
                        self.rollout_thread.velocity_rate = float(
                            np.mean(self.mean_velocity) / 0.25)
                        self.rollout_thread.battery_rate = float(np.mean(self.mean_battery))
                        self.rollout_thread.time_offset = float(self.mission_time)
                    print(self.rollout_thread)
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
            if goa_ret is not None:
                labels = [self.label_6, self.label_7, self.label_8, self.label_15, self.label_16]
                goas = []
                for gg, label in zip(goa_ret.items(), labels):
                    outcome = gg[1]
                    goas.append(outcome)
                    label.setStyleSheet(
                        'background-color: {}; color: black'.format(
                            goa.semantic_label_color(outcome)))
                    label.setText("{}".format(goa.semantic_label_text(outcome)))
                self.goa = goas
            if self.condition == self.COND_ETGOA:
                self.etgoa.preprocess()
            if self.mission_phase.state == ControlModeState.phase_mission_planning:
                self.mission_mode.state = ControlModeState.planning
            elif self.mission_phase.state == ControlModeState.phase_mission_execution:
                self.mission_mode.state = ControlModeState.execution
                # This is a terrible workaround for being able to trigger the POI accept button callbacks
                self.poi_accepted_workaround = True
                self.accept_poi_button.setEnabled(True)
                self.accept_poi_button.click()
                self.poi_accepted_workaround = False
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
                t = self.mission_time
                self.etgoa.set_start_time(t)
                px, py, pv, pb = self.position.x, self.position.y, self.velocity, self.battery_level
                si = self.etgoa.get_si(px, py, pv, pb, t)
                self.mqa = si
                if np.min(self.mqa) <= self.et_goa_threshold:
                    self.stop_mode_button.click()
                    print('Should trigger a stop + reassessment with:')
                    print('    speed: {:.2f}'.format(np.mean(self.mean_velocity)))
                    print('    battery level: {:.2f}'.format(self.battery_level))
                    print('    battery rate: {:.2f}'.format(np.mean(self.mean_battery)))
                    print('    position: ({:.2f}, {:.2f})'.format(self.position.x, self.position.y))
                    print('    heading: {:.2f}'.format(self.heading))
                    self.start_competency_assessment()

            # TODO need an in-mission assmt state so we don't hit the below case when ET-GOA triggers
            if self.control_mode.state == ControlModeState.stopped and self.condition == self.COND_ETGOA:
                self.etgoa.forget_start_time()
                self.mqa = [0] * 3
        except Exception as e:
            traceback.print_exc()

    def activate_buttons(self):
        """
        Disable control and communications buttons during processing

        if we are in mission execution, always disable poi plan and accept buttons
        if executing + assessing, disable
        :return:
        """
        try:
            if self.mission_phase.state == ControlModeState.phase_mission_complete:
                # disable everything when mission complete
                self.drive_mode_button.setDisabled(True)
                self.request_mission_control_help_button.setDisabled(True)
                self.mission_control_update_text.setDisabled(True)
                self.poi_selection.setDisabled(True)
                self.plan_poi_button.setDisabled(True)
                self.accept_poi_button.setDisabled(True)
                self.robot_battery_slider.setDisabled(True)
                self.robot_gps_dial.setDisabled(True)
                self.robot_power_slider.setDisabled(True)
                return 0

            # Mission Planning phase
            elif self.mission_phase.state == ControlModeState.phase_mission_planning:
                self.drive_mode_button.setDisabled(True)
                self.request_mission_control_help_button.setDisabled(True)
                self.mission_control_update_text.setDisabled(True)
                self.poi_selection.setEnabled(True)
                self.plan_poi_button.setEnabled(True)
                if self.mission_manager.has_plan() and self.mission_mode.state is not ControlModeState.assessing:
                    self.accept_poi_button.setEnabled(True)
                else:
                    self.accept_poi_button.setDisabled(True)

            # Mission execution
            elif self.mission_phase.state == ControlModeState.phase_mission_execution:
                self.poi_selection.setDisabled(True)
                self.plan_poi_button.setDisabled(True)
                if self.poi_accepted_workaround:
                    self.accept_poi_button.setEnabled(True)
                else:
                    self.accept_poi_button.setDisabled(True)
                self.drive_mode_button.setEnabled(True)
                self.stop_mode_button.setEnabled(True)

            if self.mission_mode.state == ControlModeState.assessing or self.mission_mode.state == ControlModeState.planning:
                self.drive_mode_button.setDisabled(True)
                self.request_mission_control_help_button.setDisabled(True)
                self.mission_control_update_text.setDisabled(True)
            elif self.mission_mode.state == ControlModeState.execution:
                self.drive_mode_button.setEnabled(True)
                self.request_mission_control_help_button.setEnabled(True)
                self.mission_control_update_text.setEnabled(True)

            if self.poi_accepted_workaround:
                self.accept_poi_button.setEnabled(True)

        except Exception as e:
            traceback.print_exc()

    def survey_prompt(self, survey):
        try:
            if not self.surveys[survey]:
                self.surveys[survey] = True
                # run_survey_popup_offline(survey)
                responses, score = run_survey_popup_online()
                # self.survey_recorder.add_row(responses, score, datetime.now())
        except Exception as e:
            traceback.print_exc()

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
