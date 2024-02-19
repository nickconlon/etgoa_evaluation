import time
import traceback
from collections import deque
from PIL import Image
from PyQt5.QtWidgets import QMainWindow, QApplication
import PyQt5.QtCore as QtCore
from PyQt5 import QtMultimedia
import PyQt5.QtGui as QtGui
import sys
from datetime import datetime
import numpy as np
import os

from motion_planning.mission_management import MissionManager
from base_interface.control_modes import ControlModeState
from base_interface.ui_mdrs import Ui_MainWindow
from analysis.data_recorder import PrimaryTaskRecorder, SurveyRecorder
from motion_planning.projections import Projector, PointOfInterest, get_heading, to_orientation_rhr
from famsec import goa, rollout, et_goa
from base_interface.settings import Settings
from mission_control.mission_control import MissionControl
from motion_planning.rrt import Obstacle

from surveys.trust_survey_popup import run_survey_popup_online as trust_survey
from surveys.usability_survey_popup import run_survey_popup_online as usability_survey


class BaseInterface(QMainWindow, Ui_MainWindow):
    COND_TELEMETRY = 'TELEM'
    COND_GOA = 'GOA'
    COND_ETGOA = 'ET-GOA'

    def __init__(self, settings_path):
        QMainWindow.__init__(self)
        self.setupUi(self)
        settings = Settings(settings_path)
        settings.read()
        self.area = settings.area
        self.robot = settings.robot_name
        self.cohrint_logo_img_path = settings.logo_path
        self.mission_area_img_path = settings.map_path
        self.rollout_path = settings.rollout_path
        self.condition = settings.condition

        data_fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_primary_{}.csv'.format(settings.condition)
        self.data_recorder = PrimaryTaskRecorder(self.condition, settings.map, os.path.join(settings.record_path, data_fname))
        trust_fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_trust_survey_{}.csv'.format(settings.condition)
        usability_fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_usability_survey_{}.csv'.format(
            settings.condition)
        self.survey_recorder = SurveyRecorder(os.path.join(settings.record_path, trust_fname),
                                              os.path.join(settings.record_path, usability_fname))

        self.projector = Projector(settings.lat_center, settings.lon_center)
        self.projector.setup()
        self.mission_manager = MissionManager(self.mission_area_img_path, self.projector,
                                              settings.pois,
                                              settings.obstructions,
                                              settings.hazards,
                                              settings.power_draws)
        self.batt_drain_rate = settings.batt_drain_anomaly

        self.mission_time = 0.0
        self.t0 = datetime.now()
        self.update_rate = 0.5  # seconds
        self.poi_accepted_workaround = False

        #################
        # Setup the System Connections
        print('Setting up connections')
        self.robot_connected = -100
        self.gps_connected = -100
        self.sensor1_connected = -100
        self.sensor2_connected = -100

        #################
        # Setup the robot state
        print('Setting up state')
        """
        planning
        planning_assessing
        executing_manual
        executing_auto_stopped
        executing_auto_driving
        completed
        """
        self.test_state_test = ControlModeState(ControlModeState.planning)
        self.experiencing_anomaly = ''

        #################
        # Setup the Mission Planning Panel
        print('Setting up Mission Planning')
        self.poi_selected = None
        self.poi_accepted = None
        self.accept_poi_button.clicked.connect(self.accept_poi_callback)
        self.plan_poi_button.clicked.connect(self.plan_poi_callback)
        self.poi_selection.currentTextChanged.connect(self.select_poi_callback)

        #################
        # Setup the Telemetry Panel
        print('Setting up telemetry')
        self.battery_level = 100
        self.old_battery_level = 100
        self.battery_number = 1
        self.power_number = 50
        self.gps_frequency = 68
        self.velocity = 0.0
        self.heading = 0.0
        self.position = PointOfInterest()
        self.time_start = self.t0
        self.telemetry_updater = QtCore.QTimer()
        self.telemetry_updater.timeout.connect(self.periodic_update)
        self.telemetry_updater.setInterval(int(self.update_rate * 1000))
        self.telemetry_updater.start()

        self.mean_velocity = deque([0.25] * 10, maxlen=10)
        self.mean_battery = deque(maxlen=10)

        #################
        # Setup the Robot Control panel
        print('Setting up robot control')
        self.automatic_drive_mode_button.clicked.connect(lambda: self.state_update_test('auto_drive_selected'))
        self.automatic_drive_mode_button.setDisabled(True)
        self.manual_drive_mode_button.clicked.connect(lambda: self.state_update_test('manual_drive_selected'))
        self.stop_mode_button.clicked.connect(lambda: self.state_update_test('stopping'))
        self.poi_selection.addItems(["Select POI", *["POI {}".format(x.name) for x in settings.pois]])

        #################
        # Setup the Competency Assessment Panel
        self.etgoa = et_goa.et_goa(min_stds=settings.et_goa_stds)
        self.mission_objectives = {}
        if self.condition == self.COND_GOA or self.condition == self.COND_ETGOA:
            print('Setting up Competency Assessment')
            self.etgoa.set_pred_paths([self.rollout_path.format(i) for i in range(10)])
            self.rollout_thread = None
            self.et_goa_threshold = settings.et_goa_threshold
            self.objective_2_text.setText(self.objective_2_text.text().replace('X', '5'))
            self.objective_3_text.setText(self.objective_3_text.text().replace('X %', '50%'))
        self.mqa = [0] * len(settings.et_goa_stds)
        self.goa = [0] * 5  # []

        #################
        # Setup the mission control panel
        print('Setting up Mission Control')
        self.mission_control = MissionControl(self.gps_frequency,
                                              self.power_number,
                                              self.battery_number,
                                              100, 100, 5)
        self.mission_control.set_mission_pois(settings.available_pois)
        if settings.mode == 'cu':
            self.request_help_button.clicked.connect(
                self.request_mission_control_help_callback)
            self.request_help_button.clicked.connect(lambda: self.splash_of_color(self.request_help_button))
            self.robot_power_slider.setValue(self.power_number)
            self.num_backup_batteries = settings.num_backup_batteries
            self.robot_battery_slider.setMaximum(self.num_backup_batteries)
            self.robot_battery_slider.setMinimum(1)
            self.robot_battery_slider.setValue(self.battery_number)
            self.robot_gps_slider.setValue(self.gps_frequency)
            self.robot_power_slider.valueChanged.connect(self.update_robot_power_callback)
            self.robot_battery_slider.valueChanged.connect(self.update_robot_battery_callback)
            self.robot_gps_slider.valueChanged.connect(self.update_robot_gps_frequency_callback)
            self.robot_battery_slider.setDisabled(True)
            self.robot_gps_slider.setDisabled(True)
            self.robot_power_slider.setDisabled(True)
        else: # TODO cover this area with something fun
            self.mission_control_panel.setVisible(False)

        #################
        # Setup the mission prompt panel
        self.mission_text.setText(self.mission_control.send_mission())
        self.assessment_started_sound = QtMultimedia.QSound("./imgs/start_assmt.wav")
        self.assessment_finished_sound = QtMultimedia.QSound("./imgs/end_assmt.wav")
        self.arrive_sound = QtMultimedia.QSound("./imgs/arrive.wav")
        self.update_ff_camera()
        self.manual_drive_mode_button.clicked.connect(self.manual_mode_callback)

        ##################
        # Dynamically add/delete obstacles panel
        # TODO edit existing obstacles
        self.obstacle_scroll.addItem('Add new obstacle')
        self.obs_set = {}
        self.obs_counter = 5
        self.obstacle_confirm.clicked.connect(self.add_obstacle)

        ##################
        # Automatic surveys
        self.surveys = settings.show_surveys


    def add_obstacle(self):
        try:
            if self.obstacle_scroll.currentText() == 'Add new obstacle':
                print('adding new obstacle')
                r = float(self.obstacle_r.toPlainText())
                x = float(self.obstacle_x.toPlainText())
                y = float(self.obstacle_y.toPlainText())
                oid = 'o{}'.format(self.obs_counter)
                self.obs_counter += 1
                ob = Obstacle(Obstacle.circle, [x, y], [r], oid)
                self.mission_manager.setup_obstacles([ob])
                self.mission_manager.activate_obstacles([ob.id])

                self.obs_set[oid] = ob
                self.obstacle_scroll.addItem(oid)
                self.obstacle_r.setText('')
                self.obstacle_x.setText('')
                self.obstacle_y.setText('')
            else:
                print('deleting obstacle')
                oid = self.obstacle_scroll.currentText()
                idx = self.obstacle_scroll.currentIndex()
                self.obstacle_scroll.removeItem(idx)
                self.mission_manager.remove_obstacles([oid])
                self.obstacle_scroll.setCurrentIndex(0)

        except Exception as e:
            traceback.print_exc()

    def periodic_update(self):
        """
        Updater for most periodic updates on the UI

        :return:
        """
        try:
            if self.test_state_test.state is not ControlModeState.phase_mission_done:
                self.update_mission_time()
                self.update_battery_level()
                self.update_connections()
                self.update_robot_state_text()
                self.update_battery_text()
                self.update_velocity_text()
                self.update_heading_text()
                self.update_position_text()

                mission_control_text = self.request_help_text.text()
                mission_control_text = mission_control_text.replace('\n', ' ').replace(',', '.')
                self.data_recorder.add_row(self.position.x, self.position.y, self.position.z,
                                           self.heading, self.velocity,
                                           self.test_state_test,
                                           self.battery_number, self.battery_level,
                                           self.power_number, self.gps_frequency,
                                           self.experiencing_anomaly,
                                           mission_control_text,
                                           "|".join(["{:.2f}".format(x) for x in self.goa]),
                                           "|".join(["{:.2f}".format(x) for x in self.mqa]),
                                           "|".join([str(x) for x in self.mission_objectives.values()]),
                                           self.mission_time)
                self.mission_objectives = {}
                self.update_map()
                self.update_et_goa()

            # always update these
            self.update_complete()
            #self.update_surveys()
            self.update_time_text()
            self.activate_buttons()

        except Exception as e:
            traceback.print_exc()

    def update_surveys(self):
        if self.test_state_test.state == ControlModeState.planning:
            if 'pre-planning' in self.surveys and self.surveys['pre-planning']:
                self.surveys['pre-planning'] = False
                self.trust_survey_prompt()

        if self.test_state_test.state == ControlModeState.execution:
            if 'post-planning' in self.surveys and self.surveys['post-planning']:
                self.surveys['post-planning'] = False
                self.trust_survey_prompt()

        if self.test_state_test.state == ControlModeState.phase_mission_done:
            if 'post-mission' in self.surveys and self.surveys['post-mission']:
                self.surveys['post-mission'] = False
                self.trust_survey_prompt()

            if 'usability' in self.surveys and self.surveys['usability']:
                self.surveys['usability'] = False
                self.usability_survey_prompt()

    def update_complete(self):
        if self.test_state_test.state == ControlModeState.completed:
            self.arrive_sound.play()
            self.update_assessment_mission_complete()
            if self.mission_manager.captured_home:
                print('home captured at t=', self.mission_time)
                self.state_update_test('captured_home')
            elif self.mission_manager.captured_goal:
                print('goal captured at t=', self.mission_time)
                self.state_update_test('captured_goal')


    def state_update_test(self, transition):
        #print()
        #print('updating state from: ', self.test_state_test)
        #print('With transition: ', transition)

        next_state = self.test_state_test.state

        if transition == 'anomaly_found':
            next_state = ControlModeState.anomaly_found

        if transition == 'started_planning':
            next_state = ControlModeState.planning

        # Planning state changes
        if self.test_state_test.state == ControlModeState.planning and transition =='started_assessing':
            next_state = ControlModeState.planning_assessing
        if self.test_state_test.state == ControlModeState.planning_assessing and transition == 'completed_assessment':
            next_state = ControlModeState.planning
        if self.test_state_test.state == ControlModeState.planning and transition == 'accepted_plan':
            next_state = ControlModeState.executing

        # Autonomous driving state changes
        if self.test_state_test.state == ControlModeState.executing and transition == 'auto_drive_selected':
            next_state = ControlModeState.executing_auto_driving
        if self.test_state_test.state == ControlModeState.executing_auto_driving and transition == 'stopping':
            next_state = ControlModeState.executing_auto_stopped
        if self.test_state_test.state == ControlModeState.executing and transition == 'manual_drive_selected':
            next_state = ControlModeState.executing_manual
        if self.test_state_test.state == ControlModeState.executing_auto_stopped and transition == 'manual_drive_selected':
            next_state = ControlModeState.executing_manual
        if self.test_state_test.state == ControlModeState.executing_auto_stopped and transition == 'auto_drive_selected':
            next_state = ControlModeState.executing_auto_driving

        # Manual driving state changes
        if self.test_state_test.state == ControlModeState.planning and transition ==  'manual_drive_selected':
            next_state = ControlModeState.executing_manual
        if self.test_state_test.state == ControlModeState.executing_manual and transition == 'auto_drive_selected':
            next_state = ControlModeState.executing_auto_stopped
        if self.test_state_test.state == ControlModeState.executing_manual and transition == 'manual_drive_selected':
            next_state = ControlModeState.planning

        # In mission assessment state change
        if self.test_state_test.state == ControlModeState.executing_auto_driving and transition == 'started_assessing':
            next_state = ControlModeState.executing_auto_stopped_assessing
        if self.test_state_test.state == ControlModeState.executing_auto_stopped_assessing and transition == 'completed_assessment':
            next_state = ControlModeState.executing
        if self.test_state_test.state == ControlModeState.executing_manual and transition == 'started_assessing':
            next_state = ControlModeState.executing_manual_stopped_assessing
        if self.test_state_test.state == ControlModeState.executing_manual_stopped_assessing and transition == 'completed_assessment':
            next_state = ControlModeState.executing

        if self.test_state_test.state == ControlModeState.anomaly_found and transition == 'anomaly_fixed':
            next_state = ControlModeState.planning

        # Mission complete state change
        if self.test_state_test.state == ControlModeState.executing_auto_driving and transition == 'nav_completed':
            #self.update_assessment_mission_complete()
            #self.mission_manager.hit_known_hazards = 0
            next_state = ControlModeState.completed
            #self.arrive_sound.play()

        if self.test_state_test.state == ControlModeState.completed and transition == 'captured_home':
            next_state = ControlModeState.phase_mission_done
        if self.test_state_test.state == ControlModeState.completed and transition == 'captured_goal':
            next_state = ControlModeState.planning

        self.test_state_test.state = next_state
        #print('to: ', self.test_state_test)
        #print()

    def check_planning_phase_limit(self):
        # TODO
        pass

    def check_execution_phase_limit(self):
        # TODO
        pass

    def navigation_complete(self):
        try:
            self.state_update_test('nav_completed')
        except Exception as e:
            traceback.print_exc()

    def update_assessment_mission_complete(self):
        try:
            achieve, fail = 'Achieved', 'Failed to Achieve'
            green, red = 'green', 'red'
            colors, assmts = [], []
            print('Outcomes:')
            # Reached the goal
            achieved = self.mission_manager.captured_goal or self.mission_manager.captured_home
            colors.append(green if achieved else red)
            assmts.append(achieve if achieved else fail)
            self.mission_objectives['goal'] = int(achieved)
            print('     Goal outcome :', self.mission_manager.captured_home, self.mission_manager.captured_goal)

            # Arrived within time limit
            achieved = self.mission_time < 5 * 60
            colors.append(green if achieved else red)
            assmts.append(achieve if achieved else fail)
            self.mission_objectives['time'] = int(achieved)
            print('     Time outcome :', self.mission_time)

            # Battery always above threshold
            achieved = self.battery_level >= 50 and self.old_battery_level >= 50
            colors.append(green if achieved else red)
            assmts.append(achieve if achieved else fail)
            self.mission_objectives['battery'] = int(achieved)
            print('     Battery outcome :', self.battery_level)

            # Battery always above threshold
            achieved = self.mission_manager.hit_known_hazards == 0
            colors.append(green if achieved else red)
            assmts.append(achieve if achieved else fail)
            self.mission_objectives['hazards'] = int(achieved)
            print('     Avoidance outcome :', self.mission_manager.hit_known_hazards)

            # TODO outcome
            achieved = True
            colors.append(green if achieved else red)
            assmts.append(achieve if achieved else fail)
            self.mission_objectives['todo'] = int(achieved)
            print('     TODO outcome :', self.mission_manager.hit_known_hazards)

            self.update_assessment_text(assmts, colors)
        except Exception as e:
            traceback.print_exc()

    def update_map(self):
        """
        Updater for the map
        :return:
        """
        try:
            complete = self.mission_manager.update_progress(self.position.x, self.position.y)
            if complete:
                self.navigation_complete() # TODO when we don't actually have a plan vs we just completed a plan
            img = self.mission_manager.get_overlay_image(self.position.x, self.position.y, to_orientation_rhr(self.heading)+180+90)
            height, width, channel = img.shape
            bytesPerLine = 3 * width
            qImg = QtGui.QImage(img, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.map_label.setPixmap(QtGui.QPixmap(qImg))
        except Exception as e:
            traceback.print_exc()

    def update_ff_camera(self):
        img = Image.open('./imgs/ff_camera.jpg')
        img = np.array(img)
        height, width, channel = img.shape
        bytesPerLine = 3 * width
        qImg = QtGui.QImage(img, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
        self.camera_label.setPixmap(QtGui.QPixmap(qImg))

    def update_connections(self):
        """
        Updater for connection status

        :return:
        """
        try:

            self.robot_connected_indicator.setStyleSheet(
                'background-color: {}'.format('green' if abs(self.robot_connected - self.mission_time) < 2 else 'red'))
            self.gps_connected_indicator.setStyleSheet(
                'background-color: {}'.format('green' if abs(self.gps_connected - self.mission_time) < 2 else 'red'))
            self.camera360_connected_indicator.setStyleSheet(
                'background-color: {}'.format('green' if abs(self.sensor1_connected - self.mission_time) < 5 else 'red'))
            self.cameraFF_connected_indicator.setStyleSheet(
                'background-color: {}'.format('green' if abs(self.sensor2_connected - self.mission_time) < 5 else 'red'))
        except Exception as e:
            traceback.print_exc()

    def update_robot_state_text(self):
        """
        Updater for control mode

        :return:
        """
        try:
            text = '{}'.format(self.test_state_test)
            self.state_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_position_text(self):
        """
        Updater for position

        :return:
        """
        try:
            self.latitude_label.setText('x: {:.2f} m'.format(self.position.x))
            self.longitude_label.setText('y: {:.2f} m'.format(self.position.y))
            self.altitude_label.setText('z:  {:.2f} m'.format(self.position.z))
        except Exception as e:
            traceback.print_exc()

    def update_heading_text(self):
        """
        Updater for heading

        :return:
        """
        try:
            text = '{:03.0f} deg | {}'.format(self.heading, get_heading(self.heading))
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
            if (self.test_state_test.state == ControlModeState.executing_auto_driving
                    or self.test_state_test.state == ControlModeState.executing_manual):
                dt = self.update_rate
                drain_rate = self.batt_drain_rate
                anomalies = []
                for o in self.mission_manager.get_active_obstacles():
                    dh = o.distance(self.position.x, self.position.y)
                    if dh <= o.axis[0]:
                        if o.visible:
                            self.mission_manager.hit_known_hazards += 1
                            print('hit known obstacle at t=', self.mission_time)
                        else:
                            print('hit unknown obstacle at t=', self.mission_time)
                        anomalies.append(o.id)
                        if 'b' in o.id:
                            drain_rate = o.data
                self.experiencing_anomaly = '|'.join(anomalies)

                prev_battery = self.battery_level
                new_battery = float(np.maximum(self.battery_level - dt * drain_rate, 0.0))
                self.mean_battery.append(abs(prev_battery - new_battery) / dt)
                self.battery_level = new_battery
                if self.battery_level <= 0:
                    self.stop_mode_button.click()
        except Exception as e:
            traceback.print_exc()

    def update_battery_text(self):
        """
        Updater for battery level

        :return:
        """
        try:
            if self.mission_control.backup_batts_used == 0:
                text = 'Batt 1: {}% Batt 2: {}%'.format(int(self.battery_level), 100)
            else:
                text = 'Batt 1: {}% Batt 2: {}%'.format(int(self.old_battery_level), int(self.battery_level))
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
            else:
                self.poi_selected = None
        except Exception as e:
            traceback.print_exc()

    def plan_poi_callback(self):
        """
        Callback to generate a waypoint plan to the selected POI

        :return:
        """
        self.accept_poi_button.setStyleSheet('background-color: light grey')
        self.assessment_started_sound.play()
        if self.poi_selected:
            self.update_assessment_text(['']*5, ['light grey']*5)
            self.state_update_test('started_planning')
            self.mission_manager.delete_plan()
            print('Planning route to POI: ', self.poi_selected)
            self.mission_manager.plan_known_poi(self.position.x, self.position.y, self.poi_selected.replace('POI ', ''),
                                                tofrom=False)
            if self.mission_manager.get_plan() is None:
                self.splash_of_color(self.frame_2, color='red')
                t = self.mission_text.text()
                t += "\n{} is unreachable!".format(self.poi_selected)
                self.mission_text.setText(t)
            else:
                self.splash_of_color(self.frame_2, color='green')
            self.start_competency_assessment('planning_assessing')
        else:
            self.splash_of_color(self.poi_selection, color='red')

    def accept_poi_callback(self):
        """
        Callback to accept a POI

        :return:
        """
        try:
            if self.poi_selected and self.mission_manager.has_plan():
                self.state_update_test('accepted_plan')
                print('Accepted POI: ', self.poi_selected)
                self.poi_accepted = self.poi_selected
                self.splash_of_color(self.accept_poi_button, color='green')
        except Exception as e:
            traceback.print_exc()

    def start_competency_assessment(self, trigger):
        """
        Callback to start a GOA competency assessment

        :return:
        """
        try:
            if self.condition == self.COND_ETGOA or self.condition == self.COND_GOA:
                if self.mission_manager.has_plan():
                    # Maybe add back the current (x,y) location to the beginning of the plan?
                    self.state_update_test('started_assessing')
                    self.splash_of_color(self.competency_assessment_frame, color='light grey', timeout=0)
                    labels = [self.objective_1_assmt, self.objective_2_assmt, self.objective_3_assmt,
                              self.objective_4_assmt, self.objective_5_assmt]
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
                    self.rollout_thread.known_obstacles = self.mission_manager.get_all_active_visible_obstacles()
                    self.rollout_thread.time_offset = self.mission_time
                    if trigger == 'planning_assessing':
                        # During mission planning, parameter values are at their baseline
                        self.rollout_thread.velocity_rate = 1.0
                        self.rollout_thread.battery_rate = self.batt_drain_rate
                    elif trigger == 'et_goa':
                        # During mission execution, parameter values may have changed
                        # TODO readjust this, so we don't accidentally get zero velocity
                        self.rollout_thread.velocity_rate = float(
                            np.mean(np.asarray(self.mean_velocity)) / 0.25)
                        self.rollout_thread.battery_rate = float(np.mean(np.asarray(self.mean_battery)))
                        self.rollout_thread.time_offset = float(self.mission_time)
                    print(self.rollout_thread)
                    self.rollout_thread.finished.connect(self.finish_competency_assessment)
                    self.rollout_thread.start()
                else:
                    goas = {x:0 for x in range(5)}
                    self.finish_competency_assessment(goas)
            else:
                time.sleep(1)
                self.assessment_finished_sound.play()
        except Exception as e:
            traceback.print_exc()

    def update_assessment_text(self, texts, colors):
        labels = [self.objective_1_assmt,  # arrive at POI
                  self.objective_2_assmt,  # time
                  self.objective_3_assmt,  # battery level
                  self.objective_4_assmt,  # avoid stuff
                  self.objective_5_assmt]
        for text, color, label in zip(texts, colors, labels):
            label.setStyleSheet(
                'background-color: {}; color: black'.format(color))
            label.setText("{}".format(text))


    def finish_competency_assessment(self, goa_ret):
        """
        Callback when a GOA competency assessment is completed

        :param goa_ret:
        :return:
        """
        try:
            if goa_ret is not None:
                self.state_update_test('completed_assessment')

                goas_val = []
                goa_text = []
                colors = []

                for gg in goa_ret.items():
                    outcome = gg[1]
                    goas_val.append(outcome)
                    goa_text.append(goa.semantic_label_text(outcome))
                    colors.append(goa.semantic_label_color(outcome))

                self.update_assessment_text(goa_text, colors)
                self.goa = goas_val
            if self.condition == self.COND_ETGOA:
                self.etgoa.preprocess()
            if self.test_state_test.state == ControlModeState.executing:
                self.accept_poi_button.setEnabled(True)
            self.splash_of_color(self.competency_assessment_frame, color='light grey', timeout=0)
            self.assessment_finished_sound.play()
        except Exception as e:
            traceback.print_exc()

    def update_et_goa(self):
        """
        Updater for the ET-GOA algorithm

        :return:
        """
        try:
            if (self.test_state_test.state == ControlModeState.executing_auto_driving
                    and self.etgoa.has_data() and self.condition == self.COND_ETGOA):
                t = self.mission_time
                self.etgoa.set_start_time(t)
                px, py, pv, pb = self.position.x, self.position.y, self.velocity, self.battery_level
                si = self.etgoa.get_si(px, py, pv, pb, t)
                self.mqa = si
                if np.min(self.mqa) <= self.et_goa_threshold:
                    self.assessment_started_sound.play()
                    self.state_update_test('started_assessing')
                    self.stop_mode_button.click()
                    print('Should trigger a stop + reassessment with:')
                    print('    speed: {:.2f}'.format(np.mean(np.asarray(self.mean_velocity))))
                    print('    battery level: {:.2f}'.format(self.battery_level))
                    print('    battery rate: {:.2f}'.format(np.mean(np.asarray(self.mean_battery))))
                    print('    position: ({:.2f}, {:.2f})'.format(self.position.x, self.position.y))
                    print('    heading: {:.2f}'.format(self.heading))
                    self.start_competency_assessment('et_goa')

            # TODO need an in-mission assmt state so we don't hit the below case when ET-GOA triggers
            if self.test_state_test.state != ControlModeState.executing_auto_driving and self.condition == self.COND_ETGOA:
                self.etgoa.forget_start_time()
                self.mqa = [0] * 3
        except Exception as e:
            traceback.print_exc()

    def manual_mode_callback(self):
        self.stop_mode_button.click()
        self.mission_manager.delete_plan()
        self.splash_of_color(self.accept_poi_button, color='light grey', timeout=0)


    def activate_buttons(self):
        """
        Disable control and communications buttons during processing

        if we are in mission execution, always disable poi plan and accept buttons
        if executing + assessing, disable
        :return:
        """
        try:
            if self.test_state_test.state == ControlModeState.planning:
                self.splash_of_color(self.manual_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.automatic_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.stop_mode_button, color='green', timeout=0)

                # Stop always enabled
                self.stop_mode_button.setEnabled(True)

                # control modes
                self.automatic_drive_mode_button.setDisabled(True)
                self.manual_drive_mode_button.setEnabled(True)

                # Manual control buttons
                self.forward_button_2.setDisabled(True)
                self.back_button_2.setDisabled(True)
                self.left_button_2.setDisabled(True)
                self.right_button_2.setDisabled(True)

                # Planning buttons
                self.poi_selection.setEnabled(True)
                self.plan_poi_button.setEnabled(True)
                if self.mission_manager.has_plan():
                    self.accept_poi_button.setEnabled(True)
                else:
                    self.accept_poi_button.setDisabled(True)

                # Mission Control buttons
                self.request_help_button.setDisabled(True)

            if self.test_state_test.state == ControlModeState.planning_assessing:
                self.splash_of_color(self.manual_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.automatic_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.stop_mode_button, color='green', timeout=0)

                # Stop always enabled
                self.stop_mode_button.setEnabled(True)

                # control modes
                self.automatic_drive_mode_button.setDisabled(True)
                self.manual_drive_mode_button.setDisabled(True)

                # Manual control buttons
                self.forward_button_2.setDisabled(True)
                self.back_button_2.setDisabled(True)
                self.left_button_2.setDisabled(True)
                self.right_button_2.setDisabled(True)

                # Planning buttons
                self.poi_selection.setDisabled(True)
                self.plan_poi_button.setDisabled(True)
                self.accept_poi_button.setDisabled(True)

                # Mission Control buttons
                self.request_help_button.setDisabled(True)

            if self.test_state_test.state == ControlModeState.executing:
                self.splash_of_color(self.manual_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.automatic_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.stop_mode_button, color='green', timeout=0)

                # Stop always enabled
                self.stop_mode_button.setEnabled(True)

                # control modes
                self.automatic_drive_mode_button.setEnabled(True)
                self.manual_drive_mode_button.setEnabled(True)

                # Manual control buttons
                self.forward_button_2.setDisabled(True)
                self.back_button_2.setDisabled(True)
                self.left_button_2.setDisabled(True)
                self.right_button_2.setDisabled(True)

                # Planning buttons
                self.poi_selection.setEnabled(True)
                self.plan_poi_button.setEnabled(True)
                self.accept_poi_button.setDisabled(True)

                # Mission Control buttons
                self.request_help_button.setEnabled(True)

            if self.test_state_test.state == ControlModeState.executing_manual:
                self.splash_of_color(self.manual_drive_mode_button, color='green', timeout=0)
                self.splash_of_color(self.automatic_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.stop_mode_button, color='light grey', timeout=0)

                # Stop always enabled
                self.stop_mode_button.setDisabled(True)

                # control modes
                self.automatic_drive_mode_button.setDisabled(True)
                self.manual_drive_mode_button.setEnabled(True)

                # Manual control buttons
                self.forward_button_2.setEnabled(True)
                self.back_button_2.setEnabled(True)
                self.left_button_2.setEnabled(True)
                self.right_button_2.setEnabled(True)

                # Planning buttons
                self.poi_selection.setEnabled(True)
                self.plan_poi_button.setEnabled(True)
                self.accept_poi_button.setDisabled(True)

                # Mission Control buttons
                self.request_help_button.setEnabled(True)

            if self.test_state_test.state == ControlModeState.executing_manual_stopped_assessing:
                self.splash_of_color(self.manual_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.automatic_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.stop_mode_button, color='green', timeout=0)

                # Stop always enabled
                self.stop_mode_button.setEnabled(True)

                # control modes
                self.automatic_drive_mode_button.setDisabled(True)
                self.manual_drive_mode_button.setDisabled(True)

                # Manual control buttons
                self.forward_button_2.setDisabled(True)
                self.back_button_2.setDisabled(True)
                self.left_button_2.setDisabled(True)
                self.right_button_2.setDisabled(True)
                self.accept_poi_button.setDisabled(True)

                # Planning buttons
                self.poi_selection.setEnabled(True)
                self.plan_poi_button.setEnabled(True)
                self.accept_poi_button.setDisabled(True)

                # Mission Control buttons
                self.request_help_button.setDisabled(True)

            if self.test_state_test.state == ControlModeState.executing_auto_stopped:
                self.splash_of_color(self.manual_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.automatic_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.stop_mode_button, color='green', timeout=0)

                # Stop always enabled
                self.stop_mode_button.setEnabled(True)

                # control modes
                self.automatic_drive_mode_button.setEnabled(True)
                self.manual_drive_mode_button.setEnabled(True)

                # Manual control buttons
                self.forward_button_2.setDisabled(True)
                self.back_button_2.setDisabled(True)
                self.left_button_2.setDisabled(True)
                self.right_button_2.setDisabled(True)

                # Planning buttons
                self.poi_selection.setEnabled(True)
                self.plan_poi_button.setEnabled(True)
                self.accept_poi_button.setDisabled(True)

                # Mission Control buttons
                self.request_help_button.setEnabled(True)

            if self.test_state_test.state == ControlModeState.executing_auto_driving:
                self.splash_of_color(self.manual_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.automatic_drive_mode_button, color='green', timeout=0)
                self.splash_of_color(self.stop_mode_button, color='light grey', timeout=0)

                # Stop always enabled
                self.stop_mode_button.setEnabled(True)

                # control modes
                self.automatic_drive_mode_button.setEnabled(True)
                self.manual_drive_mode_button.setEnabled(True)

                # Manual control buttons
                self.forward_button_2.setDisabled(True)
                self.back_button_2.setDisabled(True)
                self.left_button_2.setDisabled(True)
                self.right_button_2.setDisabled(True)

                # Planning buttons
                self.poi_selection.setEnabled(True)
                self.plan_poi_button.setEnabled(True)
                self.accept_poi_button.setDisabled(True)

                # Mission Control buttons
                self.request_help_button.setEnabled(True)

            if self.test_state_test.state == ControlModeState.executing_auto_stopped_assessing:
                self.splash_of_color(self.manual_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.automatic_drive_mode_button, color='light grey', timeout=0)
                self.splash_of_color(self.stop_mode_button, color='green', timeout=0)

                # Stop always enabled
                self.stop_mode_button.setEnabled(True)

                # control modes
                self.automatic_drive_mode_button.setDisabled(True)
                self.manual_drive_mode_button.setDisabled(True)

                # Manual control buttons
                self.forward_button_2.setDisabled(True)
                self.back_button_2.setDisabled(True)
                self.left_button_2.setDisabled(True)
                self.right_button_2.setDisabled(True)

                # Planning buttons
                self.poi_selection.setDisabled(True)
                self.plan_poi_button.setDisabled(True)
                self.accept_poi_button.setDisabled(True)

                # Mission Control buttons
                self.request_help_button.setDisabled(True)

            if self.test_state_test.state == ControlModeState.phase_mission_done:
                # Make sure we are stopped, then disable everything

                self.stop_mode_button.click()
                # Stop always enabled
                self.stop_mode_button.setDisabled(True)

                # control modes
                self.automatic_drive_mode_button.setDisabled(True)
                self.manual_drive_mode_button.setDisabled(True)

                # Manual control buttons
                self.forward_button_2.setDisabled(True)
                self.back_button_2.setDisabled(True)
                self.left_button_2.setDisabled(True)
                self.right_button_2.setDisabled(True)

                # Planning buttons
                self.poi_selection.setDisabled(True)
                self.plan_poi_button.setDisabled(True)
                self.accept_poi_button.setDisabled(True)

                # Mission Control buttons
                self.request_help_button.setDisabled(True)

        except Exception as e:
            traceback.print_exc()

    def update_robot_power_callback(self):# TODO anomaly
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

    def update_robot_battery_callback(self):# TODO anomaly
        """
        Updater for robot battery LCD

        :return:
        """
        try:
            val = self.robot_battery_slider.value()
            self.robot_battery_lcd.display(val)
            self.battery_number = int(val)
            self.check_anomaly_strategy()
        except Exception as e:
            traceback.print_exc()

    def update_robot_gps_frequency_callback(self):# TODO anomaly
        """
        Updater for robot GPS LCD

        :return:
        """
        try:
            val = self.robot_gps_slider.value()
            self.robot_gps_lcd.display(val)
            self.gps_frequency = val
            self.check_anomaly_strategy()
        except Exception as e:
            traceback.print_exc()

    def check_anomaly_strategy(self):# TODO anomaly
        """
        Check that the strategy fixed the anomaly

        :return:
        """
        try:
            if self.mission_control:
                text = self.mission_control.check_strategy(self.power_number, self.gps_frequency,
                                                           self.battery_number)
                if text != "":
                    print('Anomaly strategy successful! at t=', self.mission_time)
                    self.assessment_started_sound.play()
                    self.old_battery_level = self.battery_level
                    self.battery_level = 100 # TODO switch only if a battery anomaly!!!!!!!!!!
                    self.robot_battery_slider.setDisabled(True)
                    self.robot_gps_slider.setDisabled(True)
                    self.robot_power_slider.setDisabled(True)
                    self.request_help_button.setEnabled(True)
                    self.update_mission_control_text(text, 'green')

                    # if the anomaly resolution was successful, deactivate the obstacle
                    addressed_anomalies = []
                    for ob_id, o in self.mission_manager.all_obstacles.items():
                        if o.distance(self.position.x, self.position.y) <= o.axis[0] + 0.1:
                            addressed_anomalies.append(o.id)
                    self.mission_manager.deactivate_obstacles(addressed_anomalies)
                    self.state_update_test('anomaly_fixed')
                    if self.condition == self.COND_ETGOA:
                        self.start_competency_assessment('planning_assessing')
                    else:
                        self.finish_competency_assessment(None)
        except Exception as e:
            traceback.print_exc()

    def update_mission_control_text(self, text, color=None):# TODO anomaly
        """
        Updater for new mission control text

        :param text:
        :param color:
        :return:
        """
        try:
            if color is not None:
                self.splash_of_color(self.request_help_text, color=color, timeout=1000)
            self.request_help_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def request_mission_control_help_callback(self):# TODO anomaly
        """
        Callback when help is requested from Mission Control

        :return:
        """
        print('requesting help')

        self.accept_poi_button.setStyleSheet('background-color: light grey')
        anomaly = False
        anomaly_type = ''
        for ob_id, o in self.mission_manager.all_obstacles.items():
            if ob_id in self.mission_manager.active_obstacle_ids:
                if o.distance(self.position.x, self.position.y) <= o.axis[0]:
                    anomaly = True
                    anomaly_type = o.id

        if anomaly:
            self.stop_mode_button.click()
            self.state_update_test('anomaly_found')
            self.robot_battery_slider.setEnabled(True)
            self.robot_gps_slider.setEnabled(True)
            self.robot_power_slider.setEnabled(True)
            self.request_help_button.setDisabled(True)
            self.update_mission_control_text(self.mission_control.get_response(True, anomaly_type), 'red')
            self.state_update_test('help_request')
        else:
            self.stop_mode_button.click()
            self.state_update_test('anomaly_fixed')
            self.update_mission_control_text(self.mission_control.get_response(False, anomaly_type), 'green')

        time.sleep(0.1)  # just in case some asynch messes up the periodic button activation and this click

    def trust_survey_prompt(self):
        try:
            responses, score = trust_survey()
            self.survey_recorder.record_trust(responses, score, datetime.now())
        except Exception as e:
            traceback.print_exc()

    def usability_survey_prompt(self):
        try:
            responses, score = usability_survey()
            self.survey_recorder.record_usability(responses, score, datetime.now())
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
    MainWindow = BaseInterface('../settings.yaml')
    MainWindow.show()
    app.exec_()
