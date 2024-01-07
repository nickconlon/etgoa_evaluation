import traceback

from PyQt5.QtWidgets import QMainWindow, QApplication
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


class BaseInterface(QMainWindow, Ui_MainWindow):
    COND_TELEMENTRY = 'Telemetry'
    COND_GOA = 'GOA'
    COND_ETGOA = 'ET-GOA'

    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)
        self.condition_combo_box.addItems(['', 'telemetry', 'GOA', 'ET-GOA'])
        # TODO paths to settings.yaml
        self.cohrint_logo_path = ''  # for telemetry condition
        self.img_path = 'base_interface/mission_area.png'
        self.condition = self.COND_ETGOA  # 'pre-mission assmt' 'in-situ assmt'
        self.mission_control = MissionControl()
        recorder_fname = './data/recording_{}.csv'.format(datetime.strftime(datetime.now(), "%d.%m.%Y_%H.%M.%S"))
        print('Recording with: ', recorder_fname)
        self.data_recorder = DataRecorder(self.condition, recorder_fname)
        self.battery_model = Battery()
        lat_center, lon_center = 40.01045433, 105.24432153
        self.projector = Projector(lat_center, lon_center)
        self.projector.setup()
        self.mission_manager = MissionManager(self.img_path, self.projector)

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
        self.send_mission_control_udpate_button.clicked.connect(
            self.send_mission_control_update_callback)
        self.send_mission_control_udpate_button.clicked.connect(
            lambda: self.splash_of_color(self.send_mission_control_udpate_button))
        self.request_mission_control_help_button.clicked.connect(
            self.request_mission_control_help_callback)
        self.request_mission_control_help_button.clicked.connect(
            lambda: self.splash_of_color(self.request_mission_control_help_button))

        #################
        # Setup the Mission Planning Panel
        self.poi_selected = None
        self.accept_poi_order_button.clicked.connect(
            lambda: self.splash_of_color(self.accept_poi_order_button))
        self.accept_poi_order_button.clicked.connect(self.accept_poi_callback)
        self.poi_selection.currentTextChanged.connect(self.select_poi_callback)

        #################
        # Setup the Telemetry Panel
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
        # Setup the Robot Control panel
        self.drive_mode_button.clicked.connect(
            lambda: self.update_control_mode_state(ControlModeState.drive))
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
        # TODO paths to settings.yaml
        if self.condition is self.COND_GOA or self.condition is self.COND_ETGOA:
            self.etgoa = et_goa.et_goa()
            base = '/data/webots/rollout{}_state.npy'
            self.etgoa.set_pred_paths([base.format(i) for i in range(10)])
            self.rollout_thread = None
        else:
            self.competency_assessment_frame.hide()
        self.mqa = []  # [x, y, v]
        self.goa = []  # []

    def periodic_update(self):
        try:
            self.update_control_loa_mode_text()
            self.update_mission_mode_text()
            self.update_time_text()
            self.update_battery_text()
            self.update_velocity_text()
            self.update_heading_text()
            self.update_position_text()
            self.update_connections()
            # TODO add goa, mqa
            self.data_recorder.add_row(self.position.x, self.position.y, self.position.z,
                                       self.heading, self.velocity,
                                       self.control_mode, self.mission_mode,
                                       self.battery_model.get_number(),
                                       self.battery_model.get_level(),
                                       self.power_number, self.gps_frequency,
                                       '', '',
                                       "_".join(["{:.2f}".format(x) for x in self.goa]),
                                       "_".join(["{:.2f}".format(x) for x in self.mqa]),
                                       (datetime.now() - self.time_start).total_seconds())
            self.update_map()
            self.update_et_goa()
        except Exception as e:
            traceback.print_exc()

    def update_map(self):
        try:
            complete = self.mission_manager.update_progress(self.position.x, self.position.y)
            if complete:
                self.update_control_mode_state(ControlModeState.stopped)
                self.update_mission_mode_state(ControlModeState.planning)
            img = self.mission_manager.get_overlay_image(self.position.x, self.position.y)
            self.update_map_display(img)
        except Exception as e:
            traceback.print_exc()

    def update_connections(self):
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

    def update_control_loa_mode_text(self):
        try:
            text = '{}'.format(self.control_mode)
            self.state_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_mission_mode_text(self):
        try:
            text = '{}'.format(self.mission_mode)
            self.mode_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_LOA_mode_text(self):
        try:
            text = '{}'.format(self.mission_mode)
            self.mode_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_position_text(self):
        try:
            self.latitude_label.setText('x: {:.10f}'.format(self.position.x))
            self.longitude_label.setText('y: {:.10f}'.format(self.position.y))
            self.altitude_label.setText('z:  {:.10f}'.format(self.position.z))
        except Exception as e:
            traceback.print_exc()

    def update_heading_text(self):
        try:
            text = '{:03.0f} degrees | {}'.format(self.heading, get_heading(self.heading))
            self.heading_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_velocity_text(self):
        try:
            text = '{:.2f} m/s'.format(self.velocity)
            self.velocity_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_battery_text(self):
        try:
            text = '{:.2f}%'.format(self.battery_model.get_level())
            self.battery_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_time_text(self):
        try:
            mission_time = datetime.now() - self.time_start
            now = datetime.now()
            mins = mission_time.total_seconds() // 60
            secs = mission_time.total_seconds() % 60
            textm = "{}".format(str(int(mins)).rjust(2, "0"))
            texts = "{}".format(str(int(secs)).rjust(2, "0"))
            text = '{}:{} into {} | {} MT'.format(textm, texts, self.mission_mode,
                                                  now.strftime("%H:%M:%S"))
            self.time_text.setText(text)
        except Exception as e:
            traceback.print_exc()

    def update_robot_power_callback(self):
        try:
            val = self.robot_power_slider.value()
            self.robot_power_lcd.display(val)
        except Exception as e:
            traceback.print_exc()

    def update_robot_battery_callback(self):
        try:
            val = self.robot_battery_slider.value()
            self.robot_battery_lcd.display(val)
            self.battery_model.swap_battery(val)
        except Exception as e:
            traceback.print_exc()

    def update_robot_gps_frequency_callback(self):
        try:
            val = self.robot_gps_dial.value()
            self.robot_gps_lcd.display(val)
        except Exception as e:
            traceback.print_exc()

    def update_control_mode_state(self, new_state):
        try:
            self.control_mode.state = new_state
            if self.control_mode.state == ControlModeState.stopped:
                self.stop_mode_button.setStyleSheet('background-color: green')
                self.drive_mode_button.setStyleSheet('background-color: light gray')
            elif self.control_mode.state == ControlModeState.drive:
                self.stop_mode_button.setStyleSheet('background-color: light gray')
                self.drive_mode_button.setStyleSheet('background-color: green')
            self.update_control_loa_mode_text()
        except Exception as e:
            traceback.print_exc()

    def update_mission_mode_state(self, new_state):
        try:
            self.mission_mode.state = new_state
            self.update_mission_mode_text()
        except Exception as e:
            traceback.print_exc()

    def select_poi_callback(self):
        try:
            if self.poi_selection.currentText() == "Select POI":
                return 0
            self.splash_of_color(self.frame_2)
            self.poi_selected = self.poi_selection.currentText()
            self.mission_manager.plan_known_poi(self.position.x, self.position.y, self.poi_selected)
        except Exception as e:
            traceback.print_exc()

    def accept_poi_callback(self):
        try:
            self.mission_mode.state = ControlModeState.execution
        except Exception as e:
            traceback.print_exc()

    def update_mission_control_text(self, text):
        try:
            self.splash_of_color(self.mission_control_update_text, color='red', timeout=1000)
            self.mission_control_update_text.setPlainText(text)
        except Exception as e:
            traceback.print_exc()

    def send_mission_control_update_callback(self):
        try:
            text = self.mission_control_update_text.toPlainText()
            self.mission_control_update_text.clear()
            print(text)
        except Exception as e:
            traceback.print_exc()

    def request_mission_control_help_callback(self):
        print('asking for some help.')

    def run_competency_assessment(self):
        print('running competency assessment')

    def update_competency_assessment(self):
        try:
            self.splash_of_color(self.competency_assessment_frame)
        except Exception as e:
            traceback.print_exc()

    def update_map_display(self, img):
        try:
            height, width, channel = img.shape
            bytesPerLine = 3 * width
            qImg = QtGui.QImage(img, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.label_21.setPixmap(QtGui.QPixmap(qImg))
        except Exception as e:
            traceback.print_exc()

    def start_competency_assessment(self):
        # TODO only if GOA or ET-GOA condition
        try:
            self.mission_mode.state = ControlModeState.assessing
            self.splash_of_color(self.competency_assessment_frame, color='light grey', timeout=0)
            self.disable_buttons()
            labels = [self.label_6, self.label_7, self.label_8, self.label_15, self.label_16]
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
            self.mission_mode.state = ControlModeState.planning
            self.splash_of_color(self.competency_assessment_frame, color='light grey', timeout=0)
        except Exception as e:
            traceback.print_exc()

    def update_et_goa(self):
        try:
            if (self.control_mode.state == ControlModeState.drive
                    and self.etgoa.has_data() and self.condition == self.COND_ETGOA):
                t = (datetime.now() - self.time_start).total_seconds()
                self.etgoa.set_start_time(t)  # TODO delete when done
                px, py, pv = self.position.x, self.position.y, self.velocity
                si = self.etgoa.get_si(px, py, pv, t)
                self.mqa = si
                print(self.mqa)
            if self.control_mode.state == ControlModeState.stopped:
                self.etgoa.forget_start_time()
        except Exception as e:
            traceback.print_exc()

    def disable_buttons(self):
        try:
            self.drive_mode_button.setDisabled(True)
            self.poi_selection.setDisabled(True)
            self.stop_mode_button.setDisabled(True)
            self.accept_poi_order_button.setDisabled(True)
        except Exception as e:
            traceback.print_exc()

    def enable_buttons(self):
        try:
            self.drive_mode_button.setEnabled(True)
            self.poi_selection.setEnabled(True)
            self.stop_mode_button.setEnabled(True)
            self.accept_poi_order_button.setEnabled(True)
        except Exception as e:
            traceback.print_exc()

    def splash_of_color(self, obj, color='green', timeout=500):
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
