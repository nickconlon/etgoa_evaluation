import sys
from PyQt5.QtWidgets import QApplication
import PyQt5.QtCore as QtCore
import numpy as np
import qdarktheme
import argparse

from base_interface.mdrs_base_interface import BaseInterface
from famsec import goa


class InterfaceImpl(BaseInterface):
    def __init__(self, settings_path):
        BaseInterface.__init__(self, settings_path)

        self.position.x = 10
        self.position.y = 10

        self.act_position_x = 40
        self.act_position_y = -20

        self.poi_selection.currentTextChanged.connect(self.test_competency_assessment)
        self.ui_connected = True
        self.robot_connected = True
        self.gps_connected = True
        self.sensor1_connected = True
        self.sensor2_connected = True
        self.left_button_2.clicked.connect(self.left)
        self.right_button_2.clicked.connect(self.right)

        self.telemetry_test = QtCore.QTimer()
        self.telemetry_test.timeout.connect(self.test_position_update)
        self.telemetry_test.setInterval(100)
        self.telemetry_test.start()

    def test_position_update(self):
        self.position.x = self.act_position_x + np.random.normal(loc=0, scale=0.05) - self.zero[0]
        self.position.y = self.act_position_y + np.random.normal(loc=0, scale=0.05) - self.zero[1]
        self.heading = 359
        self.velocity = 0.0

    def test_competency_assessment(self):
        outcomes = np.random.random(size=5)
        labels = [self.objective_1_assmt, self.objective_2_assmt, self.objective_3_assmt, self.objective_4_assmt]
        for outcome, label in zip(outcomes, labels):
            label.setStyleSheet('background-color: {}; color: black'.format(goa.semantic_label_color(outcome)))
            label.setText("{}".format(goa.semantic_label_text(outcome)))
        self.splash_of_color(self.competency_assessment_frame)

    def left(self):
        self.heading = (self.heading - 5) % 360

    def right(self):
        self.heading = (self.heading + 5) % 360


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--settings",
                        help='path to settings file',
                        default="./scenarios/settings_mdrs.yaml")
    args = parser.parse_args()

    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = InterfaceImpl(args.settings)
    MainWindow.show()
    app.exec_()
