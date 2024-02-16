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

        self.test_position_update()
        self.poi_selection.currentTextChanged.connect(self.test_competency_assessment)
        self.ui_connected = True
        self.robot_connected = True
        self.gps_connected = True
        self.sensor1_connected = True
        self.sensor2_connected = True

        #self.pos_updated = QtCore.QTimer()
        #self.pos_updated.timeout.connect(self.test_position_change)
        #self.pos_updated.setInterval(500)
        #self.pos_updated.start()

    def test_position_update(self):
        self.position.x = 2
        self.position.y = 3
        self.heading = 337
        self.velocity = 0.5
        self.battery_level = 90

    def test_competency_assessment(self):
        outcomes = np.random.random(size=5)
        labels = [self.objective_1_assmt, self.objective_2_assmt, self.objective_3_assmt, self.objective_4_assmt, self.objective_5_assmt]
        for outcome, label in zip(outcomes, labels):
            label.setStyleSheet('background-color: {}; color: black'.format(goa.semantic_label_color(outcome)))
            label.setText("{}".format(goa.semantic_label_text(outcome)))
        self.splash_of_color(self.competency_assessment_frame)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--settings",
                        help='path to settings file',
                        default="./scenarios/settings_aspen.yaml")
    args = parser.parse_args()

    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = InterfaceImpl(args.settings)
    MainWindow.show()
    app.exec_()
