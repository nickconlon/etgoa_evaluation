import sys
from PyQt5.QtWidgets import QApplication
import PyQt5.QtCore as QtCore
import numpy as np
import qdarktheme
from PIL import Image

from base_interface.base_interface import BaseInterface
from famsec import goa


class InterfaceImpl(BaseInterface):
    def __init__(self):
        BaseInterface.__init__(self)
        self.update_map_display(np.asarray(Image.open(self.img_path)))
        self.poi_selection.addItems(["Select POI", "POI A", "POI B", "POI C", "POI D"])
        self.num_backup_batteries = 5
        self.robot_battery_slider.setMaximum(self.num_backup_batteries)
        self.test_position_update()
        self.poi_selection.currentTextChanged.connect(self.test_competency_assessment)
        self.ui_connected = True
        self.robot_connected = True
        self.gps_connected = True
        self.sensor1_connected = True
        self.sensor2_connected = True

        self.pupdated = QtCore.QTimer()
        self.pupdated.timeout.connect(self.test_position_change)
        self.pupdated.setInterval(500)
        self.pupdated.start()

    def test_position_update(self):
        self.position.x = 1
        self.heading = 337
        self.velocity = 0.5
        self.battery_remaining = 90

    def test_competency_assessment(self):
        outcomes = np.random.random(size=5)
        labels = [self.label_6, self.label_7, self.label_8, self.label_15, self.label_16]
        for outcome, label in zip(outcomes, labels):
            label.setStyleSheet('background-color: {}; color: black'.format(goa.semantic_label_color(outcome)))
            label.setText("{}".format(goa.semantic_label_text(outcome)))
        self.splash_of_color(self.competency_assessment_frame)

    def test_position_change(self):
        self.position.x += 0.1
        self.position.y += 0.1


if __name__ == '__main__':
    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = InterfaceImpl()
    MainWindow.show()
    app.exec_()
