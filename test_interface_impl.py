import sys
from PyQt5.QtWidgets import QApplication
import PyQt5.QtGui as QtGui
import numpy as np
import qdarktheme
from PIL import Image

from base_interface.base_interface import BaseInterface
from famsec import goa


class InterfaceImpl(BaseInterface):
    def __init__(self):
        BaseInterface.__init__(self)
        self.update_map(np.asarray(Image.open(self.img_path)))
        self.poi_selection.addItems(["Select POI", "POI A", "POI B", "POI C", "POI D"])
        self.num_backup_batteries = 5
        self.robot_battery_slider.setMaximum(self.num_backup_batteries)
        self.test_position_update()
        self.poi_selection.currentTextChanged.connect(self.test_competency_assessment)
        self.ui_connected = True

    def test_position_update(self):
        self.position = [40.010385, -105.244390, 0.0001]
        self.heading = 337
        self.velocity = 0.5
        self.battery_remaining = 90

    def test_competency_assessment(self):
        outcomes = np.random.random(size=5)
        labels = [self.label_6, self.label_7, self.label_8, self.label_15, self.label_16]
        colors = ['red', 'yellow', 'green']
        for outcome, label in zip(outcomes, labels):
            c = np.random.choice(colors)
            label.setStyleSheet('background-color: {}; color: black'.format(goa.semantic_label_color(outcome)))
            label.setText("{}".format(goa.semantic_label_text(outcome)))
        self.splash_of_color(self.competency_assessment_frame)


if __name__ == '__main__':
    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = InterfaceImpl()
    MainWindow.show()
    app.exec_()
