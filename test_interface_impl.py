import sys
from PyQt5.QtWidgets import QApplication
import PyQt5.QtCore as QtCore
import PyQt5.QtGui as QtGui
import numpy as np
import qdarktheme

from base_interface.base_interface import BaseInterface


class InterfaceImpl(BaseInterface):
    def __init__(self):
        BaseInterface.__init__(self)
        self.img_path = 'base_interface/mission_area.png'
        self.label_21.setPixmap(QtGui.QPixmap(self.img_path))
        self.poi_selection.addItems(["",
                                     "A->B->C->D",
                                     "A->B->C",
                                     "A->B->D",
                                     "A->C->D"])
        self.mission_control_help_selector.addItems(self.mission_control.help_requests.keys())
        self.num_backup_batteries = 5
        self.robot_battery_slider.setMaximum(self.num_backup_batteries)
        self.ui_connected_indicator.setStyleSheet('background-color: green')
        self.request_mission_control_help_button.clicked.connect(self.test_mission_control_questions)
        self.test_position_update()
        self.select_poi_order_button.clicked.connect(self.test_competency_assessment)

    def test_mission_control_questions(self):
        help_with = self.mission_control_help_selector.currentText()
        item = self.mission_control.help_requests[help_with]
        txt = self.mission_control.get_response(item)
        self.update_mission_control_text(txt)

    def test_position_update(self):
        self.position = [40.010385, -105.244390, 0.0001]
        self.heading = 337
        self.velocity = 0.5
        self.battery_remaining = 90

    def test_competency_assessment(self):
        # TODO ET-GOA: "because <unexpected state>"
        if self.poi_selection.currentText() != "":
            self.update_competency_assessment()


if __name__ == '__main__':
    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = InterfaceImpl()
    MainWindow.show()
    app.exec_()
