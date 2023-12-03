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
        self.data_path = './data'
        self.img_path = 'base_interface/mission_area.png'
        self.label_21.setPixmap(QtGui.QPixmap(self.img_path))
        self.poi_selection.addItems(["",
                                     "A->B->C->D",
                                     "A->B->C",
                                     "A->B->D",
                                     "A->C->D"])
        self.mission_control_help_selector.addItems(["",
                                                     "Diagnosing Everything",
                                                     "Diagnosing Position Issues",
                                                     "Diagnosing Battery Issues",
                                                     "Diagnosing Poser Issues"])
        self.num_backup_batteries = 5
        self.robot_battery_slider.setMaximum(self.num_backup_batteries)
        self.ui_connected_indicator.setStyleSheet('background-color: green')

    def ros_position_callback(self, msg):
        """
        Receive position data from the robot
        :param msg:
        :return:
        """
        pass

    def ros_image_callback(self, msg):
        """
        Receive images from the robot
        :param msg:
        :return:
        """
        pass

    def ros_automatic_control_command(self, command):
        """
        Send automatic control cmd_vel messages to the robot
        :param command:
        :return:
        """
        pass

    def ros_manual_control_command(self, command):
        """
        Send manual control cmd_vel messages to the robot
        :param command:
        :return:
        """
        pass



if __name__ == '__main__':
    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = InterfaceImpl()
    MainWindow.show()
    app.exec_()
