import sys

import numpy as np
from PyQt5.QtWidgets import QMainWindow, QApplication
import PyQt5.QtCore as QtCore
import qdarktheme

from anomaly import Ui_MainWindow
import rospy
from std_msgs.msg import String, Float32


class anomaly_ui(QMainWindow, Ui_MainWindow):
    VELOCITY_CHANGE_IN_SITU = 0
    POWER_CHANGE_IN_SITU = 1
    OBSTACLES_A_PRIORI = 3
    IMG_QUALITY_A_PRIORI = 4

    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)
        rospy.init_node('anomalies', anonymous=True)

        self.curr_bat_rate = 1  # 1/s battery discharge rate
        self.curr_vel_rate = 100
        self.default_speed = 0.25
        self.default_battery = 100

        self.battery_slider.setMinimum(0)
        self.battery_slider.setMaximum(200)
        self.velocity_slider.setMinimum(0)
        self.velocity_slider.setMaximum(500)
        self.velocity_slider.setSingleStep(10)
        self.battery_slider.setSingleStep(10)
        self.velocity_slider.valueChanged.connect(self.update_velocity_rate)
        self.battery_slider.valueChanged.connect(self.update_battery_rate)
        self.battery_update_button.clicked.connect(self.send_battery_update)
        self.velocity_update_button.clicked.connect(self.send_velocity_update)
        self.battery_slider.setValue(self.curr_bat_rate)
        self.velocity_slider.setValue(self.curr_vel_rate)

        self.waypoint_speed_pub = rospy.Publisher('/{}/control'.format('tars'), Float32,
                                                  queue_size=10)
        self.battery_pub = rospy.Publisher('/{}/battery'.format('tars'), Float32, queue_size=10)

        self.telemetry_updater = QtCore.QTimer()
        self.telemetry_updater.timeout.connect(self.update_battery)
        self.telemetry_updater.setInterval(1000)
        self.telemetry_updater.start()

    def update_battery(self):
        self.default_battery -= self.curr_bat_rate
        msg = Float32()
        msg.data = self.default_battery
        self.battery_pub.publish(msg)
        print('sending battery update: {}'.format(self.default_battery))

    def update_battery_rate(self):
        self.curr_bat_rate = self.battery_slider.value()
        self.battery_lcd.display(str(self.curr_bat_rate))

    def update_velocity_rate(self):
        self.curr_vel_rate = self.velocity_slider.value()
        self.velocity_lcd.display(str(self.curr_vel_rate))

    def send_battery_update(self):
        print("sending battery update {}%".format(self.curr_bat_rate))
        val = self.curr_bat_rate / 100

    def send_velocity_update(self):
        new_speed = self.default_speed * self.curr_vel_rate / 100
        new_speed = np.maximum(0.01, new_speed)
        print('setting speed to ', new_speed)
        msg = Float32()
        msg.data = new_speed
        self.waypoint_speed_pub.publish(msg)


if __name__ == '__main__':
    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = anomaly_ui()
    MainWindow.show()
    app.exec_()
