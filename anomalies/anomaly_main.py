import sys
from PyQt5.QtWidgets import QMainWindow, QApplication
import PyQt5.QtCore as QtCore
import qdarktheme

from anomalies.anomaly import Ui_MainWindow


class anomaly_ui(QMainWindow, Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)
        self.curr_bat_rate = 100
        self.curr_vel_rate = 100

        self.battery_slider.setMinimum(0)
        self.battery_slider.setMaximum(200)
        self.velocity_slider.setMinimum(0)
        self.velocity_slider.setMaximum(200)
        self.velocity_slider.setSingleStep(10)
        self.battery_slider.setSingleStep(10)
        self.velocity_slider.valueChanged.connect(self.update_velocity_rate)
        self.battery_slider.valueChanged.connect(self.update_battery_rate)
        self.battery_update_button.clicked.connect(self.send_battery_update)
        self.velocity_update_button.clicked.connect(self.send_velocity_update)
        self.battery_slider.setValue(self.curr_bat_rate)
        self.velocity_slider.setValue(self.curr_vel_rate)

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
        print("sending velocity update {}%".format(self.curr_vel_rate))
        val = self.curr_vel_rate / 100

if __name__ == '__main__':
    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = anomaly_ui()
    MainWindow.show()
    app.exec_()
