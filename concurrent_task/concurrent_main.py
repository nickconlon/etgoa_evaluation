import sys
import time
import numpy as np
import traceback

from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow
import PyQt5.QtGui as QtGui
import PyQt5.QtCore as QtCore
import qdarktheme
from concurrent_task.concurrent_ui import Ui_MainWindow

from concurrent_task_images_generator import MarsMap, metals


class ConcurrentTask(QMainWindow, Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)
        self.map = MarsMap('./mars_map_cropped.png')
        self.request_time = None
        self.mineral_of_interest = None
        self.minerals2means = None
        self.request_new_identification(False)
        self.next_mineral()
        self.submit_button.clicked.connect(self.submit_button_callback)
        self.start_task = False

    def make_timer(self, timeout):
        self.next_mineral()
        QtCore.QTimer.singleShot(timeout, lambda: self.request_new_identification(True))

    def check_response(self, lat_y, lon_x):
        """
        :param lat_y:
        :param lon_x:
        :return:
        """
        mineral = self.mineral_of_interest
        mean_stds = self.minerals2means[mineral]
        x = self.map.mapped_x[lon_x]
        y = self.map.mapped_y[lat_y]
        print(y, x)
        print(mean_stds)
        for mean_std in mean_stds:
            mu_y, mu_x, std = mean_std
            print(abs(mu_x-x) <= 2*std)
            print(abs(mu_y-y) <= 2*std)
        return True

    def next_mineral(self):
        self.mineral_of_interest = np.random.choice(a=metals)

    def submit_button_callback(self):
        """
        Mission Control receives a new identification from the user

        :return:
        """
        try:
            if not self.start_task:
                self.start_task = True
                self.make_timer(3000)
                return 0

            lat = self.latitude_input.toPlainText().strip()
            lon = self.longitude_input.toPlainText().strip()
            print('received:')
            print('   latitude:  {}'.format(lat))
            print('   longitude: {}'.format(lon))

            try:
                lat = int(lat)
                lon = int(lon)
                correct = self.check_response(lat, lon)
                if correct:
                    self.mineral_of_interest = None
                    self.request_new_identification(False)
                    self.splash_of_color(self.frame)
                    print('correct in {:2f} seconds'.format(time.time()-self.request_time))
                    self.request_time = None
                    self.make_timer(int(np.random.uniform(3000, 5000)))
            except Exception as e:
                traceback.print_exc()

                print('bad cast')

        except Exception as e:
            traceback.print_exc()

    def request_new_identification(self, include_minerals):
        """
        Mission Control requests a new identification from the user
        :return:
        """
        try:
            if include_minerals:
                self.splash_of_color(self.frame, color='red', timeout=1000)
            img, self.minerals2means = self.map.make_task_instance(include_minerals, False)
            height, width, channel = img.shape
            bytesPerLine = 3 * width
            qImg = QtGui.QImage(img, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.map_label.setPixmap(QtGui.QPixmap(qImg))
            if self.mineral_of_interest is not None:
                self.request_time = time.time()
                text = "Please identify one site with {}".format(self.mineral_of_interest)
                print("trying to find ", self.minerals2means[self.mineral_of_interest])
            else:
                text = ""
            self.mission_control_text.setText(text)
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
    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = ConcurrentTask()
    MainWindow.show()
    app.exec_()
