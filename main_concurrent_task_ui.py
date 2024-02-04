import sys
import time
import numpy as np
import traceback
import os
from datetime import datetime
import argparse

from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow
import PyQt5.QtGui as QtGui
import PyQt5.QtCore as QtCore
import qdarktheme

from concurrent_task.concurrent_ui import Ui_MainWindow
from analysis.data_recorder import ConcurrentTaskRecorder
from concurrent_task.concurrent_task_helper import MarsMap, metals, get_image, read_next
from base_interface.settings import Settings


class ConcurrentTask(QMainWindow, Ui_MainWindow):
    def __init__(self, settings_path):
        QMainWindow.__init__(self)
        self.setupUi(self)

        settings = Settings('./settings.yaml')
        settings.read()
        self.map = MarsMap('./imgs/mars_map_cropped.png')
        self.set_legend('./imgs/legend.png')
        fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_secondary_{}.csv'.format(settings.condition)
        self.recorder = ConcurrentTaskRecorder(os.path.join(settings.record_path, fname))
        self.request_time = None
        self.mineral_of_interest = None
        self.minerals2means = None
        self.next_identification(False)
        self.next_mineral()
        self.submit_button.clicked.connect(self.submit_button_callback)
        self.start_task = False
        self.next_idx = 1
        self.total = 5

    def set_legend(self, imgpath):
        img = get_image(imgpath)
        height, width, channel = img.shape
        bytesPerLine = 3 * width
        qImg = QtGui.QImage(img, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
        self.key_label.setPixmap(QtGui.QPixmap(qImg))

    def make_timer(self, timeout):
        """

        :param timeout:
        :return:
        """
        try:
            self.next_mineral()
            QtCore.QTimer.singleShot(timeout, lambda: self.next_identification(True))
        except Exception as e:
            traceback.print_exc()

    def check_response(self, lat_y, lon_x):
        """
        :param lat_y:
        :param lon_x:            #for mean_std in mean_stds:

        :return:
        """
        d = -1
        try:
            mineral = self.mineral_of_interest
            mean_std = self.minerals2means[mineral]
            x = self.map.mapped_x[lon_x]
            y = self.map.mapped_y[lat_y]

            mu_y, mu_x, std = mean_std
            d = np.linalg.norm(np.asarray([x, y])-np.asarray([mu_x, mu_y]))
        except Exception as e:
            traceback.print_exc()
        return d

    def next_mineral(self):
        """
        
        :return:
        """
        try:
            self.mineral_of_interest = np.random.choice(a=metals)
        except Exception as e:
            traceback.print_exc()

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

            lat = self.latitude_input.text().strip()
            lon = self.longitude_input.text().strip()

            try:
                lat = int(float(lat))
                lon = int(float(lon))
                distance = self.check_response(lat, lon)
                self.splash_of_color(self.frame)
                dt = time.time() - self.request_time
                self.recorder.add_row(time.time(), distance, dt, self.mineral_of_interest)
                self.latitude_input.setText("")
                self.longitude_input.setText("")
                self.mineral_of_interest = None
                self.request_time = None
                self.next_identification(False)
                self.make_timer(int(np.random.uniform(3000, 5000)))
            except Exception as e:
                traceback.print_exc()

                print('bad cast')

        except Exception as e:
            traceback.print_exc()

    def next_identification(self, include_minerals):
        p = './data/concurrent_{}.{}'
        try:
            if include_minerals:
                self.splash_of_color(self.frame, color='red', timeout=1000)
                img, self.minerals2means, self.mineral_of_interest = read_next(p.format(self.next_idx, 'yaml'),
                                                          p.format(self.next_idx, 'png'))
                self.request_time = time.time()
                text = "Please identify one site with {}".format(self.mineral_of_interest)
                print("trying to find ", self.minerals2means[self.mineral_of_interest])
                print('index ', self.next_idx)
                self.next_idx = (self.next_idx + 1) % self.total
                self.next_idx = self.next_idx + 1 if self.next_idx == 0 else self.next_idx
            else:
                img, _, _ = read_next(p.format(0, 'yaml'), p.format(0, 'png'))
                text = ""
            img = np.array(img)
            height, width, channel = img.shape
            bytesPerLine = 3 * width
            qImg = QtGui.QImage(img, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
            self.map_label.setPixmap(QtGui.QPixmap(qImg))
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
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--settings",
                        help='path to settings file',
                        default="./settings.yaml")
    args = parser.parse_args()

    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = ConcurrentTask(args.settings)
    MainWindow.show()
    app.exec_()
