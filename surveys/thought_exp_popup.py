import traceback
from PyQt5 import QtWidgets, QtGui
import time
import numpy as np
import PyQt5.QtCore as QtCore

from surveys.thought_experiment import Ui_Form


class Popup(QtWidgets.QDialog, Ui_Form):
    def __init__(self):
        QtWidgets.QDialog.__init__(self)
        self.setupUi(self)
        self.label_6.setPixmap(QtGui.QPixmap("./imgs/famsec.png"))
        self.results = []
        self.submit_button.clicked.connect(self.capture_response)

    def save(self):
        print(self.results)
        return self.results

    def capture_response(self):
        self.q1.toPlainText()
        self.q2.toPlainText()
        self.q3.toPlainText()
        self.results.append(self.q1.toPlainText())
        self.results.append(self.q2.toPlainText())
        self.results.append(self.q3.toPlainText())

        time.sleep(0.25)
        self.done(0)

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


def run_survey_popup_online():
    """

    :return:
    """
    responses = None
    try:
        ui = Popup()
        ui.exec_()
        responses = ui.save()
    except Exception as e:
        traceback.print_exc()
    return responses


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    resp = run_survey_popup_online()
    print(resp)
