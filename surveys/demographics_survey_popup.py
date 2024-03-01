import traceback
from PyQt5 import QtWidgets
import time
import numpy as np
import PyQt5.QtCore as QtCore

from surveys.demographics_survey import Ui_Survey


class Popup(QtWidgets.QDialog, Ui_Survey):
    def __init__(self):
        QtWidgets.QDialog.__init__(self)
        self.setupUi(self)
        self.results = []

        self.age_combo.addItems(['Please choose a response']+[str(x) for x in np.arange(18, 100)])
        self.gender_combo.addItems(['Please choose a response', 'Female', 'Male', 'Other', 'Prefer not to answer'])
        self.robotics_combo.addItems(['Please choose a response']+[str(x) for x in np.arange(0, 51)])
        self.games_combo.addItems(['Please choose a response']+[str(x) for x in np.arange(0, 51)])

        self.submit_button.clicked.connect(self.capture_response)

    def save(self):
        return self.results

    def capture_response(self):
        age = self.age_combo.currentText()
        gender = self.gender_combo.currentText()
        robotics_exp = self.robotics_combo.currentText()
        gaming_exp = self.games_combo.currentText()
        self.results = [age, gender, robotics_exp, gaming_exp]
        if 'Please choose a response' in self.results:
            self.splash_of_color(self.label_2, color='red')
        else:
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
