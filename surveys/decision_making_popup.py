import traceback
from PyQt5 import QtWidgets
import time
import numpy as np
import PyQt5.QtCore as QtCore

from surveys.decision_making_survey import Ui_Form


class Popup(QtWidgets.QDialog, Ui_Form):
    def __init__(self):
        QtWidgets.QDialog.__init__(self)
        self.setupUi(self)
        self.results = []
        self.submit_button.clicked.connect(self.capture_response)

    def save(self):
        print(self.results)
        return self.results

    def capture_response(self):
        planning_used = [self.q1_map_check.isChecked(),
                         self.q1_telem_check.isChecked(),
                         self.q1_cam_check.isChecked(),
                         self.q1_assmt_check.isChecked(),
                         self.q1_other_check.isChecked(),
                         self.q1_other_free.toPlainText()]
        execution_used = [self.q2_map_check.isChecked(),
                          self.q2_telem_check.isChecked(),
                          self.q2_cam_check.isChecked(),
                          self.q2_assmt_check.isChecked(),
                          self.q2_other_check.isChecked(),
                          self.q2_other_free.toPlainText()]
        useful = [self.q1_useful.toPlainText(), self.q2_useful.toPlainText()]
        needs = [self.q1_needs.toPlainText(), self.q2_needs.toPlainText()]

        self.results = [*planning_used, *execution_used, *useful, *needs]
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
