import traceback
from PyQt5 import QtWidgets
import time
import numpy as np
from PyQt5.QtWidgets import QMessageBox
from datetime import datetime

from surveys.usability_survey import Ui_Trust_Survey
from analysis.data_recorder import SurveyRecorder

class Popup(QtWidgets.QDialog, Ui_Trust_Survey):
    def __init__(self, prompt, number, total):
        QtWidgets.QDialog.__init__(self)
        self.setupUi(self)
        self.results = ''
        self.prompt_text.setText(prompt)
        self.label_2.setText(self.label_2.text() + " {}/{}".format(number, total))
        self.radio_1.clicked.connect(lambda: self.capture_response(self.radio_1))
        self.radio_2.clicked.connect(lambda: self.capture_response(self.radio_2))
        self.radio_3.clicked.connect(lambda: self.capture_response(self.radio_3))
        self.radio_4.clicked.connect(lambda: self.capture_response(self.radio_4))
        self.radio_5.clicked.connect(lambda: self.capture_response(self.radio_5))

    def save(self):
        return self.results

    def capture_response(self, button):
        buttons = [self.radio_1, self.radio_2, self.radio_3, self.radio_4, self.radio_5]
        for b in buttons:
            if b != button:
                b.setChecked(False)
        self.results = button.objectName()
        time.sleep(0.25)
        self.done(0)


def run_survey_popup_online():
    """
    To calculate the SUS score, first sum the score contributions from each item. Each item's
    score contribution will range from 0 to 4. For items 1,3,5,7,and 9 the score contribution is the
    scale position minus 1. For items 2,4,6,8 and 10, the contribution is 5 minus the scale position.
    Multiply the sum of the scores by 2.5 to obtain the overall value of SU.

    SUS scores have a range of 0 to 100

    :return:
    """
    responses = []
    score = -1
    try:
        prompts = ['I think that I would like to use this system frequently',
                   'I found the system unnecessarily complex',
                   'I thought the system was easy to use',
                   'I think that I would need the support of a technical\nperson to be able to use this system',
                   'I found the various functions in the system were well\nintegrated',
                   'I thought there was too much inconsistency in the system',
                   'I would imagine that most people would learn to use this\nsystem very quickly',
                   'I found the system very cumbersome to use',
                   'I felt very confident using the system',
                   'I need to learn a lot of things before I could get going on\nthis system',
                   ]

        responses = []
        scores = []
        for num, p in enumerate(prompts):
            ui = Popup(p, num+1, 10)
            ui.exec_()
            resp = ui.save()
            resp = resp.replace('radio_', '')
            responses.append(int(resp))
            if num % 2 == 0:
                resp = int(resp) - 1
            else:
                resp = 5-int(resp)
            scores.append(resp)
        score = np.sum(responses) * 2.5
        print("Scored: ", score)
    except Exception as e:
        traceback.print_exc()
    return responses, score


def run_survey_popup_offline(survey_number):
    msgBox = QMessageBox()
    msgBox.setIcon(QMessageBox.Information)
    msgBox.setText("Please complete questionnaire {}.\n\nPress Okay when done".format(survey_number))
    msgBox.setWindowTitle("Questionnaire prompt")
    msgBox.setStandardButtons(QMessageBox.Ok)
    returnValue = msgBox.exec()
    if returnValue == QMessageBox.Ok:
        print('OK clicked')

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    resp, score = run_survey_popup_online()
    sys.exit(app.exec_())
