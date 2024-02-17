import traceback
from PyQt5 import QtWidgets
import time
import numpy as np
from PyQt5.QtWidgets import QMessageBox

from surveys.trust_survey import Ui_Survey


class Popup(QtWidgets.QDialog, Ui_Survey):
    def __init__(self, prompt, section, number, total):
        QtWidgets.QDialog.__init__(self)
        self.setupUi(self)
        self.results = ''
        self.prompt_text.setText(prompt)
        self.section_text.setText(section)
        self.label_2.setText(self.label_2.text() + " {}/{}".format(number, total))
        self.radio_0.clicked.connect(lambda: self.capture_response(self.radio_0))
        self.radio_10.clicked.connect(lambda: self.capture_response(self.radio_10))
        self.radio_20.clicked.connect(lambda: self.capture_response(self.radio_20))
        self.radio_30.clicked.connect(lambda: self.capture_response(self.radio_30))
        self.radio_40.clicked.connect(lambda: self.capture_response(self.radio_40))
        self.radio_50.clicked.connect(lambda: self.capture_response(self.radio_50))
        self.radio_60.clicked.connect(lambda: self.capture_response(self.radio_60))
        self.radio_70.clicked.connect(lambda: self.capture_response(self.radio_70))
        self.radio_80.clicked.connect(lambda: self.capture_response(self.radio_80))
        self.radio_90.clicked.connect(lambda: self.capture_response(self.radio_90))
        self.radio_100.clicked.connect(lambda: self.capture_response(self.radio_100))

    def save(self):
        return self.results

    def capture_response(self, button):
        buttons = [self.radio_0, self.radio_20, self.radio_30, self.radio_40, self.radio_50,
                   self.radio_60, self.radio_70, self.radio_80, self.radio_90, self.radio_100]
        for b in buttons:
            if b != button:
                b.setChecked(False)
        self.results = button.objectName()
        time.sleep(0.25)
        self.done(0)


def run_survey_popup_online():
    responses = []
    score = -1
    try:
        prompts = ['Dependable', 'Reliable', 'Unresponsive', 'Predictable',
                   'Act consistently', 'Malfunction', 'Require frequent maintenance', 'Have errors',
                   'Provide feedback', 'Meet the needs of the mission/task',
                   'Provide appropriate information', 'Communicate with people',
                   'Perform exactly as instructed', 'Follow directions']

        reverse_codes = ['Unresponsive', 'Malfunction', 'Require frequent maintenance', 'Have errors']

        sections = (['What % of the time will this robot be...'] * 4
                    + ['What % of the time will this robot...'] * 10)
        responses = []
        for num, (p, s) in enumerate(zip(prompts, sections)):
            ui = Popup(p, s, num+1, 14)
            ui.exec_()
            resp = ui.save()
            resp = resp.replace('radio_', '')
            if p in reverse_codes:
                resp = 100 - int(resp)
            else:
                resp = int(resp)
            responses.append(resp)
        score = np.sum(responses)/14
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
