import traceback
from PyQt5 import QtWidgets
import time

from base_interface.survey import Ui_Trust_Survey


class Popup(QtWidgets.QDialog, Ui_Trust_Survey):
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


def startup():
    responses = []
    try:
        prompts = ['Dependable', 'Reliable', 'Unresponsive', 'Predictable',
                   'Act consistently', 'Malfunction', 'Require frequent maintenance', 'Have errors',
                   'Provide feedback', 'Meet the needs of the mission/task',
                   'Provide appropriate information', 'Communicate with people',
                   'Perform exactly as instructed', 'Follow directions']
        sections = ['What % of the time will this robot be...'] * 4 + [
            'What % of the time will this robot...'] * 10
        responses = []
        for num, (p, s) in enumerate(zip(prompts, sections)):
            ui = Popup(p, s, num+1, 14)
            ui.exec_()
            resp = ui.save()
            resp = resp.replace('radio_', '')
            responses.append(resp)
    except Exception as e:
        traceback.print_exc()
    return responses


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    Trust_Survey = QtWidgets.QDialog()
    ui = Popup()
    # ui.setupUi(Trust_Survey)
    ui.show()
    sys.exit(app.exec_())
