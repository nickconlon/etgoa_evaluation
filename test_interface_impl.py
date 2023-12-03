import sys
from PyQt5.QtWidgets import QApplication
import PyQt5.QtCore as QtCore
import numpy as np
import qdarktheme

from base_interface.base_interface import BaseInterface


class InterfaceImpl(BaseInterface):
    def __init__(self):
        BaseInterface.__init__(self)
        self.data_path = './data'
        self.poi_selection.addItems(["",
                                     "A->B->C->D",
                                     "A->B->C",
                                     "A->B->D",
                                     "A->C->D"])
        self.mission_control_help_selector.addItems(["",
                                                     "Diagnosing Everything",
                                                     "Diagnosing Position Issues",
                                                     "Diagnosing Battery Issues",
                                                     "Diagnosing Poser Issues"])
        self.num_backup_batteries = 5
        self.robot_battery_slider.setMaximum(self.num_backup_batteries)
        self.ui_connected_indicator.setStyleSheet('background-color: green')
        self.test_mission_control_questions()

        self.send_mission_control_udpate_button.clicked.connect(
            lambda: QtCore.QTimer.singleShot(1000, self.test_mission_control_questions))
        self.request_mission_control_help_button.clicked.connect(self.test_request_help)

    def test_mission_control_questions(self):
        self.update_mission_control_text(np.random.choice(a=self.mission_control.questions))

    def test_request_help(self):
        issue_type = np.random.choice(a=list(self.mission_control.help_responses.keys()))
        self.mission_control_assistance_prompt.setText(self.mission_control.help_responses[issue_type])

    def run_competency_assessment(self):
        print('running competency assessment')
        self.splash_of_color(self.competency_assessment_frame)
        conf = np.random.choice(a=['Highly Likely', 'Likely', 'Even Chance', 'Unlikely', 'Highly Unlikely'])
        self.famsec_confidence_text.setText(
            'Likelihood of success in survey of POIs {}: {}'.format(self.POIs_selected, conf))
        text = 'XX with YY%\nQQ with XX%\nZZ with TT%'
        self.famsec_reason_text.setText(text)


if __name__ == '__main__':
    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = InterfaceImpl()
    MainWindow.show()
    app.exec_()
