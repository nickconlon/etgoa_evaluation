import traceback
from PyQt5 import QtWidgets
import time
import numpy as np
import sys
import argparse
import os
from datetime import datetime
from PyQt5.QtWidgets import QMessageBox
import qdarktheme

from surveys.trust_survey_popup import run_survey_popup_online as run_trust
from surveys.usability_survey_popup import run_survey_popup_online as run_usability
from base_interface.settings import Settings
from analysis.data_recorder import UsabilityRecorder, TrustRecorder

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--settings",
                        default='./scenarios/settings.yaml',
                        help='path to settings file')
    parser.add_argument("-t", "--type",
                        default='usability',
                        help='type of survey: usability or trust')
    args = parser.parse_args()

    settings = Settings(args.settings)
    settings.read()

    fname = ''
    survey = None

    if args.type == 'trust':
        fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_trust_survey.csv'
        survey = run_trust
        recorder = TrustRecorder(os.path.join(settings.record_path, fname))

    elif args.type == 'usability':
        fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_usability_survey.csv'
        survey = run_usability
        recorder = UsabilityRecorder(os.path.join(settings.record_path, fname))

    qdarktheme.enable_hi_dpi()
    app = QtWidgets.QApplication(sys.argv)
    qdarktheme.setup_theme()
    t1 = time.time()
    resp, score = survey()
    t2 = time.time()
    recorder.record(resp, score, abs(t2 - t1))
