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
from surveys.demographics_survey_popup import run_survey_popup_online as run_demographics
from surveys.decision_making_popup import run_survey_popup_online as run_decisions
from base_interface.settings import Settings
from surveys.thought_exp_popup import run_survey_popup_online as run_thoughts
from analysis.data_recorder import UsabilityRecorder, TrustRecorder, DemographicsRecorder, QuestionsRecorder

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--settings",
                        default='./scenarios/settings.yaml',
                        help='path to settings file')
    parser.add_argument("-t", "--type",
                        help='type of survey: usability or trust')
    parser.add_argument('-c', '--condition',
                        help='condition')
    args = parser.parse_args()

    settings = Settings(args.settings)
    settings.read()

    todo = []
    label = 'TODO'
    if args.type == 'before' or args.type == 'mdrs_before':
        todo = ['demographics', 'thoughts', 'trust']
        label = 'BEFORE'
    if args.type == 'after':
        todo = ['trust', 'usability', 'demographics']
        label = 'AFTER'
    if args.type == 'mdrs_after':
        todo = ['trust', 'usability', 'decisions']
        label = 'AFTER'
    if args.type == 'decisions':
        todo = ['decisions']
    if args.type == 'thoughts':
        todo = ['thoughts']

    fname = ''
    available_surveys = []
    available_recorders = []

    print('Starting survey for {}'.format(args.type))
    for item in todo:
        if item == 'trust':
            fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_trust_survey_{}_{}.csv'.format(label, args.condition)
            available_surveys.append(run_trust)
            available_recorders.append(TrustRecorder(os.path.join(settings.record_path, fname)))

        if item == 'usability':
            fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_usability_survey_{}.csv'.format(args.condition)
            available_surveys.append(run_usability)
            available_recorders.append(UsabilityRecorder(os.path.join(settings.record_path, fname)))

        if item == 'decisions':
            fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_decisions_survey.csv'
            available_surveys.append(run_decisions)
            available_recorders.append(QuestionsRecorder(os.path.join(settings.record_path, fname)))

        if item == 'thoughts':
            fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_thoughts_survey.csv'
            available_surveys.append(run_thoughts)
            available_recorders.append(QuestionsRecorder(os.path.join(settings.record_path, fname)))

        if item == 'demographics':
            fname = datetime.now().strftime("%Y%m%d_%H%M%S") + '_demographics_survey.csv'
            available_surveys.append(run_demographics)
            available_recorders.append(DemographicsRecorder(os.path.join(settings.record_path, fname)))

    qdarktheme.enable_hi_dpi()
    app = QtWidgets.QApplication(sys.argv)
    qdarktheme.setup_theme()
    for survey, recorder in zip(available_surveys, available_recorders):
        t1 = time.time()
        resp = survey()
        t2 = time.time()
        recorder.record(resp, abs(t2 - t1))
        time.sleep(1)
