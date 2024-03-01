# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'demographics_survey.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Survey(object):
    def setupUi(self, Survey):
        Survey.setObjectName("Survey")
        Survey.setWindowModality(QtCore.Qt.ApplicationModal)
        Survey.resize(874, 504)
        self.label_2 = QtWidgets.QLabel(Survey)
        self.label_2.setGeometry(QtCore.QRect(30, 10, 571, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.frame_6 = QtWidgets.QFrame(Survey)
        self.frame_6.setGeometry(QtCore.QRect(20, 230, 781, 81))
        self.frame_6.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.prompt_text = QtWidgets.QLabel(self.frame_6)
        self.prompt_text.setGeometry(QtCore.QRect(10, 10, 461, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.prompt_text.setFont(font)
        self.prompt_text.setFrameShape(QtWidgets.QFrame.Box)
        self.prompt_text.setObjectName("prompt_text")
        self.robotics_combo = QtWidgets.QComboBox(self.frame_6)
        self.robotics_combo.setGeometry(QtCore.QRect(490, 10, 281, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.robotics_combo.setFont(font)
        self.robotics_combo.setObjectName("robotics_combo")
        self.frame_7 = QtWidgets.QFrame(Survey)
        self.frame_7.setGeometry(QtCore.QRect(20, 320, 781, 81))
        self.frame_7.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_7.setObjectName("frame_7")
        self.prompt_text_2 = QtWidgets.QLabel(self.frame_7)
        self.prompt_text_2.setGeometry(QtCore.QRect(10, 10, 461, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.prompt_text_2.setFont(font)
        self.prompt_text_2.setFrameShape(QtWidgets.QFrame.Box)
        self.prompt_text_2.setObjectName("prompt_text_2")
        self.games_combo = QtWidgets.QComboBox(self.frame_7)
        self.games_combo.setGeometry(QtCore.QRect(490, 10, 281, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.games_combo.setFont(font)
        self.games_combo.setObjectName("games_combo")
        self.frame_8 = QtWidgets.QFrame(Survey)
        self.frame_8.setGeometry(QtCore.QRect(20, 50, 781, 81))
        self.frame_8.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_8.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_8.setObjectName("frame_8")
        self.prompt_text_3 = QtWidgets.QLabel(self.frame_8)
        self.prompt_text_3.setGeometry(QtCore.QRect(10, 10, 461, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.prompt_text_3.setFont(font)
        self.prompt_text_3.setFrameShape(QtWidgets.QFrame.Box)
        self.prompt_text_3.setObjectName("prompt_text_3")
        self.age_combo = QtWidgets.QComboBox(self.frame_8)
        self.age_combo.setGeometry(QtCore.QRect(490, 10, 281, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.age_combo.setFont(font)
        self.age_combo.setObjectName("age_combo")
        self.frame_9 = QtWidgets.QFrame(Survey)
        self.frame_9.setGeometry(QtCore.QRect(20, 140, 781, 81))
        self.frame_9.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_9.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_9.setObjectName("frame_9")
        self.prompt_text_5 = QtWidgets.QLabel(self.frame_9)
        self.prompt_text_5.setGeometry(QtCore.QRect(10, 10, 461, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.prompt_text_5.setFont(font)
        self.prompt_text_5.setFrameShape(QtWidgets.QFrame.Box)
        self.prompt_text_5.setObjectName("prompt_text_5")
        self.gender_combo = QtWidgets.QComboBox(self.frame_9)
        self.gender_combo.setGeometry(QtCore.QRect(490, 10, 281, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.gender_combo.setFont(font)
        self.gender_combo.setObjectName("gender_combo")
        self.submit_button = QtWidgets.QPushButton(Survey)
        self.submit_button.setGeometry(QtCore.QRect(20, 420, 191, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.submit_button.setFont(font)
        self.submit_button.setObjectName("submit_button")

        self.retranslateUi(Survey)
        QtCore.QMetaObject.connectSlotsByName(Survey)

    def retranslateUi(self, Survey):
        _translate = QtCore.QCoreApplication.translate
        Survey.setWindowTitle(_translate("Survey", "Survey"))
        self.label_2.setText(_translate("Survey", "Please answer the following questions"))
        self.prompt_text.setText(_translate("Survey", "How much experience with robotics\n"
"do you have in years?"))
        self.prompt_text_2.setText(_translate("Survey", "How much experience with video games\n"
"do you have in years?"))
        self.prompt_text_3.setText(_translate("Survey", "What is your age in years?"))
        self.prompt_text_5.setText(_translate("Survey", "What is your gender?"))
        self.submit_button.setText(_translate("Survey", "Submit"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Survey = QtWidgets.QDialog()
    ui = Ui_Survey()
    ui.setupUi(Survey)
    Survey.show()
    sys.exit(app.exec_())
