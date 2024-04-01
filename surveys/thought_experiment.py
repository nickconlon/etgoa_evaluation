# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'thought_experiment.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1319, 839)
        self.frame = QtWidgets.QFrame(Form)
        self.frame.setGeometry(QtCore.QRect(10, 20, 1291, 801))
        self.frame.setFrameShape(QtWidgets.QFrame.Box)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.label = QtWidgets.QLabel(self.frame)
        self.label.setGeometry(QtCore.QRect(20, 0, 1261, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label.setFont(font)
        self.label.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label.setWordWrap(True)
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.frame)
        self.label_2.setGeometry(QtCore.QRect(20, 40, 1251, 231))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_2.setFont(font)
        self.label_2.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_2.setWordWrap(True)
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.frame)
        self.label_3.setGeometry(QtCore.QRect(60, 270, 1061, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_3.setFont(font)
        self.label_3.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_3.setWordWrap(True)
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.frame)
        self.label_4.setGeometry(QtCore.QRect(60, 450, 1061, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_4.setFont(font)
        self.label_4.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_4.setWordWrap(True)
        self.label_4.setObjectName("label_4")
        self.q1 = QtWidgets.QTextEdit(self.frame)
        self.q1.setGeometry(QtCore.QRect(60, 330, 1061, 111))
        self.q1.setObjectName("q1")
        self.q2 = QtWidgets.QTextEdit(self.frame)
        self.q2.setGeometry(QtCore.QRect(60, 500, 1061, 111))
        self.q2.setObjectName("q2")
        self.label_5 = QtWidgets.QLabel(self.frame)
        self.label_5.setGeometry(QtCore.QRect(60, 620, 1061, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_5.setFont(font)
        self.label_5.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_5.setWordWrap(True)
        self.label_5.setObjectName("label_5")
        self.q3 = QtWidgets.QTextEdit(self.frame)
        self.q3.setGeometry(QtCore.QRect(60, 670, 1061, 111))
        self.q3.setObjectName("q3")
        self.submit_button = QtWidgets.QPushButton(self.frame)
        self.submit_button.setGeometry(QtCore.QRect(1140, 720, 131, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.submit_button.setFont(font)
        self.submit_button.setObjectName("submit_button")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label.setText(_translate("Form", "Please read the following scenario and answer the questions, and click Submit when you are done:"))
        self.label_2.setText(_translate("Form", "<html><head/><body><p>Imagine that are the suprevisor of a robotic exploration mission, and tasked to explore and take measurements of an area, we\'ll call it Area A. You assign the robot a task, &quot;drive to Area A, and look for deposits of minerals, try to avoid hazardous craters, and return within 2 hours.&quot; Here, the mission goals are (1) to drive to Area A, (2) to search for minerals, (3) avoid craters, and (4) to complete the mission with 2 hours. You are charged with deciding if the robot should attempt this task. You must balance the value in exploring Area A with the potential costs of failing the mission (e.g., damage to the robot, etc.). </p><p>To help with your decision making, the robot can report to you how confident it is in achieving each of those four mission goals. Here, the robot reports back: &quot;<span style=\" font-style:italic;\">Highly Likely</span> to driving to Area A, <span style=\" font-style:italic;\">Likely</span> to find the minerals, <span style=\" font-style:italic;\">Even Chance</span> to avoid hazardous craters, and <span style=\" font-style:italic;\">Unlikely</span> to complete the mission within 2 hours.&quot; Given this scenario, please answer the following three questions:</p></body></html>"))
        self.label_3.setText(_translate("Form", "(1) Would, and how would, you use the robot\'s report of confidence (Highly Likely, Likely, Even Chance, Unlikely, Highly Unlikely etc.) in meeting the mission objectives? "))
        self.label_4.setText(_translate("Form", "(2) What additional information or reasoning would you want the robot to report along with the confidence?"))
        self.label_5.setText(_translate("Form", "(3) Why would that information be useful to you as the robot\'s supervisor? How would you act on it?"))
        self.submit_button.setText(_translate("Form", "Submit"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
