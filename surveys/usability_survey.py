# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'usability_survey.ui'
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
        Survey.resize(900, 300)
        self.frame = QtWidgets.QFrame(Survey)
        self.frame.setGeometry(QtCore.QRect(484, 140, 211, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame.setFont(font)
        self.frame.setFrameShape(QtWidgets.QFrame.Box)
        self.frame.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame.setObjectName("frame")
        self.frame_2 = QtWidgets.QFrame(self.frame)
        self.frame_2.setGeometry(QtCore.QRect(0, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_2.setFont(font)
        self.frame_2.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_2.setObjectName("frame_2")
        self.radio_1 = QtWidgets.QRadioButton(self.frame_2)
        self.radio_1.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_1.setFont(font)
        self.radio_1.setText("")
        self.radio_1.setChecked(False)
        self.radio_1.setObjectName("radio_1")
        self.frame_3 = QtWidgets.QFrame(self.frame)
        self.frame_3.setGeometry(QtCore.QRect(40, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_3.setFont(font)
        self.frame_3.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_3.setObjectName("frame_3")
        self.radio_2 = QtWidgets.QRadioButton(self.frame_3)
        self.radio_2.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_2.setFont(font)
        self.radio_2.setText("")
        self.radio_2.setChecked(False)
        self.radio_2.setObjectName("radio_2")
        self.frame_4 = QtWidgets.QFrame(self.frame)
        self.frame_4.setGeometry(QtCore.QRect(80, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_4.setFont(font)
        self.frame_4.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_4.setObjectName("frame_4")
        self.radio_3 = QtWidgets.QRadioButton(self.frame_4)
        self.radio_3.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_3.setFont(font)
        self.radio_3.setText("")
        self.radio_3.setChecked(False)
        self.radio_3.setObjectName("radio_3")
        self.frame_5 = QtWidgets.QFrame(self.frame)
        self.frame_5.setGeometry(QtCore.QRect(120, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_5.setFont(font)
        self.frame_5.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_5.setObjectName("frame_5")
        self.radio_4 = QtWidgets.QRadioButton(self.frame_5)
        self.radio_4.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_4.setFont(font)
        self.radio_4.setText("")
        self.radio_4.setChecked(False)
        self.radio_4.setObjectName("radio_4")
        self.frame_7 = QtWidgets.QFrame(self.frame)
        self.frame_7.setGeometry(QtCore.QRect(160, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_7.setFont(font)
        self.frame_7.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_7.setObjectName("frame_7")
        self.radio_5 = QtWidgets.QRadioButton(self.frame_7)
        self.radio_5.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_5.setFont(font)
        self.radio_5.setText("")
        self.radio_5.setChecked(False)
        self.radio_5.setObjectName("radio_5")
        self.prompt_text = QtWidgets.QLabel(Survey)
        self.prompt_text.setGeometry(QtCore.QRect(24, 120, 461, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.prompt_text.setFont(font)
        self.prompt_text.setFrameShape(QtWidgets.QFrame.Box)
        self.prompt_text.setObjectName("prompt_text")
        self.frame_13 = QtWidgets.QFrame(Survey)
        self.frame_13.setGeometry(QtCore.QRect(484, 90, 211, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_13.setFont(font)
        self.frame_13.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_13.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_13.setObjectName("frame_13")
        self.frame_14 = QtWidgets.QFrame(self.frame_13)
        self.frame_14.setGeometry(QtCore.QRect(0, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_14.setFont(font)
        self.frame_14.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_14.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_14.setObjectName("frame_14")
        self.label = QtWidgets.QLabel(self.frame_14)
        self.label.setGeometry(QtCore.QRect(5, 10, 32, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.frame_15 = QtWidgets.QFrame(self.frame_13)
        self.frame_15.setGeometry(QtCore.QRect(40, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_15.setFont(font)
        self.frame_15.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_15.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_15.setObjectName("frame_15")
        self.label_13 = QtWidgets.QLabel(self.frame_15)
        self.label_13.setGeometry(QtCore.QRect(5, 10, 32, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_13.setFont(font)
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.frame_16 = QtWidgets.QFrame(self.frame_13)
        self.frame_16.setGeometry(QtCore.QRect(80, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_16.setFont(font)
        self.frame_16.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_16.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_16.setObjectName("frame_16")
        self.label_14 = QtWidgets.QLabel(self.frame_16)
        self.label_14.setGeometry(QtCore.QRect(5, 10, 32, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_14.setFont(font)
        self.label_14.setAlignment(QtCore.Qt.AlignCenter)
        self.label_14.setObjectName("label_14")
        self.frame_17 = QtWidgets.QFrame(self.frame_13)
        self.frame_17.setGeometry(QtCore.QRect(120, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_17.setFont(font)
        self.frame_17.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_17.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_17.setObjectName("frame_17")
        self.label_15 = QtWidgets.QLabel(self.frame_17)
        self.label_15.setGeometry(QtCore.QRect(5, 10, 32, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_15.setFont(font)
        self.label_15.setAlignment(QtCore.Qt.AlignCenter)
        self.label_15.setObjectName("label_15")
        self.frame_18 = QtWidgets.QFrame(self.frame_13)
        self.frame_18.setGeometry(QtCore.QRect(160, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_18.setFont(font)
        self.frame_18.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_18.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_18.setObjectName("frame_18")
        self.label_16 = QtWidgets.QLabel(self.frame_18)
        self.label_16.setGeometry(QtCore.QRect(5, 10, 32, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_16.setFont(font)
        self.label_16.setAlignment(QtCore.Qt.AlignCenter)
        self.label_16.setObjectName("label_16")
        self.label_2 = QtWidgets.QLabel(Survey)
        self.label_2.setGeometry(QtCore.QRect(30, 10, 571, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(Survey)
        self.label_3.setGeometry(QtCore.QRect(440, 30, 91, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(Survey)
        self.label_4.setGeometry(QtCore.QRect(650, 30, 101, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")

        self.retranslateUi(Survey)
        QtCore.QMetaObject.connectSlotsByName(Survey)

    def retranslateUi(self, Survey):
        _translate = QtCore.QCoreApplication.translate
        Survey.setWindowTitle(_translate("Survey", "Survey"))
        self.prompt_text.setText(_translate("Survey", "TextLabel"))
        self.label.setText(_translate("Survey", "1"))
        self.label_13.setText(_translate("Survey", "2"))
        self.label_14.setText(_translate("Survey", "3"))
        self.label_15.setText(_translate("Survey", "4"))
        self.label_16.setText(_translate("Survey", "5"))
        self.label_2.setText(_translate("Survey", "Please answer the following question"))
        self.label_3.setText(_translate("Survey", "Strongly\n"
"disagree"))
        self.label_4.setText(_translate("Survey", "Strongly\n"
"agree"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Survey = QtWidgets.QDialog()
    ui = Ui_Survey()
    ui.setupUi(Survey)
    Survey.show()
    sys.exit(app.exec_())
