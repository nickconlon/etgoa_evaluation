# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'trust_survey.ui'
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
        self.frame.setGeometry(QtCore.QRect(405, 150, 461, 41))
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
        self.radio_0 = QtWidgets.QRadioButton(self.frame_2)
        self.radio_0.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_0.setFont(font)
        self.radio_0.setText("")
        self.radio_0.setChecked(False)
        self.radio_0.setObjectName("radio_0")
        self.frame_3 = QtWidgets.QFrame(self.frame)
        self.frame_3.setGeometry(QtCore.QRect(40, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_3.setFont(font)
        self.frame_3.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_3.setObjectName("frame_3")
        self.radio_10 = QtWidgets.QRadioButton(self.frame_3)
        self.radio_10.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_10.setFont(font)
        self.radio_10.setText("")
        self.radio_10.setChecked(False)
        self.radio_10.setObjectName("radio_10")
        self.frame_4 = QtWidgets.QFrame(self.frame)
        self.frame_4.setGeometry(QtCore.QRect(80, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_4.setFont(font)
        self.frame_4.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_4.setObjectName("frame_4")
        self.radio_20 = QtWidgets.QRadioButton(self.frame_4)
        self.radio_20.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_20.setFont(font)
        self.radio_20.setText("")
        self.radio_20.setChecked(False)
        self.radio_20.setObjectName("radio_20")
        self.frame_5 = QtWidgets.QFrame(self.frame)
        self.frame_5.setGeometry(QtCore.QRect(120, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_5.setFont(font)
        self.frame_5.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_5.setObjectName("frame_5")
        self.radio_30 = QtWidgets.QRadioButton(self.frame_5)
        self.radio_30.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_30.setFont(font)
        self.radio_30.setText("")
        self.radio_30.setChecked(False)
        self.radio_30.setObjectName("radio_30")
        self.frame_7 = QtWidgets.QFrame(self.frame)
        self.frame_7.setGeometry(QtCore.QRect(160, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_7.setFont(font)
        self.frame_7.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_7.setObjectName("frame_7")
        self.radio_40 = QtWidgets.QRadioButton(self.frame_7)
        self.radio_40.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_40.setFont(font)
        self.radio_40.setText("")
        self.radio_40.setChecked(False)
        self.radio_40.setObjectName("radio_40")
        self.frame_6 = QtWidgets.QFrame(self.frame)
        self.frame_6.setGeometry(QtCore.QRect(200, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_6.setFont(font)
        self.frame_6.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_6.setObjectName("frame_6")
        self.radio_50 = QtWidgets.QRadioButton(self.frame_6)
        self.radio_50.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_50.setFont(font)
        self.radio_50.setText("")
        self.radio_50.setChecked(False)
        self.radio_50.setObjectName("radio_50")
        self.frame_8 = QtWidgets.QFrame(self.frame)
        self.frame_8.setGeometry(QtCore.QRect(240, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_8.setFont(font)
        self.frame_8.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_8.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_8.setObjectName("frame_8")
        self.radio_60 = QtWidgets.QRadioButton(self.frame_8)
        self.radio_60.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_60.setFont(font)
        self.radio_60.setText("")
        self.radio_60.setChecked(False)
        self.radio_60.setObjectName("radio_60")
        self.frame_9 = QtWidgets.QFrame(self.frame)
        self.frame_9.setGeometry(QtCore.QRect(280, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_9.setFont(font)
        self.frame_9.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_9.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_9.setObjectName("frame_9")
        self.radio_70 = QtWidgets.QRadioButton(self.frame_9)
        self.radio_70.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_70.setFont(font)
        self.radio_70.setText("")
        self.radio_70.setChecked(False)
        self.radio_70.setObjectName("radio_70")
        self.frame_10 = QtWidgets.QFrame(self.frame)
        self.frame_10.setGeometry(QtCore.QRect(320, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_10.setFont(font)
        self.frame_10.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_10.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_10.setObjectName("frame_10")
        self.radioButton_10 = QtWidgets.QRadioButton(self.frame_10)
        self.radioButton_10.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radioButton_10.setFont(font)
        self.radioButton_10.setText("")
        self.radioButton_10.setChecked(False)
        self.radioButton_10.setObjectName("radioButton_10")
        self.frame_11 = QtWidgets.QFrame(self.frame_10)
        self.frame_11.setGeometry(QtCore.QRect(0, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_11.setFont(font)
        self.frame_11.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_11.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_11.setObjectName("frame_11")
        self.radio_80 = QtWidgets.QRadioButton(self.frame_11)
        self.radio_80.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_80.setFont(font)
        self.radio_80.setText("")
        self.radio_80.setChecked(False)
        self.radio_80.setObjectName("radio_80")
        self.frame_12 = QtWidgets.QFrame(self.frame)
        self.frame_12.setGeometry(QtCore.QRect(360, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_12.setFont(font)
        self.frame_12.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_12.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_12.setObjectName("frame_12")
        self.radio_90 = QtWidgets.QRadioButton(self.frame_12)
        self.radio_90.setGeometry(QtCore.QRect(13, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_90.setFont(font)
        self.radio_90.setText("")
        self.radio_90.setChecked(False)
        self.radio_90.setObjectName("radio_90")
        self.frame_24 = QtWidgets.QFrame(self.frame)
        self.frame_24.setGeometry(QtCore.QRect(400, 0, 51, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_24.setFont(font)
        self.frame_24.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_24.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_24.setObjectName("frame_24")
        self.radio_100 = QtWidgets.QRadioButton(self.frame_24)
        self.radio_100.setGeometry(QtCore.QRect(17, 10, 16, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radio_100.setFont(font)
        self.radio_100.setText("")
        self.radio_100.setChecked(False)
        self.radio_100.setObjectName("radio_100")
        self.section_text = QtWidgets.QLabel(Survey)
        self.section_text.setGeometry(QtCore.QRect(15, 100, 391, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.section_text.setFont(font)
        self.section_text.setFrameShape(QtWidgets.QFrame.Box)
        self.section_text.setObjectName("section_text")
        self.prompt_text = QtWidgets.QLabel(Survey)
        self.prompt_text.setGeometry(QtCore.QRect(15, 140, 391, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.prompt_text.setFont(font)
        self.prompt_text.setFrameShape(QtWidgets.QFrame.Box)
        self.prompt_text.setObjectName("prompt_text")
        self.frame_13 = QtWidgets.QFrame(Survey)
        self.frame_13.setGeometry(QtCore.QRect(405, 100, 461, 41))
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
        self.frame_19 = QtWidgets.QFrame(self.frame_13)
        self.frame_19.setGeometry(QtCore.QRect(200, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_19.setFont(font)
        self.frame_19.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_19.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_19.setObjectName("frame_19")
        self.label_18 = QtWidgets.QLabel(self.frame_19)
        self.label_18.setGeometry(QtCore.QRect(5, 10, 32, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_18.setFont(font)
        self.label_18.setAlignment(QtCore.Qt.AlignCenter)
        self.label_18.setObjectName("label_18")
        self.frame_20 = QtWidgets.QFrame(self.frame_13)
        self.frame_20.setGeometry(QtCore.QRect(240, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_20.setFont(font)
        self.frame_20.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_20.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_20.setObjectName("frame_20")
        self.label_19 = QtWidgets.QLabel(self.frame_20)
        self.label_19.setGeometry(QtCore.QRect(5, 10, 32, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_19.setFont(font)
        self.label_19.setAlignment(QtCore.Qt.AlignCenter)
        self.label_19.setObjectName("label_19")
        self.frame_21 = QtWidgets.QFrame(self.frame_13)
        self.frame_21.setGeometry(QtCore.QRect(280, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_21.setFont(font)
        self.frame_21.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_21.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_21.setObjectName("frame_21")
        self.label_20 = QtWidgets.QLabel(self.frame_21)
        self.label_20.setGeometry(QtCore.QRect(5, 10, 32, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_20.setFont(font)
        self.label_20.setAlignment(QtCore.Qt.AlignCenter)
        self.label_20.setObjectName("label_20")
        self.frame_22 = QtWidgets.QFrame(self.frame_13)
        self.frame_22.setGeometry(QtCore.QRect(320, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_22.setFont(font)
        self.frame_22.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_22.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_22.setObjectName("frame_22")
        self.label_21 = QtWidgets.QLabel(self.frame_22)
        self.label_21.setGeometry(QtCore.QRect(5, 10, 32, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_21.setFont(font)
        self.label_21.setAlignment(QtCore.Qt.AlignCenter)
        self.label_21.setObjectName("label_21")
        self.frame_23 = QtWidgets.QFrame(self.frame_13)
        self.frame_23.setGeometry(QtCore.QRect(360, 0, 41, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_23.setFont(font)
        self.frame_23.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_23.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_23.setObjectName("frame_23")
        self.label_22 = QtWidgets.QLabel(self.frame_23)
        self.label_22.setGeometry(QtCore.QRect(5, 10, 32, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_22.setFont(font)
        self.label_22.setAlignment(QtCore.Qt.AlignCenter)
        self.label_22.setObjectName("label_22")
        self.frame_25 = QtWidgets.QFrame(self.frame_13)
        self.frame_25.setGeometry(QtCore.QRect(400, 0, 51, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.frame_25.setFont(font)
        self.frame_25.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_25.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_25.setObjectName("frame_25")
        self.label_23 = QtWidgets.QLabel(self.frame_25)
        self.label_23.setGeometry(QtCore.QRect(5, 10, 41, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_23.setFont(font)
        self.label_23.setAlignment(QtCore.Qt.AlignCenter)
        self.label_23.setObjectName("label_23")
        self.label_2 = QtWidgets.QLabel(Survey)
        self.label_2.setGeometry(QtCore.QRect(30, 10, 571, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")

        self.retranslateUi(Survey)
        QtCore.QMetaObject.connectSlotsByName(Survey)

    def retranslateUi(self, Survey):
        _translate = QtCore.QCoreApplication.translate
        Survey.setWindowTitle(_translate("Survey", "Survey"))
        self.section_text.setText(_translate("Survey", "What % of the time will this robot be..."))
        self.prompt_text.setText(_translate("Survey", "TextLabel"))
        self.label.setText(_translate("Survey", "0"))
        self.label_13.setText(_translate("Survey", "10"))
        self.label_14.setText(_translate("Survey", "20"))
        self.label_15.setText(_translate("Survey", "30"))
        self.label_16.setText(_translate("Survey", "40"))
        self.label_18.setText(_translate("Survey", "50"))
        self.label_19.setText(_translate("Survey", "60"))
        self.label_20.setText(_translate("Survey", "70"))
        self.label_21.setText(_translate("Survey", "80"))
        self.label_22.setText(_translate("Survey", "90"))
        self.label_23.setText(_translate("Survey", "100"))
        self.label_2.setText(_translate("Survey", "Please answer the following question"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Survey = QtWidgets.QDialog()
    ui = Ui_Survey()
    ui.setupUi(Survey)
    Survey.show()
    sys.exit(app.exec_())
