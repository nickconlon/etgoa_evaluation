# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'decision_making_survey.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1678, 850)
        self.submit_button = QtWidgets.QPushButton(Form)
        self.submit_button.setGeometry(QtCore.QRect(1010, 780, 121, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.submit_button.setFont(font)
        self.submit_button.setObjectName("submit_button")
        self.frame = QtWidgets.QFrame(Form)
        self.frame.setGeometry(QtCore.QRect(20, 10, 961, 411))
        self.frame.setFrameShape(QtWidgets.QFrame.Box)
        self.frame.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame.setObjectName("frame")
        self.label = QtWidgets.QLabel(self.frame)
        self.label.setGeometry(QtCore.QRect(20, 0, 871, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label.setFont(font)
        self.label.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label.setWordWrap(True)
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.frame)
        self.label_2.setGeometry(QtCore.QRect(40, 150, 881, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(self.frame)
        self.label_3.setGeometry(QtCore.QRect(40, 290, 911, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.q1_useful = QtWidgets.QTextEdit(self.frame)
        self.q1_useful.setGeometry(QtCore.QRect(40, 190, 901, 71))
        self.q1_useful.setObjectName("q1_useful")
        self.q1_needs = QtWidgets.QTextEdit(self.frame)
        self.q1_needs.setGeometry(QtCore.QRect(40, 330, 901, 71))
        self.q1_needs.setObjectName("q1_needs")
        self.q1_other_free = QtWidgets.QTextEdit(self.frame)
        self.q1_other_free.setGeometry(QtCore.QRect(610, 100, 331, 61))
        self.q1_other_free.setObjectName("q1_other_free")
        self.q1_map_check = QtWidgets.QCheckBox(self.frame)
        self.q1_map_check.setGeometry(QtCore.QRect(50, 70, 70, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.q1_map_check.setFont(font)
        self.q1_map_check.setObjectName("q1_map_check")
        self.q1_telem_check = QtWidgets.QCheckBox(self.frame)
        self.q1_telem_check.setGeometry(QtCore.QRect(120, 70, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.q1_telem_check.setFont(font)
        self.q1_telem_check.setObjectName("q1_telem_check")
        self.q1_cam_check = QtWidgets.QCheckBox(self.frame)
        self.q1_cam_check.setGeometry(QtCore.QRect(240, 70, 161, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.q1_cam_check.setFont(font)
        self.q1_cam_check.setObjectName("q1_cam_check")
        self.q1_assmt_check = QtWidgets.QCheckBox(self.frame)
        self.q1_assmt_check.setGeometry(QtCore.QRect(390, 70, 191, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.q1_assmt_check.setFont(font)
        self.q1_assmt_check.setObjectName("q1_assmt_check")
        self.q1_other_check = QtWidgets.QCheckBox(self.frame)
        self.q1_other_check.setGeometry(QtCore.QRect(590, 70, 251, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.q1_other_check.setFont(font)
        self.q1_other_check.setObjectName("q1_other_check")
        self.frame_2 = QtWidgets.QFrame(Form)
        self.frame_2.setGeometry(QtCore.QRect(20, 430, 961, 411))
        self.frame_2.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_2.setObjectName("frame_2")
        self.label_4 = QtWidgets.QLabel(self.frame_2)
        self.label_4.setGeometry(QtCore.QRect(20, 0, 871, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_4.setFont(font)
        self.label_4.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_4.setWordWrap(True)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.frame_2)
        self.label_5.setGeometry(QtCore.QRect(40, 150, 881, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(self.frame_2)
        self.label_6.setGeometry(QtCore.QRect(40, 290, 911, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")
        self.q2_useful = QtWidgets.QTextEdit(self.frame_2)
        self.q2_useful.setGeometry(QtCore.QRect(40, 190, 901, 71))
        self.q2_useful.setObjectName("q2_useful")
        self.q2_needs = QtWidgets.QTextEdit(self.frame_2)
        self.q2_needs.setGeometry(QtCore.QRect(40, 330, 901, 71))
        self.q2_needs.setObjectName("q2_needs")
        self.q2_other_free = QtWidgets.QTextEdit(self.frame_2)
        self.q2_other_free.setGeometry(QtCore.QRect(590, 100, 351, 61))
        self.q2_other_free.setObjectName("q2_other_free")
        self.q2_cam_check = QtWidgets.QCheckBox(self.frame_2)
        self.q2_cam_check.setGeometry(QtCore.QRect(240, 70, 161, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.q2_cam_check.setFont(font)
        self.q2_cam_check.setObjectName("q2_cam_check")
        self.q2_telem_check = QtWidgets.QCheckBox(self.frame_2)
        self.q2_telem_check.setGeometry(QtCore.QRect(120, 70, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.q2_telem_check.setFont(font)
        self.q2_telem_check.setObjectName("q2_telem_check")
        self.q2_other_check = QtWidgets.QCheckBox(self.frame_2)
        self.q2_other_check.setGeometry(QtCore.QRect(590, 70, 251, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.q2_other_check.setFont(font)
        self.q2_other_check.setObjectName("q2_other_check")
        self.q2_assmt_check = QtWidgets.QCheckBox(self.frame_2)
        self.q2_assmt_check.setGeometry(QtCore.QRect(390, 70, 191, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.q2_assmt_check.setFont(font)
        self.q2_assmt_check.setObjectName("q2_assmt_check")
        self.q2_map_check = QtWidgets.QCheckBox(self.frame_2)
        self.q2_map_check.setGeometry(QtCore.QRect(50, 70, 70, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.q2_map_check.setFont(font)
        self.q2_map_check.setObjectName("q2_map_check")
        self.label_7 = QtWidgets.QLabel(Form)
        self.label_7.setGeometry(QtCore.QRect(990, 240, 661, 371))
        self.label_7.setFrameShape(QtWidgets.QFrame.Box)
        self.label_7.setText("")
        self.label_7.setPixmap(QtGui.QPixmap("../imgs/ui_annotated.jpg"))
        self.label_7.setScaledContents(True)
        self.label_7.setObjectName("label_7")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.submit_button.setText(_translate("Form", "Submit"))
        self.label.setText(_translate("Form", "What information in the user interface did you use to make decisions while planning which POI the robot should visit? (select all that fit)"))
        self.label_2.setText(_translate("Form", "Why was this information useful in making decisions? "))
        self.label_3.setText(_translate("Form", "Do you think any information was missing that would help you make more informed decisions? "))
        self.q1_map_check.setText(_translate("Form", "Map"))
        self.q1_telem_check.setText(_translate("Form", "Telemetry"))
        self.q1_cam_check.setText(_translate("Form", "Camera feed"))
        self.q1_assmt_check.setText(_translate("Form", "Mission assessment"))
        self.q1_other_check.setText(_translate("Form", "Other (free response)"))
        self.label_4.setText(_translate("Form", "What information in the user interface did you use to make decisions while supervising the robot\'s journey to/from the POI? (select all that fit)"))
        self.label_5.setText(_translate("Form", "Why was this information useful in making decisions? "))
        self.label_6.setText(_translate("Form", "Do you think any information was missing that would help you make more informed decisions? "))
        self.q2_cam_check.setText(_translate("Form", "Camera feed"))
        self.q2_telem_check.setText(_translate("Form", "Telemetry"))
        self.q2_other_check.setText(_translate("Form", "Other (free response)"))
        self.q2_assmt_check.setText(_translate("Form", "Mission assessment"))
        self.q2_map_check.setText(_translate("Form", "Map"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
