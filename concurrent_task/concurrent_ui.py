# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'concurrent_ui.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1859, 988)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setGeometry(QtCore.QRect(0, 0, 1931, 941))
        self.frame.setFrameShape(QtWidgets.QFrame.Box)
        self.frame.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame.setObjectName("frame")
        self.map_label = QtWidgets.QLabel(self.frame)
        self.map_label.setGeometry(QtCore.QRect(10, 10, 1621, 831))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.map_label.sizePolicy().hasHeightForWidth())
        self.map_label.setSizePolicy(sizePolicy)
        self.map_label.setFrameShape(QtWidgets.QFrame.Box)
        self.map_label.setText("")
        self.map_label.setPixmap(QtGui.QPixmap("./imgs/base_map.png"))
        self.map_label.setScaledContents(False)
        self.map_label.setObjectName("map_label")
        self.mission_control_text = QtWidgets.QLabel(self.frame)
        self.mission_control_text.setGeometry(QtCore.QRect(10, 850, 1021, 71))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.mission_control_text.setFont(font)
        self.mission_control_text.setFrameShape(QtWidgets.QFrame.Box)
        self.mission_control_text.setText("")
        self.mission_control_text.setObjectName("mission_control_text")
        self.longitude_label = QtWidgets.QLabel(self.frame)
        self.longitude_label.setGeometry(QtCore.QRect(1080, 890, 151, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.longitude_label.setFont(font)
        self.longitude_label.setObjectName("longitude_label")
        self.longitude_input = QtWidgets.QPlainTextEdit(self.frame)
        self.longitude_input.setGeometry(QtCore.QRect(1230, 890, 251, 31))
        self.longitude_input.setObjectName("longitude_input")
        self.submit_button = QtWidgets.QPushButton(self.frame)
        self.submit_button.setGeometry(QtCore.QRect(1490, 850, 141, 71))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.submit_button.setFont(font)
        self.submit_button.setObjectName("submit_button")
        self.latitude_input = QtWidgets.QPlainTextEdit(self.frame)
        self.latitude_input.setGeometry(QtCore.QRect(1230, 850, 251, 31))
        self.latitude_input.setObjectName("latitude_input")
        self.latitude_label = QtWidgets.QLabel(self.frame)
        self.latitude_label.setGeometry(QtCore.QRect(1080, 850, 151, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.latitude_label.setFont(font)
        self.latitude_label.setObjectName("latitude_label")
        self.key_label = QtWidgets.QLabel(self.frame)
        self.key_label.setGeometry(QtCore.QRect(1640, 10, 211, 191))
        self.key_label.setFrameShape(QtWidgets.QFrame.Box)
        self.key_label.setText("")
        self.key_label.setPixmap(QtGui.QPixmap("./imgs/legend.png"))
        self.key_label.setScaledContents(True)
        self.key_label.setAlignment(QtCore.Qt.AlignCenter)
        self.key_label.setObjectName("key_label")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1859, 21))
        self.menubar.setObjectName("menubar")
        self.menuPOI_Identification = QtWidgets.QMenu(self.menubar)
        self.menuPOI_Identification.setObjectName("menuPOI_Identification")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.menubar.addAction(self.menuPOI_Identification.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.longitude_label.setText(_translate("MainWindow", "Deg. Longitude"))
        self.submit_button.setText(_translate("MainWindow", "Submit"))
        self.latitude_label.setText(_translate("MainWindow", "Deg. Latitude"))
        self.menuPOI_Identification.setTitle(_translate("MainWindow", "POI Identification"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
