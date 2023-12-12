# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'test_ui.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1330, 594)
        MainWindow.setDocumentMode(False)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setGeometry(QtCore.QRect(170, 20, 351, 361))
        self.frame.setFrameShape(QtWidgets.QFrame.Box)
        self.frame.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame.setObjectName("frame")
        self.label = QtWidgets.QLabel(self.frame)
        self.label.setGeometry(QtCore.QRect(10, 30, 331, 81))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label.setFont(font)
        self.label.setFrameShape(QtWidgets.QFrame.Box)
        self.label.setObjectName("label")
        self.label_3 = QtWidgets.QLabel(self.frame)
        self.label_3.setGeometry(QtCore.QRect(10, 110, 331, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_3.setFont(font)
        self.label_3.setFrameShape(QtWidgets.QFrame.Box)
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.frame)
        self.label_4.setGeometry(QtCore.QRect(10, 270, 331, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_4.setFont(font)
        self.label_4.setFrameShape(QtWidgets.QFrame.Box)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.frame)
        self.label_5.setGeometry(QtCore.QRect(10, 230, 331, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_5.setFont(font)
        self.label_5.setFrameShape(QtWidgets.QFrame.Box)
        self.label_5.setObjectName("label_5")
        self.heading_text = QtWidgets.QLabel(self.frame)
        self.heading_text.setGeometry(QtCore.QRect(106, 115, 231, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.heading_text.setFont(font)
        self.heading_text.setObjectName("heading_text")
        self.state_text = QtWidgets.QLabel(self.frame)
        self.state_text.setGeometry(QtCore.QRect(106, 275, 231, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.state_text.setFont(font)
        self.state_text.setObjectName("state_text")
        self.time_text = QtWidgets.QLabel(self.frame)
        self.time_text.setGeometry(QtCore.QRect(107, 235, 231, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.time_text.setFont(font)
        self.time_text.setObjectName("time_text")
        self.label_13 = QtWidgets.QLabel(self.frame)
        self.label_13.setGeometry(QtCore.QRect(10, 150, 331, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_13.setFont(font)
        self.label_13.setFrameShape(QtWidgets.QFrame.Box)
        self.label_13.setObjectName("label_13")
        self.velocity_text = QtWidgets.QLabel(self.frame)
        self.velocity_text.setGeometry(QtCore.QRect(106, 155, 231, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.velocity_text.setFont(font)
        self.velocity_text.setObjectName("velocity_text")
        self.label_18 = QtWidgets.QLabel(self.frame)
        self.label_18.setGeometry(QtCore.QRect(0, 0, 351, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_18.setFont(font)
        self.label_18.setAlignment(QtCore.Qt.AlignCenter)
        self.label_18.setObjectName("label_18")
        self.label_20 = QtWidgets.QLabel(self.frame)
        self.label_20.setGeometry(QtCore.QRect(10, 190, 331, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_20.setFont(font)
        self.label_20.setFrameShape(QtWidgets.QFrame.Box)
        self.label_20.setObjectName("label_20")
        self.battery_text = QtWidgets.QLabel(self.frame)
        self.battery_text.setGeometry(QtCore.QRect(107, 195, 231, 31))
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(10)
        self.battery_text.setFont(font)
        self.battery_text.setObjectName("battery_text")
        self.label_29 = QtWidgets.QLabel(self.frame)
        self.label_29.setGeometry(QtCore.QRect(10, 310, 331, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_29.setFont(font)
        self.label_29.setFrameShape(QtWidgets.QFrame.Box)
        self.label_29.setObjectName("label_29")
        self.mode_text = QtWidgets.QLabel(self.frame)
        self.mode_text.setGeometry(QtCore.QRect(106, 315, 231, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.mode_text.setFont(font)
        self.mode_text.setObjectName("mode_text")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.frame)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(104, 35, 231, 71))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.latitude_label = QtWidgets.QLabel(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.latitude_label.setFont(font)
        self.latitude_label.setObjectName("latitude_label")
        self.verticalLayout.addWidget(self.latitude_label)
        self.longitude_label = QtWidgets.QLabel(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.longitude_label.setFont(font)
        self.longitude_label.setObjectName("longitude_label")
        self.verticalLayout.addWidget(self.longitude_label)
        self.altitude_label = QtWidgets.QLabel(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.altitude_label.setFont(font)
        self.altitude_label.setObjectName("altitude_label")
        self.verticalLayout.addWidget(self.altitude_label)
        self.frame_2 = QtWidgets.QFrame(self.centralwidget)
        self.frame_2.setGeometry(QtCore.QRect(530, 20, 391, 321))
        self.frame_2.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_2.setObjectName("frame_2")
        self.label_21 = QtWidgets.QLabel(self.frame_2)
        self.label_21.setGeometry(QtCore.QRect(10, 10, 371, 301))
        self.label_21.setMouseTracking(False)
        self.label_21.setFrameShape(QtWidgets.QFrame.Box)
        self.label_21.setText("")
        self.label_21.setPixmap(QtGui.QPixmap("../../../../../../../.designer/backup/mission_area.png"))
        self.label_21.setScaledContents(True)
        self.label_21.setAlignment(QtCore.Qt.AlignCenter)
        self.label_21.setObjectName("label_21")
        self.competency_assessment_frame = QtWidgets.QFrame(self.centralwidget)
        self.competency_assessment_frame.setGeometry(QtCore.QRect(930, 20, 381, 251))
        self.competency_assessment_frame.setFrameShape(QtWidgets.QFrame.Box)
        self.competency_assessment_frame.setFrameShadow(QtWidgets.QFrame.Plain)
        self.competency_assessment_frame.setObjectName("competency_assessment_frame")
        self.label_2 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_2.setGeometry(QtCore.QRect(0, 0, 381, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.label_6 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_6.setGeometry(QtCore.QRect(250, 40, 121, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_6.setFont(font)
        self.label_6.setFrameShape(QtWidgets.QFrame.Box)
        self.label_6.setText("")
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.label_7 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_7.setGeometry(QtCore.QRect(250, 80, 121, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_7.setFont(font)
        self.label_7.setFrameShape(QtWidgets.QFrame.Box)
        self.label_7.setText("")
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_8.setGeometry(QtCore.QRect(250, 120, 121, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_8.setFont(font)
        self.label_8.setFrameShape(QtWidgets.QFrame.Box)
        self.label_8.setText("")
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.label_15 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_15.setGeometry(QtCore.QRect(250, 160, 121, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_15.setFont(font)
        self.label_15.setFrameShape(QtWidgets.QFrame.Box)
        self.label_15.setText("")
        self.label_15.setAlignment(QtCore.Qt.AlignCenter)
        self.label_15.setObjectName("label_15")
        self.label_16 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_16.setGeometry(QtCore.QRect(250, 200, 121, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_16.setFont(font)
        self.label_16.setFrameShape(QtWidgets.QFrame.Box)
        self.label_16.setText("")
        self.label_16.setAlignment(QtCore.Qt.AlignCenter)
        self.label_16.setObjectName("label_16")
        self.label_17 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_17.setGeometry(QtCore.QRect(10, 160, 361, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_17.setFont(font)
        self.label_17.setFrameShape(QtWidgets.QFrame.Box)
        self.label_17.setObjectName("label_17")
        self.label_23 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_23.setGeometry(QtCore.QRect(10, 80, 361, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_23.setFont(font)
        self.label_23.setFrameShape(QtWidgets.QFrame.Box)
        self.label_23.setObjectName("label_23")
        self.label_24 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_24.setGeometry(QtCore.QRect(10, 120, 361, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_24.setFont(font)
        self.label_24.setFrameShape(QtWidgets.QFrame.Box)
        self.label_24.setObjectName("label_24")
        self.label_25 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_25.setGeometry(QtCore.QRect(10, 40, 361, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_25.setFont(font)
        self.label_25.setFrameShape(QtWidgets.QFrame.Box)
        self.label_25.setObjectName("label_25")
        self.label_26 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_26.setGeometry(QtCore.QRect(10, 200, 361, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_26.setFont(font)
        self.label_26.setFrameShape(QtWidgets.QFrame.Box)
        self.label_26.setObjectName("label_26")
        self.label_2.raise_()
        self.label_17.raise_()
        self.label_23.raise_()
        self.label_24.raise_()
        self.label_25.raise_()
        self.label_26.raise_()
        self.label_15.raise_()
        self.label_6.raise_()
        self.label_16.raise_()
        self.label_7.raise_()
        self.label_8.raise_()
        self.frame_5 = QtWidgets.QFrame(self.centralwidget)
        self.frame_5.setGeometry(QtCore.QRect(20, 20, 141, 291))
        self.frame_5.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_5.setObjectName("frame_5")
        self.gps_connected_indicator = QtWidgets.QLabel(self.frame_5)
        self.gps_connected_indicator.setGeometry(QtCore.QRect(10, 120, 21, 21))
        self.gps_connected_indicator.setFrameShape(QtWidgets.QFrame.Box)
        self.gps_connected_indicator.setText("")
        self.gps_connected_indicator.setObjectName("gps_connected_indicator")
        self.robot_connected_indicator = QtWidgets.QLabel(self.frame_5)
        self.robot_connected_indicator.setGeometry(QtCore.QRect(10, 80, 21, 21))
        self.robot_connected_indicator.setStyleSheet("background-color: rgba(255, 0, 0, 0);")
        self.robot_connected_indicator.setFrameShape(QtWidgets.QFrame.Box)
        self.robot_connected_indicator.setText("")
        self.robot_connected_indicator.setObjectName("robot_connected_indicator")
        self.sensor1_connected_indicator = QtWidgets.QLabel(self.frame_5)
        self.sensor1_connected_indicator.setGeometry(QtCore.QRect(10, 160, 21, 21))
        self.sensor1_connected_indicator.setFrameShape(QtWidgets.QFrame.Box)
        self.sensor1_connected_indicator.setText("")
        self.sensor1_connected_indicator.setObjectName("sensor1_connected_indicator")
        self.sensor2_connected_indicator = QtWidgets.QLabel(self.frame_5)
        self.sensor2_connected_indicator.setGeometry(QtCore.QRect(10, 200, 21, 21))
        self.sensor2_connected_indicator.setFrameShape(QtWidgets.QFrame.Box)
        self.sensor2_connected_indicator.setText("")
        self.sensor2_connected_indicator.setObjectName("sensor2_connected_indicator")
        self.label_9 = QtWidgets.QLabel(self.frame_5)
        self.label_9.setGeometry(QtCore.QRect(40, 80, 91, 21))
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(self.frame_5)
        self.label_10.setGeometry(QtCore.QRect(40, 120, 91, 21))
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(self.frame_5)
        self.label_11.setGeometry(QtCore.QRect(40, 160, 91, 21))
        self.label_11.setObjectName("label_11")
        self.label_22 = QtWidgets.QLabel(self.frame_5)
        self.label_22.setGeometry(QtCore.QRect(0, 0, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_22.setFont(font)
        self.label_22.setAlignment(QtCore.Qt.AlignCenter)
        self.label_22.setObjectName("label_22")
        self.label_12 = QtWidgets.QLabel(self.frame_5)
        self.label_12.setGeometry(QtCore.QRect(40, 200, 91, 21))
        self.label_12.setObjectName("label_12")
        self.ui_connected_indicator = QtWidgets.QLabel(self.frame_5)
        self.ui_connected_indicator.setGeometry(QtCore.QRect(10, 40, 21, 21))
        self.ui_connected_indicator.setFrameShape(QtWidgets.QFrame.Box)
        self.ui_connected_indicator.setText("")
        self.ui_connected_indicator.setObjectName("ui_connected_indicator")
        self.label_14 = QtWidgets.QLabel(self.frame_5)
        self.label_14.setGeometry(QtCore.QRect(40, 40, 91, 21))
        self.label_14.setObjectName("label_14")
        self.frame_6 = QtWidgets.QFrame(self.centralwidget)
        self.frame_6.setGeometry(QtCore.QRect(930, 280, 381, 211))
        self.frame_6.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_6.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_6.setObjectName("frame_6")
        self.label_19 = QtWidgets.QLabel(self.frame_6)
        self.label_19.setGeometry(QtCore.QRect(0, 0, 381, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_19.setFont(font)
        self.label_19.setAlignment(QtCore.Qt.AlignCenter)
        self.label_19.setObjectName("label_19")
        self.request_mission_control_help_button = QtWidgets.QPushButton(self.frame_6)
        self.request_mission_control_help_button.setGeometry(QtCore.QRect(10, 30, 361, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.request_mission_control_help_button.setFont(font)
        self.request_mission_control_help_button.setObjectName("request_mission_control_help_button")
        self.send_mission_control_udpate_button = QtWidgets.QPushButton(self.frame_6)
        self.send_mission_control_udpate_button.setGeometry(QtCore.QRect(10, 170, 181, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.send_mission_control_udpate_button.setFont(font)
        self.send_mission_control_udpate_button.setObjectName("send_mission_control_udpate_button")
        self.mission_control_update_text = QtWidgets.QPlainTextEdit(self.frame_6)
        self.mission_control_update_text.setGeometry(QtCore.QRect(10, 70, 361, 91))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.mission_control_update_text.setFont(font)
        self.mission_control_update_text.setObjectName("mission_control_update_text")
        self.clera_mission_control_text_button = QtWidgets.QPushButton(self.frame_6)
        self.clera_mission_control_text_button.setGeometry(QtCore.QRect(200, 170, 171, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.clera_mission_control_text_button.setFont(font)
        self.clera_mission_control_text_button.setObjectName("clera_mission_control_text_button")
        self.frame_7 = QtWidgets.QFrame(self.centralwidget)
        self.frame_7.setGeometry(QtCore.QRect(530, 350, 391, 191))
        self.frame_7.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_7.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_7.setObjectName("frame_7")
        self.robot_gps_dial = QtWidgets.QDial(self.frame_7)
        self.robot_gps_dial.setGeometry(QtCore.QRect(270, 100, 111, 61))
        self.robot_gps_dial.setMaximum(100)
        self.robot_gps_dial.setObjectName("robot_gps_dial")
        self.robot_gps_lcd = QtWidgets.QLCDNumber(self.frame_7)
        self.robot_gps_lcd.setGeometry(QtCore.QRect(270, 62, 111, 31))
        self.robot_gps_lcd.setObjectName("robot_gps_lcd")
        self.robot_battery_lcd = QtWidgets.QLCDNumber(self.frame_7)
        self.robot_battery_lcd.setGeometry(QtCore.QRect(10, 62, 111, 31))
        self.robot_battery_lcd.setObjectName("robot_battery_lcd")
        self.robot_battery_slider = QtWidgets.QSlider(self.frame_7)
        self.robot_battery_slider.setGeometry(QtCore.QRect(10, 110, 111, 41))
        self.robot_battery_slider.setMaximum(100)
        self.robot_battery_slider.setOrientation(QtCore.Qt.Horizontal)
        self.robot_battery_slider.setObjectName("robot_battery_slider")
        self.label_30 = QtWidgets.QLabel(self.frame_7)
        self.label_30.setGeometry(QtCore.QRect(10, 0, 111, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_30.setFont(font)
        self.label_30.setAlignment(QtCore.Qt.AlignCenter)
        self.label_30.setWordWrap(True)
        self.label_30.setObjectName("label_30")
        self.robot_power_slider = QtWidgets.QSlider(self.frame_7)
        self.robot_power_slider.setGeometry(QtCore.QRect(180, 100, 41, 61))
        self.robot_power_slider.setMaximum(100)
        self.robot_power_slider.setOrientation(QtCore.Qt.Vertical)
        self.robot_power_slider.setObjectName("robot_power_slider")
        self.robot_power_lcd = QtWidgets.QLCDNumber(self.frame_7)
        self.robot_power_lcd.setGeometry(QtCore.QRect(140, 62, 111, 31))
        self.robot_power_lcd.setObjectName("robot_power_lcd")
        self.label_31 = QtWidgets.QLabel(self.frame_7)
        self.label_31.setGeometry(QtCore.QRect(150, 0, 101, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_31.setFont(font)
        self.label_31.setAlignment(QtCore.Qt.AlignCenter)
        self.label_31.setWordWrap(True)
        self.label_31.setObjectName("label_31")
        self.label_32 = QtWidgets.QLabel(self.frame_7)
        self.label_32.setGeometry(QtCore.QRect(270, 0, 111, 51))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_32.setFont(font)
        self.label_32.setAlignment(QtCore.Qt.AlignCenter)
        self.label_32.setWordWrap(True)
        self.label_32.setObjectName("label_32")
        self.mission_mode_tabs = QtWidgets.QTabWidget(self.centralwidget)
        self.mission_mode_tabs.setGeometry(QtCore.QRect(170, 390, 351, 101))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.mission_mode_tabs.setFont(font)
        self.mission_mode_tabs.setAutoFillBackground(False)
        self.mission_mode_tabs.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.mission_mode_tabs.setElideMode(QtCore.Qt.ElideNone)
        self.mission_mode_tabs.setUsesScrollButtons(False)
        self.mission_mode_tabs.setTabsClosable(False)
        self.mission_mode_tabs.setObjectName("mission_mode_tabs")
        self.mission_planning_tab = QtWidgets.QWidget()
        self.mission_planning_tab.setObjectName("mission_planning_tab")
        self.frame_8 = QtWidgets.QFrame(self.mission_planning_tab)
        self.frame_8.setGeometry(QtCore.QRect(10, 10, 331, 51))
        self.frame_8.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_8.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_8.setObjectName("frame_8")
        self.poi_selection = QtWidgets.QComboBox(self.frame_8)
        self.poi_selection.setGeometry(QtCore.QRect(10, 10, 151, 31))
        self.poi_selection.setObjectName("poi_selection")
        self.accept_poi_order_button = QtWidgets.QPushButton(self.frame_8)
        self.accept_poi_order_button.setGeometry(QtCore.QRect(170, 10, 151, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.accept_poi_order_button.setFont(font)
        self.accept_poi_order_button.setObjectName("accept_poi_order_button")
        self.mission_mode_tabs.addTab(self.mission_planning_tab, "")
        self.mission_execution_tab = QtWidgets.QWidget()
        self.mission_execution_tab.setObjectName("mission_execution_tab")
        self.frame_9 = QtWidgets.QFrame(self.mission_execution_tab)
        self.frame_9.setGeometry(QtCore.QRect(10, 10, 331, 51))
        self.frame_9.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_9.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_9.setObjectName("frame_9")
        self.drive_mode_button = QtWidgets.QPushButton(self.frame_9)
        self.drive_mode_button.setGeometry(QtCore.QRect(10, 10, 151, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.drive_mode_button.setFont(font)
        self.drive_mode_button.setObjectName("drive_mode_button")
        self.stop_mode_button = QtWidgets.QPushButton(self.frame_9)
        self.stop_mode_button.setGeometry(QtCore.QRect(170, 10, 151, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.stop_mode_button.setFont(font)
        self.stop_mode_button.setObjectName("stop_mode_button")
        self.mission_mode_tabs.addTab(self.mission_execution_tab, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1330, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.mission_mode_tabs.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "Position:"))
        self.label_3.setText(_translate("MainWindow", "Heading:"))
        self.label_4.setText(_translate("MainWindow", "State:"))
        self.label_5.setText(_translate("MainWindow", "Time:"))
        self.heading_text.setText(_translate("MainWindow", "0.20 deg. | North-East"))
        self.state_text.setText(_translate("MainWindow", "Autonomous control | Driving to POI A"))
        self.time_text.setText(_translate("MainWindow", "10:31 into mission | 15:30 PM"))
        self.label_13.setText(_translate("MainWindow", "Velocity:"))
        self.velocity_text.setText(_translate("MainWindow", "0.5 m/s"))
        self.label_18.setText(_translate("MainWindow", "Robot Telemetry"))
        self.label_20.setText(_translate("MainWindow", "Battery:"))
        self.battery_text.setText(_translate("MainWindow", "95% | 25 minutes remaining"))
        self.label_29.setText(_translate("MainWindow", "Mode:"))
        self.mode_text.setText(_translate("MainWindow", "Mission Execution"))
        self.latitude_label.setText(_translate("MainWindow", "Latitude: 0.0"))
        self.longitude_label.setText(_translate("MainWindow", "Longitude: 0.0"))
        self.altitude_label.setText(_translate("MainWindow", "Altitude: 0.0"))
        self.label_2.setText(_translate("MainWindow", "Robot Competency Assessment"))
        self.label_17.setText(_translate("MainWindow", "Survey complete < 10mins:"))
        self.label_23.setText(_translate("MainWindow", "Battery level > 50%:"))
        self.label_24.setText(_translate("MainWindow", "All obstacles avoided:"))
        self.label_25.setText(_translate("MainWindow", "Survey site arrival:"))
        self.label_26.setText(_translate("MainWindow", "Survey quality > 90%"))
        self.label_9.setText(_translate("MainWindow", "Robot"))
        self.label_10.setText(_translate("MainWindow", "GPS"))
        self.label_11.setText(_translate("MainWindow", "Sensor 1"))
        self.label_22.setText(_translate("MainWindow", "System"))
        self.label_12.setText(_translate("MainWindow", "Sensor 2"))
        self.label_14.setText(_translate("MainWindow", "Interface"))
        self.label_19.setText(_translate("MainWindow", "Mission Control Communication"))
        self.request_mission_control_help_button.setText(_translate("MainWindow", "Request Help"))
        self.send_mission_control_udpate_button.setText(_translate("MainWindow", "Send Message"))
        self.clera_mission_control_text_button.setText(_translate("MainWindow", "Clear text"))
        self.label_30.setText(_translate("MainWindow", "Backup Battery"))
        self.label_31.setText(_translate("MainWindow", "Robot Power"))
        self.label_32.setText(_translate("MainWindow", "GPS Freqency"))
        self.accept_poi_order_button.setText(_translate("MainWindow", "Accept POI"))
        self.mission_mode_tabs.setTabText(self.mission_mode_tabs.indexOf(self.mission_planning_tab), _translate("MainWindow", "Planning"))
        self.drive_mode_button.setText(_translate("MainWindow", "Drive"))
        self.stop_mode_button.setText(_translate("MainWindow", "Stop"))
        self.mission_mode_tabs.setTabText(self.mission_mode_tabs.indexOf(self.mission_execution_tab), _translate("MainWindow", "Execution"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
