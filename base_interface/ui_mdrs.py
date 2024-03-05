# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_mdrs.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1859, 906)
        MainWindow.setDocumentMode(False)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setGeometry(QtCore.QRect(20, 150, 331, 341))
        self.frame.setFrameShape(QtWidgets.QFrame.Box)
        self.frame.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame.setObjectName("frame")
        self.label = QtWidgets.QLabel(self.frame)
        self.label.setGeometry(QtCore.QRect(10, 30, 311, 81))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label.setFont(font)
        self.label.setFrameShape(QtWidgets.QFrame.Box)
        self.label.setObjectName("label")
        self.label_3 = QtWidgets.QLabel(self.frame)
        self.label_3.setGeometry(QtCore.QRect(10, 110, 311, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.label_3.setFont(font)
        self.label_3.setFrameShape(QtWidgets.QFrame.Box)
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.frame)
        self.label_4.setGeometry(QtCore.QRect(10, 290, 311, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_4.setFont(font)
        self.label_4.setFrameShape(QtWidgets.QFrame.Box)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.frame)
        self.label_5.setGeometry(QtCore.QRect(10, 250, 311, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_5.setFont(font)
        self.label_5.setFrameShape(QtWidgets.QFrame.Box)
        self.label_5.setObjectName("label_5")
        self.heading_text = QtWidgets.QLabel(self.frame)
        self.heading_text.setGeometry(QtCore.QRect(110, 115, 201, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.heading_text.setFont(font)
        self.heading_text.setObjectName("heading_text")
        self.state_text = QtWidgets.QLabel(self.frame)
        self.state_text.setGeometry(QtCore.QRect(110, 295, 201, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.state_text.setFont(font)
        self.state_text.setObjectName("state_text")
        self.time_text = QtWidgets.QLabel(self.frame)
        self.time_text.setGeometry(QtCore.QRect(110, 255, 201, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.time_text.setFont(font)
        self.time_text.setObjectName("time_text")
        self.label_13 = QtWidgets.QLabel(self.frame)
        self.label_13.setGeometry(QtCore.QRect(10, 150, 311, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_13.setFont(font)
        self.label_13.setFrameShape(QtWidgets.QFrame.Box)
        self.label_13.setObjectName("label_13")
        self.velocity_text = QtWidgets.QLabel(self.frame)
        self.velocity_text.setGeometry(QtCore.QRect(110, 155, 201, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.velocity_text.setFont(font)
        self.velocity_text.setObjectName("velocity_text")
        self.label_18 = QtWidgets.QLabel(self.frame)
        self.label_18.setGeometry(QtCore.QRect(0, 0, 331, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_18.setFont(font)
        self.label_18.setAlignment(QtCore.Qt.AlignCenter)
        self.label_18.setObjectName("label_18")
        self.label_20 = QtWidgets.QLabel(self.frame)
        self.label_20.setGeometry(QtCore.QRect(10, 190, 311, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_20.setFont(font)
        self.label_20.setFrameShape(QtWidgets.QFrame.Box)
        self.label_20.setObjectName("label_20")
        self.battery_text = QtWidgets.QLabel(self.frame)
        self.battery_text.setGeometry(QtCore.QRect(110, 195, 201, 31))
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(10)
        self.battery_text.setFont(font)
        self.battery_text.setObjectName("battery_text")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.frame)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(110, 35, 201, 71))
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
        self.battery_backup_text = QtWidgets.QLabel(self.frame)
        self.battery_backup_text.setGeometry(QtCore.QRect(110, 220, 201, 31))
        font = QtGui.QFont()
        font.setFamily("MS Shell Dlg 2")
        font.setPointSize(10)
        self.battery_backup_text.setFont(font)
        self.battery_backup_text.setObjectName("battery_backup_text")
        self.frame_2 = QtWidgets.QFrame(self.centralwidget)
        self.frame_2.setGeometry(QtCore.QRect(360, 10, 991, 481))
        self.frame_2.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_2.setObjectName("frame_2")
        self.map_label = QtWidgets.QLabel(self.frame_2)
        self.map_label.setGeometry(QtCore.QRect(10, 10, 481, 461))
        self.map_label.setMouseTracking(False)
        self.map_label.setFrameShape(QtWidgets.QFrame.Box)
        self.map_label.setText("")
        self.map_label.setPixmap(QtGui.QPixmap("../../../../../../../.designer/backup/mission_area.png"))
        self.map_label.setScaledContents(True)
        self.map_label.setAlignment(QtCore.Qt.AlignCenter)
        self.map_label.setObjectName("map_label")
        self.camera_label = QtWidgets.QLabel(self.frame_2)
        self.camera_label.setGeometry(QtCore.QRect(500, 10, 481, 461))
        self.camera_label.setMouseTracking(False)
        self.camera_label.setFrameShape(QtWidgets.QFrame.Box)
        self.camera_label.setText("")
        self.camera_label.setPixmap(QtGui.QPixmap("../../../../../../../.designer/backup/mission_area.png"))
        self.camera_label.setScaledContents(True)
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setObjectName("camera_label")
        self.competency_assessment_frame = QtWidgets.QFrame(self.centralwidget)
        self.competency_assessment_frame.setGeometry(QtCore.QRect(1360, 10, 451, 251))
        self.competency_assessment_frame.setFrameShape(QtWidgets.QFrame.Box)
        self.competency_assessment_frame.setFrameShadow(QtWidgets.QFrame.Plain)
        self.competency_assessment_frame.setObjectName("competency_assessment_frame")
        self.label_2 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_2.setGeometry(QtCore.QRect(10, 0, 281, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.objective_4_text = QtWidgets.QLabel(self.competency_assessment_frame)
        self.objective_4_text.setGeometry(QtCore.QRect(10, 190, 281, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.objective_4_text.setFont(font)
        self.objective_4_text.setFrameShape(QtWidgets.QFrame.Box)
        self.objective_4_text.setObjectName("objective_4_text")
        self.objective_2_text = QtWidgets.QLabel(self.competency_assessment_frame)
        self.objective_2_text.setGeometry(QtCore.QRect(10, 90, 281, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.objective_2_text.setFont(font)
        self.objective_2_text.setFrameShape(QtWidgets.QFrame.Box)
        self.objective_2_text.setObjectName("objective_2_text")
        self.objective_3_text = QtWidgets.QLabel(self.competency_assessment_frame)
        self.objective_3_text.setGeometry(QtCore.QRect(10, 140, 281, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.objective_3_text.setFont(font)
        self.objective_3_text.setFrameShape(QtWidgets.QFrame.Box)
        self.objective_3_text.setObjectName("objective_3_text")
        self.objective_1_text = QtWidgets.QLabel(self.competency_assessment_frame)
        self.objective_1_text.setGeometry(QtCore.QRect(10, 40, 281, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.objective_1_text.setFont(font)
        self.objective_1_text.setFrameShape(QtWidgets.QFrame.Box)
        self.objective_1_text.setObjectName("objective_1_text")
        self.label_27 = QtWidgets.QLabel(self.competency_assessment_frame)
        self.label_27.setGeometry(QtCore.QRect(290, 0, 151, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_27.setFont(font)
        self.label_27.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_27.setAlignment(QtCore.Qt.AlignCenter)
        self.label_27.setObjectName("label_27")
        self.objective_4_assmt = QtWidgets.QLabel(self.competency_assessment_frame)
        self.objective_4_assmt.setGeometry(QtCore.QRect(290, 190, 151, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.objective_4_assmt.setFont(font)
        self.objective_4_assmt.setFrameShape(QtWidgets.QFrame.Box)
        self.objective_4_assmt.setText("")
        self.objective_4_assmt.setAlignment(QtCore.Qt.AlignCenter)
        self.objective_4_assmt.setObjectName("objective_4_assmt")
        self.objective_3_assmt = QtWidgets.QLabel(self.competency_assessment_frame)
        self.objective_3_assmt.setGeometry(QtCore.QRect(290, 140, 151, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.objective_3_assmt.setFont(font)
        self.objective_3_assmt.setFrameShape(QtWidgets.QFrame.Box)
        self.objective_3_assmt.setText("")
        self.objective_3_assmt.setAlignment(QtCore.Qt.AlignCenter)
        self.objective_3_assmt.setObjectName("objective_3_assmt")
        self.objective_1_assmt = QtWidgets.QLabel(self.competency_assessment_frame)
        self.objective_1_assmt.setGeometry(QtCore.QRect(290, 40, 151, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.objective_1_assmt.setFont(font)
        self.objective_1_assmt.setFrameShape(QtWidgets.QFrame.Box)
        self.objective_1_assmt.setText("")
        self.objective_1_assmt.setAlignment(QtCore.Qt.AlignCenter)
        self.objective_1_assmt.setObjectName("objective_1_assmt")
        self.objective_2_assmt = QtWidgets.QLabel(self.competency_assessment_frame)
        self.objective_2_assmt.setGeometry(QtCore.QRect(290, 90, 151, 41))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.objective_2_assmt.setFont(font)
        self.objective_2_assmt.setFrameShape(QtWidgets.QFrame.Box)
        self.objective_2_assmt.setText("")
        self.objective_2_assmt.setAlignment(QtCore.Qt.AlignCenter)
        self.objective_2_assmt.setObjectName("objective_2_assmt")
        self.frame_5 = QtWidgets.QFrame(self.centralwidget)
        self.frame_5.setGeometry(QtCore.QRect(20, 10, 331, 131))
        self.frame_5.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_5.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_5.setObjectName("frame_5")
        self.gps_connected_indicator = QtWidgets.QLabel(self.frame_5)
        self.gps_connected_indicator.setGeometry(QtCore.QRect(30, 80, 41, 21))
        self.gps_connected_indicator.setFrameShape(QtWidgets.QFrame.Box)
        self.gps_connected_indicator.setText("")
        self.gps_connected_indicator.setObjectName("gps_connected_indicator")
        self.robot_connected_indicator = QtWidgets.QLabel(self.frame_5)
        self.robot_connected_indicator.setGeometry(QtCore.QRect(30, 40, 41, 21))
        self.robot_connected_indicator.setStyleSheet("background-color: rgba(255, 0, 0, 0);")
        self.robot_connected_indicator.setFrameShape(QtWidgets.QFrame.Box)
        self.robot_connected_indicator.setText("")
        self.robot_connected_indicator.setObjectName("robot_connected_indicator")
        self.cameraFF_connected_indicator = QtWidgets.QLabel(self.frame_5)
        self.cameraFF_connected_indicator.setGeometry(QtCore.QRect(170, 80, 41, 21))
        self.cameraFF_connected_indicator.setFrameShape(QtWidgets.QFrame.Box)
        self.cameraFF_connected_indicator.setText("")
        self.cameraFF_connected_indicator.setObjectName("cameraFF_connected_indicator")
        self.camera360_connected_indicator = QtWidgets.QLabel(self.frame_5)
        self.camera360_connected_indicator.setGeometry(QtCore.QRect(170, 40, 41, 21))
        self.camera360_connected_indicator.setFrameShape(QtWidgets.QFrame.Box)
        self.camera360_connected_indicator.setText("")
        self.camera360_connected_indicator.setObjectName("camera360_connected_indicator")
        self.label_9 = QtWidgets.QLabel(self.frame_5)
        self.label_9.setGeometry(QtCore.QRect(80, 40, 81, 21))
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(self.frame_5)
        self.label_10.setGeometry(QtCore.QRect(80, 80, 81, 21))
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(self.frame_5)
        self.label_11.setGeometry(QtCore.QRect(220, 80, 91, 21))
        self.label_11.setObjectName("label_11")
        self.label_22 = QtWidgets.QLabel(self.frame_5)
        self.label_22.setGeometry(QtCore.QRect(0, 0, 331, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_22.setFont(font)
        self.label_22.setAlignment(QtCore.Qt.AlignCenter)
        self.label_22.setObjectName("label_22")
        self.label_12 = QtWidgets.QLabel(self.frame_5)
        self.label_12.setGeometry(QtCore.QRect(220, 40, 101, 21))
        self.label_12.setObjectName("label_12")
        self.frame_9 = QtWidgets.QFrame(self.centralwidget)
        self.frame_9.setGeometry(QtCore.QRect(1360, 270, 451, 221))
        self.frame_9.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_9.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_9.setObjectName("frame_9")
        self.poi_selection = QtWidgets.QComboBox(self.frame_9)
        self.poi_selection.setGeometry(QtCore.QRect(20, 50, 201, 71))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.poi_selection.setFont(font)
        self.poi_selection.setObjectName("poi_selection")
        self.accept_poi_button = QtWidgets.QPushButton(self.frame_9)
        self.accept_poi_button.setGeometry(QtCore.QRect(240, 140, 201, 71))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.accept_poi_button.setFont(font)
        self.accept_poi_button.setObjectName("accept_poi_button")
        self.plan_poi_button = QtWidgets.QPushButton(self.frame_9)
        self.plan_poi_button.setGeometry(QtCore.QRect(240, 50, 201, 71))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.plan_poi_button.setFont(font)
        self.plan_poi_button.setObjectName("plan_poi_button")
        self.label_23 = QtWidgets.QLabel(self.frame_9)
        self.label_23.setGeometry(QtCore.QRect(0, 0, 451, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_23.setFont(font)
        self.label_23.setAlignment(QtCore.Qt.AlignCenter)
        self.label_23.setObjectName("label_23")
        self.frame_10 = QtWidgets.QFrame(self.centralwidget)
        self.frame_10.setGeometry(QtCore.QRect(585, 500, 541, 231))
        self.frame_10.setFrameShape(QtWidgets.QFrame.Box)
        self.frame_10.setFrameShadow(QtWidgets.QFrame.Plain)
        self.frame_10.setObjectName("frame_10")
        self.manual_drive_mode_button = QtWidgets.QPushButton(self.frame_10)
        self.manual_drive_mode_button.setGeometry(QtCore.QRect(10, 10, 161, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.manual_drive_mode_button.setFont(font)
        self.manual_drive_mode_button.setObjectName("manual_drive_mode_button")
        self.stop_mode_button = QtWidgets.QPushButton(self.frame_10)
        self.stop_mode_button.setGeometry(QtCore.QRect(370, 10, 161, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.stop_mode_button.setFont(font)
        self.stop_mode_button.setObjectName("stop_mode_button")
        self.forward_button_2 = QtWidgets.QPushButton(self.frame_10)
        self.forward_button_2.setGeometry(QtCore.QRect(210, 80, 121, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.forward_button_2.setFont(font)
        self.forward_button_2.setObjectName("forward_button_2")
        self.back_button_2 = QtWidgets.QPushButton(self.frame_10)
        self.back_button_2.setGeometry(QtCore.QRect(210, 160, 121, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.back_button_2.setFont(font)
        self.back_button_2.setObjectName("back_button_2")
        self.right_button_2 = QtWidgets.QPushButton(self.frame_10)
        self.right_button_2.setGeometry(QtCore.QRect(350, 110, 121, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.right_button_2.setFont(font)
        self.right_button_2.setObjectName("right_button_2")
        self.left_button_2 = QtWidgets.QPushButton(self.frame_10)
        self.left_button_2.setGeometry(QtCore.QRect(70, 110, 121, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.left_button_2.setFont(font)
        self.left_button_2.setObjectName("left_button_2")
        self.automatic_drive_mode_button = QtWidgets.QPushButton(self.frame_10)
        self.automatic_drive_mode_button.setGeometry(QtCore.QRect(190, 10, 161, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.automatic_drive_mode_button.setFont(font)
        self.automatic_drive_mode_button.setObjectName("automatic_drive_mode_button")
        self.mission_control_panel = QtWidgets.QFrame(self.centralwidget)
        self.mission_control_panel.setGeometry(QtCore.QRect(1140, 500, 671, 231))
        self.mission_control_panel.setFrameShape(QtWidgets.QFrame.Box)
        self.mission_control_panel.setFrameShadow(QtWidgets.QFrame.Plain)
        self.mission_control_panel.setObjectName("mission_control_panel")
        self.label_6 = QtWidgets.QLabel(self.mission_control_panel)
        self.label_6.setGeometry(QtCore.QRect(0, 0, 341, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_6.setFont(font)
        self.label_6.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.label_7 = QtWidgets.QLabel(self.mission_control_panel)
        self.label_7.setGeometry(QtCore.QRect(340, 0, 331, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label_7.setFont(font)
        self.label_7.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.line = QtWidgets.QFrame(self.mission_control_panel)
        self.line.setGeometry(QtCore.QRect(330, 0, 20, 231))
        self.line.setFrameShadow(QtWidgets.QFrame.Plain)
        self.line.setFrameShape(QtWidgets.QFrame.VLine)
        self.line.setObjectName("line")
        self.request_help_button = QtWidgets.QPushButton(self.mission_control_panel)
        self.request_help_button.setGeometry(QtCore.QRect(10, 40, 311, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.request_help_button.setFont(font)
        self.request_help_button.setObjectName("request_help_button")
        self.request_help_text = QtWidgets.QLabel(self.mission_control_panel)
        self.request_help_text.setGeometry(QtCore.QRect(10, 110, 311, 111))
        font = QtGui.QFont()
        font.setPointSize(13)
        self.request_help_text.setFont(font)
        self.request_help_text.setFrameShape(QtWidgets.QFrame.Box)
        self.request_help_text.setText("")
        self.request_help_text.setObjectName("request_help_text")
        self.label_8 = QtWidgets.QLabel(self.mission_control_panel)
        self.label_8.setGeometry(QtCore.QRect(350, 40, 141, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_8.setFont(font)
        self.label_8.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_8.setObjectName("label_8")
        self.label_14 = QtWidgets.QLabel(self.mission_control_panel)
        self.label_14.setGeometry(QtCore.QRect(350, 100, 141, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_14.setFont(font)
        self.label_14.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_14.setObjectName("label_14")
        self.label_15 = QtWidgets.QLabel(self.mission_control_panel)
        self.label_15.setGeometry(QtCore.QRect(350, 160, 141, 61))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_15.setFont(font)
        self.label_15.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_15.setObjectName("label_15")
        self.robot_battery_slider = QtWidgets.QSlider(self.mission_control_panel)
        self.robot_battery_slider.setGeometry(QtCore.QRect(490, 60, 101, 22))
        self.robot_battery_slider.setOrientation(QtCore.Qt.Horizontal)
        self.robot_battery_slider.setObjectName("robot_battery_slider")
        self.robot_battery_lcd = QtWidgets.QLCDNumber(self.mission_control_panel)
        self.robot_battery_lcd.setGeometry(QtCore.QRect(610, 52, 51, 31))
        self.robot_battery_lcd.setObjectName("robot_battery_lcd")
        self.robot_gps_lcd = QtWidgets.QLCDNumber(self.mission_control_panel)
        self.robot_gps_lcd.setGeometry(QtCore.QRect(610, 112, 51, 31))
        self.robot_gps_lcd.setObjectName("robot_gps_lcd")
        self.robot_gps_slider = QtWidgets.QSlider(self.mission_control_panel)
        self.robot_gps_slider.setGeometry(QtCore.QRect(490, 120, 101, 22))
        self.robot_gps_slider.setOrientation(QtCore.Qt.Horizontal)
        self.robot_gps_slider.setObjectName("robot_gps_slider")
        self.robot_power_lcd = QtWidgets.QLCDNumber(self.mission_control_panel)
        self.robot_power_lcd.setGeometry(QtCore.QRect(610, 172, 51, 31))
        self.robot_power_lcd.setObjectName("robot_power_lcd")
        self.robot_power_slider = QtWidgets.QSlider(self.mission_control_panel)
        self.robot_power_slider.setGeometry(QtCore.QRect(490, 180, 101, 22))
        self.robot_power_slider.setOrientation(QtCore.Qt.Horizontal)
        self.robot_power_slider.setObjectName("robot_power_slider")
        self.mission_panel = QtWidgets.QFrame(self.centralwidget)
        self.mission_panel.setGeometry(QtCore.QRect(20, 500, 551, 231))
        self.mission_panel.setFrameShape(QtWidgets.QFrame.Box)
        self.mission_panel.setFrameShadow(QtWidgets.QFrame.Plain)
        self.mission_panel.setObjectName("mission_panel")
        self.mission_text = QtWidgets.QLabel(self.mission_panel)
        self.mission_text.setGeometry(QtCore.QRect(10, 10, 531, 171))
        font = QtGui.QFont()
        font.setPointSize(13)
        self.mission_text.setFont(font)
        self.mission_text.setFrameShape(QtWidgets.QFrame.Box)
        self.mission_text.setText("")
        self.mission_text.setObjectName("mission_text")
        self.iron_checkbox = QtWidgets.QCheckBox(self.mission_panel)
        self.iron_checkbox.setGeometry(QtCore.QRect(20, 190, 81, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.iron_checkbox.setFont(font)
        self.iron_checkbox.setObjectName("iron_checkbox")
        self.gold_checkbox = QtWidgets.QCheckBox(self.mission_panel)
        self.gold_checkbox.setGeometry(QtCore.QRect(110, 190, 81, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.gold_checkbox.setFont(font)
        self.gold_checkbox.setObjectName("gold_checkbox")
        self.lithium_checkbox = QtWidgets.QCheckBox(self.mission_panel)
        self.lithium_checkbox.setGeometry(QtCore.QRect(210, 190, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.lithium_checkbox.setFont(font)
        self.lithium_checkbox.setObjectName("lithium_checkbox")
        self.cobolt_checkbox = QtWidgets.QCheckBox(self.mission_panel)
        self.cobolt_checkbox.setGeometry(QtCore.QRect(340, 190, 81, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.cobolt_checkbox.setFont(font)
        self.cobolt_checkbox.setObjectName("cobolt_checkbox")
        self.zinc_checkbox = QtWidgets.QCheckBox(self.mission_panel)
        self.zinc_checkbox.setGeometry(QtCore.QRect(460, 190, 81, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.zinc_checkbox.setFont(font)
        self.zinc_checkbox.setObjectName("zinc_checkbox")
        self.add_obstacle_frame = QtWidgets.QFrame(self.centralwidget)
        self.add_obstacle_frame.setGeometry(QtCore.QRect(20, 770, 551, 81))
        self.add_obstacle_frame.setFrameShape(QtWidgets.QFrame.Box)
        self.add_obstacle_frame.setFrameShadow(QtWidgets.QFrame.Plain)
        self.add_obstacle_frame.setObjectName("add_obstacle_frame")
        self.obstacle_scroll = QtWidgets.QComboBox(self.add_obstacle_frame)
        self.obstacle_scroll.setGeometry(QtCore.QRect(10, 10, 161, 61))
        self.obstacle_scroll.setObjectName("obstacle_scroll")
        self.obstacle_x = QtWidgets.QTextEdit(self.add_obstacle_frame)
        self.obstacle_x.setGeometry(QtCore.QRect(200, 40, 61, 31))
        self.obstacle_x.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.obstacle_x.setTabChangesFocus(True)
        self.obstacle_x.setObjectName("obstacle_x")
        self.obstacle_y = QtWidgets.QTextEdit(self.add_obstacle_frame)
        self.obstacle_y.setGeometry(QtCore.QRect(290, 40, 61, 31))
        self.obstacle_y.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.obstacle_y.setTabChangesFocus(True)
        self.obstacle_y.setObjectName("obstacle_y")
        self.obstacle_r = QtWidgets.QTextEdit(self.add_obstacle_frame)
        self.obstacle_r.setGeometry(QtCore.QRect(370, 40, 61, 31))
        self.obstacle_r.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.obstacle_r.setTabChangesFocus(True)
        self.obstacle_r.setObjectName("obstacle_r")
        self.obstacle_confirm = QtWidgets.QPushButton(self.add_obstacle_frame)
        self.obstacle_confirm.setGeometry(QtCore.QRect(440, 10, 101, 61))
        self.obstacle_confirm.setFocusPolicy(QtCore.Qt.TabFocus)
        self.obstacle_confirm.setObjectName("obstacle_confirm")
        self.label_16 = QtWidgets.QLabel(self.add_obstacle_frame)
        self.label_16.setGeometry(QtCore.QRect(200, 10, 61, 21))
        self.label_16.setAlignment(QtCore.Qt.AlignCenter)
        self.label_16.setObjectName("label_16")
        self.label_17 = QtWidgets.QLabel(self.add_obstacle_frame)
        self.label_17.setGeometry(QtCore.QRect(290, 10, 61, 21))
        self.label_17.setAlignment(QtCore.Qt.AlignCenter)
        self.label_17.setObjectName("label_17")
        self.label_19 = QtWidgets.QLabel(self.add_obstacle_frame)
        self.label_19.setGeometry(QtCore.QRect(370, 10, 61, 21))
        self.label_19.setAlignment(QtCore.Qt.AlignCenter)
        self.label_19.setObjectName("label_19")
        self.set_home_button = QtWidgets.QPushButton(self.centralwidget)
        self.set_home_button.setGeometry(QtCore.QRect(590, 770, 201, 71))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.set_home_button.setFont(font)
        self.set_home_button.setObjectName("set_home_button")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1859, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "Position:"))
        self.label_3.setText(_translate("MainWindow", "Heading:"))
        self.label_4.setText(_translate("MainWindow", "State:"))
        self.label_5.setText(_translate("MainWindow", "Time:"))
        self.heading_text.setText(_translate("MainWindow", "0.20 deg. | North-East"))
        self.state_text.setText(_translate("MainWindow", "--"))
        self.time_text.setText(_translate("MainWindow", "10:31 | 15:30 PM"))
        self.label_13.setText(_translate("MainWindow", "Velocity:"))
        self.velocity_text.setText(_translate("MainWindow", "0.0 m/s"))
        self.label_18.setText(_translate("MainWindow", "Robot Telemetry"))
        self.label_20.setText(_translate("MainWindow", "Battery:"))
        self.battery_text.setText(_translate("MainWindow", "Primary: 100%"))
        self.latitude_label.setText(_translate("MainWindow", "x: 0.0"))
        self.longitude_label.setText(_translate("MainWindow", "y: 0.0"))
        self.altitude_label.setText(_translate("MainWindow", "z: 0.0"))
        self.battery_backup_text.setText(_translate("MainWindow", "Backup 1: 100%"))
        self.label_2.setText(_translate("MainWindow", "Mission Objectives"))
        self.objective_4_text.setText(_translate("MainWindow", "Avoid all known hazards"))
        self.objective_2_text.setText(_translate("MainWindow", "Arrive within X minutes"))
        self.objective_3_text.setText(_translate("MainWindow", "Keep all batteries above X %"))
        self.objective_1_text.setText(_translate("MainWindow", "Arrive at chosen POI"))
        self.label_27.setText(_translate("MainWindow", "Assessment"))
        self.label_9.setText(_translate("MainWindow", "Robot"))
        self.label_10.setText(_translate("MainWindow", "GPS"))
        self.label_11.setText(_translate("MainWindow", "FF Camera"))
        self.label_22.setText(_translate("MainWindow", "System and Network"))
        self.label_12.setText(_translate("MainWindow", "360 Camera"))
        self.accept_poi_button.setText(_translate("MainWindow", "Accept Plan"))
        self.plan_poi_button.setText(_translate("MainWindow", "Make Plan"))
        self.label_23.setText(_translate("MainWindow", "Robot Planning and Control"))
        self.manual_drive_mode_button.setText(_translate("MainWindow", "Manual\n"
"Drive"))
        self.stop_mode_button.setText(_translate("MainWindow", "Autonomous\n"
"Stop"))
        self.forward_button_2.setText(_translate("MainWindow", "Forward"))
        self.back_button_2.setText(_translate("MainWindow", "Backward"))
        self.right_button_2.setText(_translate("MainWindow", "Right"))
        self.left_button_2.setText(_translate("MainWindow", "Left"))
        self.automatic_drive_mode_button.setText(_translate("MainWindow", "Autonomous\n"
"Drive"))
        self.label_6.setText(_translate("MainWindow", "Mission Control Communication"))
        self.label_7.setText(_translate("MainWindow", "Robot Settings"))
        self.request_help_button.setText(_translate("MainWindow", "Request Assistance"))
        self.label_8.setText(_translate("MainWindow", "Backup Battery"))
        self.label_14.setText(_translate("MainWindow", "GPS Frequency"))
        self.label_15.setText(_translate("MainWindow", "Motor Power"))
        self.iron_checkbox.setText(_translate("MainWindow", "Iron"))
        self.gold_checkbox.setText(_translate("MainWindow", "Gold"))
        self.lithium_checkbox.setText(_translate("MainWindow", "Lithium"))
        self.cobolt_checkbox.setText(_translate("MainWindow", "Cobolt"))
        self.zinc_checkbox.setText(_translate("MainWindow", "Zinc"))
        self.obstacle_confirm.setText(_translate("MainWindow", "Add/Delete"))
        self.label_16.setText(_translate("MainWindow", "Center X"))
        self.label_17.setText(_translate("MainWindow", "Center Y"))
        self.label_19.setText(_translate("MainWindow", "Radius"))
        self.set_home_button.setText(_translate("MainWindow", "Set Home"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
