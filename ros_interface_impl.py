import sys
import traceback
from PyQt5.Qt import Qt
from PyQt5.QtWidgets import QApplication
import PyQt5.QtCore as QtCore
import PyQt5.QtGui as QtGui
import numpy as np
import qdarktheme
from PIL import Image

import rospy
from geometry_msgs.msg import Twist  # Twist messages
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry  # oOdometry messages
from tf.transformations import euler_from_quaternion  # Quaternion conversions
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

from base_interface.base_interface import BaseInterface
from famsec import goa


class InterfaceImpl(BaseInterface):
    def __init__(self):
        BaseInterface.__init__(self)
        rospy.init_node('user_interface', anonymous=True)
        self.update_map(np.asarray(Image.open(self.img_path)))
        self.poi_selection.addItems(["Select POI", "POI A", "POI B", "POI C", "POI D"])
        self.num_backup_batteries = 5
        self.robot_battery_slider.setMaximum(self.num_backup_batteries)
        self.robot_battery_slider.setMaximum(self.num_backup_batteries)
        self.poi_selection.currentTextChanged.connect(self.test_competency_assessment)

        self.position_sub = rospy.Subscriber('/navsat/fix', NavSatFix, self.ros_position_callback)
        self.velocity_sub = rospy.Subscriber('/navsat/vel', Vector3Stamped,
                                             self.ros_velocity_callback)
        self.heading_sub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                            self.ros_heading_callback)
        self.velocity_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist,
                                            queue_size=10)

        self.ui_connected = True

    def test_competency_assessment(self):
        outcomes = np.random.random(size=5)
        labels = [self.label_6, self.label_7, self.label_8, self.label_15, self.label_16]
        colors = ['red', 'yellow', 'green']
        for outcome, label in zip(outcomes, labels):
            c = np.random.choice(colors)
            label.setStyleSheet(
                'background-color: {}; color: black'.format(goa.semantic_label_color(outcome)))
            label.setText("{}".format(goa.semantic_label_text(outcome)))
        self.splash_of_color(self.competency_assessment_frame)

    def ros_position_callback(self, msg):
        """
        Receive position data from the robot
        :param msg:
        :return:
        """
        try:
            self.position = [msg.latitude, msg.longitude, msg.altitude]
        except Exception as e:
            traceback.print_exc()

    def ros_velocity_callback(self, msg):
        """
        Receive velocity data from the robot
        :param msg:
        :return:
        """
        try:
            v = msg.vector
            v = np.linalg.norm(np.array([v.x, v.y, v.z]))
            self.velocity = v
        except Exception as e:
            traceback.print_exc()

    def ros_image_callback(self, msg):
        """
        Receive images from the robot
        :param msg:
        :return:
        """
        self.position = np.array([1, 2, 3])
        self.velocity = 0.5
        self.heading = 0
        pass

    def ros_heading_callback(self, msg):
        model_idx = msg.name.index('jackal')
        lin = msg.pose[model_idx].position
        ang = msg.pose[model_idx].orientation
        pose = (lin.x, lin.y, lin.z)
        angle = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])
        self.heading = (np.rad2deg(angle[2]) + 360) % 360


if __name__ == '__main__':
    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = InterfaceImpl()
    MainWindow.show()
    app.exec_()
