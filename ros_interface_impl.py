import sys
import traceback
from PyQt5.Qt import Qt
from PyQt5.QtWidgets import QApplication
import PyQt5.QtCore as QtCore
import PyQt5.QtGui as QtGui
import numpy as np
import qdarktheme

import rospy
from geometry_msgs.msg import Twist  # Twist messages
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry  # oOdometry messages
from tf.transformations import euler_from_quaternion  # Quaternion conversions
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

from base_interface.base_interface import BaseInterface


class InterfaceImpl(BaseInterface):
    def __init__(self):
        BaseInterface.__init__(self)
        rospy.init_node('user_interface', anonymous=True)
        self.img_path = 'base_interface/mission_area.png'
        self.label_21.setPixmap(QtGui.QPixmap(self.img_path))
        self.poi_selection.addItems(["", "A", "B", "C", "D"])
        self.mission_control_help_selector.addItems(self.mission_control.help_requests.keys())
        self.num_backup_batteries = 5
        self.robot_battery_slider.setMaximum(self.num_backup_batteries)
        self.position_sub = rospy.Subscriber('/navsat/fix', NavSatFix, self.ros_position_callback)
        self.velocity_sub = rospy.Subscriber('/navsat/vel', Vector3Stamped,
                                             self.ros_velocity_callback)
        self.velocity_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist,
                                            queue_size=10)

        self.ui_connected = True

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

    def ros_automatic_control_command(self, command):
        """
        Send automatic control cmd_vel messages to the robot
        :param command:
        :return:
        """
        pass

    def ros_manual_control_command(self, command):
        """
        Send manual control cmd_vel messages to the robot
        :param command:
        :return:
        """
        pass


if __name__ == '__main__':
    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = InterfaceImpl()
    MainWindow.show()
    app.exec_()
