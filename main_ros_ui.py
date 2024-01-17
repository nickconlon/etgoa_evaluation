import sys
import argparse
import traceback
from PyQt5.QtWidgets import QApplication
import numpy as np
import qdarktheme

import rospy
from geometry_msgs.msg import Twist  # Twist messages
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry  # oOdometry messages
from tf.transformations import euler_from_quaternion  # Quaternion conversions
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String, Float32, Float32MultiArray
from sensor_msgs.msg import NavSatFix

from base_interface.base_interface import BaseInterface
from base_interface.control_modes import ControlModeState
from motion_planning.waypoint_follower import extract_msg
from etgoa_evaluation.msg import Plan

class InterfaceImpl(BaseInterface):
    def __init__(self, settings_path):
        BaseInterface.__init__(self, settings_path)
        rospy.init_node('user_interface', anonymous=True)
        self.poi_selection.addItems(["Select POI", "POI A", "POI B", "POI C", "POI D", "HOME"])
        self.num_backup_batteries = 5
        self.robot_battery_slider.setMaximum(self.num_backup_batteries)
        self.robot_battery_slider.setMaximum(self.num_backup_batteries)
        self.accept_poi_button.clicked.connect(self.ros_send_waypoint_plan)
        self.stop_mode_button.clicked.connect(lambda: self.ros_control_waypoint_follower(0.0))
        self.drive_mode_button.clicked.connect(lambda: self.ros_control_waypoint_follower(0.25))

        self.position_sub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                             self.ros_position_callback)
        self.velocity_sub = rospy.Subscriber('/navsat/vel', Vector3Stamped,
                                             self.ros_velocity_callback)
        self.velocity_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist,
                                            queue_size=10)
        self.waypoint_speed_pub = rospy.Publisher('/{}/control'.format('tars'), Float32,
                                                  queue_size=10)
        self.waypoint_plan_pub = rospy.Publisher('/{}/plan'.format('tars'), Plan,
                                                 queue_size=10)

        self.ui_connected = True

    def ros_control_waypoint_follower(self, speed):
        try:
            print('setting speed to ', speed)
            msg = Float32()
            msg.data = speed
            self.waypoint_speed_pub.publish(msg)
        except Exception as e:
            traceback.print_exc()

    def ros_send_waypoint_plan(self):
        try:
            if self.mission_manager.has_plan():
                plan = self.mission_manager.current_plan
                actives = self.mission_manager.active_obstacles
                msg = Plan()
                msg.xs = list(plan[:, 0])
                msg.ys = list(plan[:, 1])
                msg.active = list(actives)
                self.waypoint_plan_pub.publish(msg)
            pass
        except Exception as e:
            traceback.print_exc()

    def ros_position_callback(self, msg):
        """
        Receive position data from the robot
        :param msg:
        :return:
        """
        try:
            pose, angle = extract_msg(msg)
            self.position.x = pose[0]
            self.position.y = pose[1]
            self.position.z = pose[2]
            a = 360 + 90 - np.rad2deg(angle[-1])
            self.heading = a % 360
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
            if self.control_mode.state == ControlModeState.drive:
                self.mean_velocity.append(v)
            self.velocity = v
        except Exception as e:
            traceback.print_exc()

    def ros_image_callback(self, msg):
        """
        Receive images from the robot
        :param msg:
        :return:
        """
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--settings",
                        help='path to settings file',
                        default="./settings.yaml")
    args = parser.parse_args()

    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = InterfaceImpl(args.settings)
    MainWindow.show()
    app.exec_()
