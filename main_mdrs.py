import sys
import argparse
import traceback
from PyQt5.QtWidgets import QApplication
import PyQt5.QtGui as QtGui
import PyQt5.QtCore as QtCore
import numpy as np
import qdarktheme
from PIL import Image as pil_image

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist  # Twist messages
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String, Float32, Float32MultiArray
from nav_msgs.msg import Odometry  # odometry messages
# from tf.transformations import euler_from_quaternion  # Quaternion conversions
# from sensor_msgs.msg import NavSatFix

from base_interface.mdrs_base_interface import BaseInterface
from base_interface.control_modes import ControlModeState
from motion_planning.waypoint_follower import extract_pose_msg, extract_velocity_msg
from motion_planning.projections import to_heading_north
from etgoa_evaluation.msg import Plan


class InterfaceImpl(BaseInterface):
    def __init__(self, settings_path):
        BaseInterface.__init__(self, settings_path)
        rospy.init_node('user_interface', anonymous=True)
        self.img_msg = None
        self.bridge = CvBridge()
        self.accept_poi_button.clicked.connect(self.ros_send_waypoint_plan)
        self.stop_mode_button.clicked.connect(lambda: self.ros_control_waypoint_follower(0.0))
        self.automatic_drive_mode_button.clicked.connect(lambda: self.ros_control_waypoint_follower(0.25))
        if self.area == 'aspen':
            self.pose_sub = rospy.Subscriber('/{}/vrpn_client_node/cohrint_{}/pose'.format(self.robot, self.robot), PoseStamped,
                                             self.ros_position_callback)
            # /case/jackal_velocity_controller/odom twist.twist.linear
            self.velocity_sub = rospy.Subscriber('/{}/jackal_velocity_controller/odom'.format(self.robot), Odometry,
                                                 self.ros_velocity_callback)
            self.ff_camera_sub = rospy.Subscriber('/front/left/image_raw', Image,
                                                  self.ros_img_callback)
            self.waypoint_speed_pub = rospy.Publisher('/{}/control'.format(self.robot), Float32,
                                                      queue_size=10)
            self.waypoint_plan_pub = rospy.Publisher('/{}/plan'.format(self.robot), Plan,
                                                     queue_size=10)
            self.control_pub = rospy.Publisher('/{}/jackal_velocity_controller/cmd_vel'.format(self.robot), Twist,
                                               queue_size=10)
        else: # self.area == 'gazebo'
            self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                             self.ros_position_callback)
            self.velocity_sub = rospy.Subscriber('/navsat/vel', Vector3Stamped,
                                                 self.ros_velocity_callback)
            self.ff_camera_sub = rospy.Subscriber('/front/left/image_raw', Image,
                                                 self.ros_img_callback)
            self.waypoint_speed_pub = rospy.Publisher('/{}/control'.format(self.robot), Float32,
                                                      queue_size=10)
            self.waypoint_plan_pub = rospy.Publisher('/{}/plan'.format(self.robot), Plan,
                                                     queue_size=10)
            self.control_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist,
                                               queue_size=10)

        self.teleoperation_actions = {'x':0, 'z':0}
        self.teleoperation_timer = QtCore.QTimer()
        self.teleoperation_timer.timeout.connect(self.teleop_action)

        self.forward_button_2.pressed.connect(lambda: self.teleop_press(0.25, 0))
        self.forward_button_2.released.connect(lambda: self.teleop_release())

        self.back_button_2.pressed.connect(lambda: self.teleop_press(-0.25, 0))
        self.back_button_2.released.connect(lambda: self.teleop_release())

        self.left_button_2.pressed.connect(lambda: self.teleop_press(0, 0.25))
        self.left_button_2.released.connect(lambda: self.teleop_release())

        self.right_button_2.pressed.connect(lambda: self.teleop_press(0, -0.25))
        self.right_button_2.released.connect(lambda: self.teleop_release())

        self.last_robot_update = 0 # position_sub
        self.last_gps_update = 0 # position_sub
        self.ff_cam_updater = QtCore.QTimer()
        self.ff_cam_updater.timeout.connect(self.update_ff_camera_ros)
        self.ff_cam_updater.setInterval(500)
        self.ff_cam_updater.start()
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
                actives = [a for a in self.mission_manager.active_obstacle_ids if 'h' in a]
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
            self.robot_connected = self.mission_time
            self.gps_connected = self.mission_time
            pose, angle = extract_pose_msg(msg)
            self.position.x = pose[0]
            self.position.y = pose[1]
            self.position.z = pose[2]
            self.heading = to_heading_north(angle[-1])
        except Exception as e:
            traceback.print_exc()

    def ros_velocity_callback(self, msg):
        """
        Receive velocity data from the robot
        :param msg:
        :return:
        """
        try:
            v = extract_velocity_msg(msg)
            if self.test_state_test.state == ControlModeState.executing_auto_driving:
                self.mean_velocity.append(v)
            elif self.test_state_test.state == ControlModeState.manual_drive:
                self.mean_velocity.append(0.25)
            self.velocity = v
        except Exception as e:
            traceback.print_exc()

    def ros_img_callback(self, msg):
        """
        Receive front facing camera images from the robot
        :param msg:
        :return:
        """
        try:
            self.img_msg = msg
            self.sensor2_connected = self.mission_time
        except Exception as e:
            traceback.print_exc()


    def update_ff_camera_ros(self):
        """
        Updater for the map
        :return:
        """
        try:
            if self.img_msg is not None:
                img = self.bridge.imgmsg_to_cv2(self.img_msg, desired_encoding='passthrough')
                height, width, channel = img.shape
                bytesPerLine = 3 * width
                qImg = QtGui.QImage(img, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
                self.camera_label.setPixmap(QtGui.QPixmap(qImg))
        except Exception as e:
            traceback.print_exc()

    def teleop_action(self):
        print(self.battery_level)
        if self.battery_level > 0:
            msg = Twist()
            msg.linear.x = self.teleoperation_actions['x']
            msg.angular.z = self.teleoperation_actions['z']
            self.control_pub.publish(msg)

    def teleop_press(self, x, z):
        self.teleoperation_actions['x'] = x
        self.teleoperation_actions['z'] = z
        self.teleoperation_timer.start(50)

    def teleop_release(self):
        self.teleoperation_actions['x'] = 0
        self.teleoperation_actions['z'] = 0
        self.teleoperation_timer.stop()


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
