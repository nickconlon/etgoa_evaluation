import sys
import time
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
from std_msgs.msg import String, Float32, Float32MultiArray
from sensor_msgs.msg import NavSatFix

from base_interface.base_interface import BaseInterface
from motion_planning.waypoint_follower import extract_msg
from famsec import goa, rollout


class InterfaceImpl(BaseInterface):
    def __init__(self):
        BaseInterface.__init__(self)
        rospy.init_node('user_interface', anonymous=True)
        self.update_map_display(np.asarray(Image.open(self.img_path)))
        self.poi_selection.addItems(["Select POI", "POI A", "POI B", "POI C", "POI D"])
        self.num_backup_batteries = 5
        self.robot_battery_slider.setMaximum(self.num_backup_batteries)
        self.robot_battery_slider.setMaximum(self.num_backup_batteries)
        self.poi_selection.currentTextChanged.connect(self.start_competency_assessment)
        self.accept_poi_order_button.clicked.connect(self.ros_send_waypoint_plan)
        self.stop_mode_button.clicked.connect(lambda: self.ros_control_waypoint_follower(0.0))
        self.drive_mode_button.clicked.connect(lambda: self.ros_control_waypoint_follower(0.25))

        self.position_sub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                             self.ros_position_callback)
        self.velocity_sub = rospy.Subscriber('/navsat/vel', Vector3Stamped,
                                             self.ros_velocity_callback)
        self.heading_sub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                            self.ros_heading_callback)
        self.velocity_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist,
                                            queue_size=10)
        self.waypoint_speed_pub = rospy.Publisher('/{}/control'.format('tars'), Float32,
                                                  queue_size=10)
        self.waypoint_plan_pub = rospy.Publisher('/{}/waypoints'.format('tars'), Float32MultiArray,
                                                  queue_size=10)

        self.rollout_thread = None
        self.ui_connected = True

    def start_competency_assessment(self):
        try:
            plan = self.mission_manager.current_plan
            self.splash_of_color(self.competency_assessment_frame, timeout=0)
            self.rollout_thread = rollout.RolloutThread()
            self.rollout_thread.pose = [self.position.x, self.position.y, self.position.z]
            self.rollout_thread.orientation = [0, 0, np.deg2rad(self.heading), 0]
            self.rollout_thread.waypoints = plan
            self.rollout_thread.known_obstacles = {}
            self.rollout_thread.goal = plan[-1]
            print('driving to', plan[-1])
            self.rollout_thread.finished.connect(self.finish_competency_assessment)
            self.rollout_thread.start()
        except Exception as e:
            traceback.print_exc()

    def finish_competency_assessment(self):
        try:
            outcomes = np.random.random(size=5)
            labels = [self.label_6, self.label_7, self.label_8, self.label_15, self.label_16]
            for outcome, label in zip(outcomes, labels):
                label.setStyleSheet(
                    'background-color: {}; color: black'.format(goa.semantic_label_color(outcome)))
                label.setText("{}".format(goa.semantic_label_text(outcome)))
            self.splash_of_color(self.competency_assessment_frame, color='light grey', timeout=0)
        except Exception as e:
            traceback.print_exc()

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
                px = plan[:,0]
                py = plan[:,1]
                flat_plan = list(px)+list(py)
                print('setting plan to', flat_plan)
                msg = Float32MultiArray()
                msg.data = flat_plan
                self.waypoint_plan_pub.publish(msg)
                # publish Float32MutliArray message to load the waypoint follower
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
            self.heading = np.rad2deg(angle[-1] + 180) % 360
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
        pass

    def ros_heading_callback(self, msg):
        try:
            model_idx = msg.name.index('jackal')
            lin = msg.pose[model_idx].position
            ang = msg.pose[model_idx].orientation
            pose = (lin.x, lin.y, lin.z)
            angle = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])
            #self.heading = (np.rad2deg(angle[2]) + 360) % 360
        except Exception as e:
            traceback.print_exc()

    def ros_battery_callback(self, msg):
        try:
            self.battery_remaining = msg.data
        except Exception as e:
            traceback.print_exc()

if __name__ == '__main__':
    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = InterfaceImpl()
    MainWindow.show()
    app.exec_()
