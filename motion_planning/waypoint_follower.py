#!/usr/bin/env python
# Import necessary libraries
import time
import argparse

import rospy  # Ros library
import math  # Math library
from geometry_msgs.msg import Twist  # Twist messages
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry  # oOdometry messages
from tf.transformations import euler_from_quaternion  # Quaternion conversions
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
import threading

from etgoa_evaluation.msg import Plan

from base_interface.settings import Settings


def extract_velocity_msg(data):
    """
    Extract the same fields from different messages
    """
    if type(data) == Odometry:
        v = data.twist.twist.linear
        v = np.linalg.norm(np.array([v.x, v.y, v.z]))
        pass
    elif type(data) == Vector3Stamped:
        v = data.vector
        v = np.linalg.norm(np.array([v.x, v.y, v.z]))
    else:
        v = 0
        print('Unknown message type ', type(data))
    return v

def extract_pose_msg(data):
    """
    Extract the same fields from different messages
    """
    if type(data) == ModelStates:
        model_idx = data.name.index('jackal')
        lin = data.pose[model_idx].position
        ang = data.pose[model_idx].orientation
        pose = (lin.x, lin.y, lin.z)
        angle = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])
        return pose, angle
    elif type(data) == PoseStamped:
        x_pose = data.pose.position.x  # Current x-position of the Jackal
        y_pose = data.pose.position.y  # Current y-position of the Jackal
        z_pose = data.pose.position.z  # Current z-position of the Jackal
        pose = (x_pose, y_pose, z_pose)
        orientation_q = [data.pose.orientation.x, data.pose.orientation.y,
                         data.pose.orientation.z, data.pose.orientation.w]
        # Convert quaternion to euler angles
        angle = euler_from_quaternion(orientation_q)
    else:
        pose, angle = [], []
        print('Unknown message type ', type(data))
    return pose, angle


class WaypointFollower:
    ASPEN = 'aspen'
    GAZEBO = 'gazebo'
    OUTDOOR = 'outdoor'
    TARS = 'tars'
    KIPP = 'kipp'
    CASE = 'case'

    def __init__(self, settings):
        settings = Settings(settings)
        settings.read()
        self.area = settings.area
        self.robot = settings.robot_name
        self.all_obstacles = {}
        self.active_obstacles = set()
        self.setup_obstacles(settings.hazards)
        self.xs = []
        self.ys = []
        self.x_dest = None
        self.y_dest = None
        self.px = None
        self.py = None
        self.pose_cmd_vel = None
        self.rot_cmd_vel = None
        self.dist_err = 1
        self.K1 = 2
        self.K2 = 4
        self.max_speed = 0.25
        self.timeout_secs = 1000
        self.capture_dist = settings.capture_dist
        # Initialize ros node
        rospy.init_node('go_to_waypoint', anonymous=True)
        # Set sleep rate
        self.sleep_rate = rospy.Rate(10)
        if self.area == self.ASPEN:
            # For inside the ASPEN lab w/ VICON and our named robots
            self.pose_sub = rospy.Subscriber('/{}/vrpn_client_node/cohrint_{}/pose'.format(self.robot, self.robot),
                                             PoseStamped, self.calculate_heading)
            self.pub_vel = rospy.Publisher('/{}/jackal_velocity_controller/cmd_vel'.format(self.robot),
                                           Twist, queue_size=10)
        elif self.area == self.GAZEBO:
            # For in Gazebo sim w/ unnamed robots
            self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.calculate_heading)
            self.pub_vel = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist,  queue_size=10)
        elif self.area == self.OUTDOOR:
            # For outside w/ GPS and our named robots. TODO GPS integration
            self.pose_sub = None
            self.pub_vel = rospy.Publisher('/{}/jackal_velocity_controller/cmd_vel'.format(self.robot),
                                           Twist, queue_size=10)

        self.control_sub = rospy.Subscriber('/{}/control'.format(self.robot), Float32, self.set_control_callback)
        self.waypoint_sub = rospy.Subscriber('/{}/plan'.format(self.robot), Plan, self.set_waypoints_callback)
        self.wp_thread = None
        self.drive = False

    def setup_obstacles(self, obstacles):
        for o in obstacles:
            if o.id not in self.all_obstacles:
                self.all_obstacles[o.id] = o
            else:
                print('found duplicate obstacle ', o.id)


    def set_control_callback(self, msg):
        print('control', msg)
        self.set_control(msg.data)

    def set_waypoints_callback(self, msg):
        print('waypoints', msg)
        xs = list(msg.xs)
        ys = list(msg.ys)
        active_obstacles = list(msg.active)
        self.set_waypoints(xs, ys)
        self.set_obstacles(active_obstacles)

    def set_obstacles(self, ob_ids):
        print('setting active obstacles: ', ob_ids)
        self.active_obstacles = set(ob_ids)

    def set_waypoints(self, xs, ys):
        self.xs = xs
        self.ys = ys

    def set_control(self, speed):
        self.max_speed = speed
        if self.max_speed > 0:
            if self.wp_thread is not None:
                self.drive = False
                self.wp_thread = None
            self.drive = True
            self.wp_thread = threading.Thread(target=self.go_to_waypoints)
            self.wp_thread.start()
        else:
            self.drive = False

        print('exiting control')

    def calculate_heading(self, data):
        if self.max_speed > 0 and self.x_dest is not None and self.y_dest is not None:

            """
            PD control for waypoint following
            """
            pose, angle = extract_pose_msg(data)
            (x_pose, y_pose, z_pose) = pose[0], pose[1], pose[2]
            (y_rot, x_rot, z_rot) = angle[0], angle[1], angle[2]
            # Calculate x and y errors
            x_err = self.x_dest - x_pose
            y_err = self.y_dest - y_pose

            # Calculate hazard errors
            self.px = x_pose
            self.py = y_pose

            # Calculate angular error
            angle_err = math.atan2(y_err, x_err)

            if angle_err > math.pi:
                angle_err = angle_err - (2 * math.pi)

            angle_err = angle_err - z_rot

            if angle_err > math.pi:
                angle_err = angle_err - (2 * math.pi)
            elif angle_err < -math.pi:
                angle_err = angle_err + (2 * math.pi)

            # Calculate distance error
            self.dist_err = math.sqrt((x_err ** 2) + (y_err ** 2))

            # Calculate command velocities
            if abs(angle_err) > 0.05:  # Only rotate
                self.rot_cmd_vel = np.minimum(angle_err * self.K1, self.max_speed)
                self.pose_cmd_vel = 0
            else:  # Rotate and move
                self.rot_cmd_vel = np.minimum(angle_err * self.K1, self.max_speed)
                self.pose_cmd_vel = np.minimum(self.dist_err * self.K2, self.max_speed)

    def reset(self):
        """
        Reset the parameters
        """
        self.dist_err = 1

    def go_to_waypoint(self, _x, _y):
        """
        Navigate to a waypoint at (_x, _y)
        """
        self.x_dest = _x
        self.y_dest = _y

        # Go to waypoint
        while self.dist_err > self.capture_dist:
            vel = Twist()
            vel.linear.x = self.pose_cmd_vel
            vel.angular.z = self.rot_cmd_vel
            self.pub_vel.publish(vel)
            self.sleep_rate.sleep()

        print("Done")

    def check_hazards(self):
        effect = 1.0
        for o in self.active_obstacles:
            if self.px is not None and self.py is not None and o in self.all_obstacles:
                obs = self.all_obstacles[o]
                if obs.distance(self.px, self.py) <= obs.axis[0]:
                    effect = obs.data
                    print('slowing down!')
                    break
        return effect

    def go_to_waypoints(self):
        """
        Navigate to a waypoint at (_x, _y)
        """
        t_max = self.timeout_secs
        t0 = time.time()
        xs, ys = self.xs.copy(), self.ys.copy()
        for _x, _y in zip(xs, ys):
            self.dist_err = 1.0
            self.x_dest = _x
            self.y_dest = _y
            print("Going to ({}, {})".format(_x, _y))
            # Go to waypoint
            while self.dist_err > self.capture_dist and abs(t0 - time.time()) < t_max and self.drive:
                if self.pose_cmd_vel is not None and self.rot_cmd_vel is not None:
                    hazard_effect = self.check_hazards()
                    vel = Twist()
                    vel.linear.x = self.pose_cmd_vel * hazard_effect
                    vel.angular.z = self.rot_cmd_vel * hazard_effect
                    self.pub_vel.publish(vel)
                self.sleep_rate.sleep()
            print("At waypoint ({}, {})".format(_x, _y))
            if self.dist_err <= self.capture_dist:
                self.xs.remove(_x)
                self.ys.remove(_y)
        print("Waypoints complete")
        self.max_speed = 0.0
        print('waiting for plan')


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("-s", "--settings",
                            help='path to settings file',
                            default="../settings.yaml")
        args = parser.parse_args()

        mission_area = WaypointFollower.GAZEBO
        robot = WaypointFollower.TARS
        wp = WaypointFollower(settings=args.settings)
        wp.set_control(0.0)
        print('waiting for plan')
        rospy.spin()
        #wp.go_to_waypoints()
    except rospy.ROSInterruptException:
        pass
