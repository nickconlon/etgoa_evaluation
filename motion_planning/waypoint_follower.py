#!/usr/bin/env python
# Import necessary libraries
import time

import rospy  # Ros library
import math  # Math library
from geometry_msgs.msg import Twist  # Twist messages
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry  # oOdometry messages
from tf.transformations import euler_from_quaternion  # Quaternion conversions
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
import threading


def extract_msg(data):
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
    return pose, angle


class WaypointFollower:
    ASPEN = 'aspen'
    GAZEBO = 'gazebo'
    OUTDOOR = 'outdoor'
    TARS = 'tars'
    KIPP = 'kipp'
    CASE = 'case'

    def __init__(self, area, robot):
        self.xs = []
        self.ys = []
        self.x_dest = None
        self.y_dest = None
        self.pose_cmd_vel = None
        self.rot_cmd_vel = None
        self.dist_err = 1
        self.K1 = 2
        self.K2 = 4
        self.max_speed = 0.25
        self.timeout_secs = 1000
        # Initialize ros node
        rospy.init_node('go_to_waypoint', anonymous=True)
        # Set sleep rate
        self.sleep_rate = rospy.Rate(10)

        if area == self.ASPEN:
            self.pose_sub = rospy.Subscriber('/{}/vrpn_client_node/cohrint_tars/pose'.format(robot),
                                             PoseStamped, self.calculate_heading)
            self.pub_vel = rospy.Publisher('/{}/jackal_velocity_controller/cmd_vel'.format(robot),
                                           Twist, queue_size=10)
        elif area == self.GAZEBO:
            self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                             self.calculate_heading)
            # Initialize command velocity publisher
            self.pub_vel = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist,
                                           queue_size=10)
        elif area == self.OUTDOOR:
            self.pose_sub = None  # TODO
            self.pub_vel = rospy.Publisher('/{}/jackal_velocity_controller/cmd_vel'.format(robot),
                                           Twist, queue_size=10)
        self.control_sub = rospy.Subscriber('/{}/control'.format(robot), Float32, self.set_control_callback)
        self.waypoint_sub = rospy.Subscriber('/{}/waypoints'.format(robot), Float32MultiArray, self.set_waypoints_callback)
        self.wp_thread = None
        self.drive = False

    def set_control_callback(self, msg):
        print('control', msg)
        self.set_control(msg.data)

    def set_waypoints_callback(self, msg):
        print('waypoints', msg)
        d = np.array(msg.data)
        half = int(len(d) / 2)
        xs = list(d[:half])
        ys = list(d[half:])
        self.set_waypoints(xs, ys)

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
        if self.max_speed > 0 and self.x_dest and self.y_dest:

            """
            PD control for waypoint following
            """
            pose, angle = extract_msg(data)
            (x_pose, y_pose, z_pose) = pose[0], pose[1], pose[2]
            (y_rot, x_rot, z_rot) = angle[0], angle[1], angle[2]
            # Calculate x and y errors
            x_err = self.x_dest - x_pose
            y_err = self.y_dest - y_pose

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
        while self.dist_err > 0.1:
            vel = Twist()
            vel.linear.x = self.pose_cmd_vel
            vel.angular.z = self.rot_cmd_vel
            self.pub_vel.publish(vel)
            self.sleep_rate.sleep()

        print("Done")

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

            # Go to waypoint
            while self.dist_err > 0.1 and abs(t0 - time.time()) < t_max and self.drive:
                if self.pose_cmd_vel is not None and self.rot_cmd_vel is not None:
                    vel = Twist()
                    vel.linear.x = self.pose_cmd_vel
                    vel.angular.z = self.rot_cmd_vel
                    self.pub_vel.publish(vel)
                self.sleep_rate.sleep()
            print("At waypoint ({}, {})".format(_x, _y))
            if self.dist_err <= 0.1:
                self.xs.remove(_x)
                self.ys.remove(_y)
        print("Waypoints complete")
        self.max_speed = 0.0
        print('waiting for plan')


if __name__ == '__main__':
    try:
        # a circle of waypoints
        # points = np.linspace(0, 2 * np.pi, 25)
        # xs = 2 + 2 * np.cos(points)
        # ys = 2 + 2 * np.sin(points)

        xs = [13.077209337210164, 4.469345172648829, -5.485107059487756, -15.439559291624342,
              -16.424231599799654]
        ys = [-8.799172244139658, -13.888835742295582, -14.842188121067972, -15.795540499840364,
              -15.889843999964624]
        mission_area = WaypointFollower.GAZEBO
        robot = WaypointFollower.TARS
        wp = WaypointFollower(area=mission_area, robot=robot)
        wp.set_waypoints(xs, ys)
        wp.set_control(0.0)
        print('waiting for plan')
        rospy.spin()
        #wp.go_to_waypoints()
    except rospy.ROSInterruptException:
        pass
