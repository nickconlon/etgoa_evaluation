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
import numpy as np


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
            self.pose_sub = None # TODO
            self.pub_vel = rospy.Publisher('/{}/jackal_velocity_controller/cmd_vel'.format(robot),
                                           Twist, queue_size=10)

    def calculate_heading(self, data):
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

    def go_to_waypoints(self, _xs, _ys):
        """
        Navigate to a waypoint at (_x, _y)
        """
        t_max = self.timeout_secs
        t0 = time.time()
        for _x, _y in zip(_xs, _ys):
            self.dist_err = 1.0
            self.x_dest = _x
            self.y_dest = _y

            # Go to waypoint
            while self.dist_err > 0.1 and abs(t0 - time.time()) < t_max:
                vel = Twist()
                vel.linear.x = self.pose_cmd_vel
                vel.angular.z = self.rot_cmd_vel
                self.pub_vel.publish(vel)
                self.sleep_rate.sleep()
            print("At waypoint")
        print("Waypoints complete")


if __name__ == '__main__':
    try:
        # a circle of waypoints
        points = np.linspace(0, 2 * np.pi, 25)
        xs = 2 + 2 * np.cos(points)
        ys = 2 + 2 * np.sin(points)
        # xs = [0, 10, 15, 0]
        # ys = [0, 20, 0, 0]
        mission_area = WaypointFollower.GAZEBO
        robot = WaypointFollower.TARS
        wp = WaypointFollower(area=mission_area, robot=robot)
        wp.go_to_waypoints(xs, ys)
    except rospy.ROSInterruptException:
        pass
