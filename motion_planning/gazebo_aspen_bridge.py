import rospy
from geometry_msgs.msg import Twist  # Twist messages
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry  # oOdometry messages
from tf.transformations import euler_from_quaternion  # Quaternion conversions
from gazebo_msgs.msg import ModelStates
import argparse

robot_name = 'case'


class Aspen2Gazebo:
    def __init__(self):
        print('Starting Bridge')
        rospy.init_node('bridgers', anonymous=True)
        self.pose_pub = rospy.Publisher('/{}/vrpn_client_node/cohrint_{}/pose'.format(robot_name, robot_name),
                                        PoseStamped, queue_size=10)
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)

        self.cmd_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.cmd_sub = rospy.Subscriber('/{}/jackal_velocity_controller/cmd_vel'.format(robot_name),
                                        Twist, self.cmd_callback)

        self.vel_pub = rospy.Publisher('/{}/jackal_velocity_controller/odom'.format(robot_name), Odometry, queue_size=10)
        self.vel_sub = rospy.Subscriber('/jackal_velocity_controller/odom', Odometry, self.vel_callback)

    def vel_callback(self, msg):
        self.vel_pub.publish(msg)

    def pose_callback(self, msg):
        def extract_msg(data):
            """
            Extract the same fields from different messages
            """
            if type(data) == ModelStates:
                model_idx = data.name.index('jackal')
                lin = data.pose[model_idx].position
                ang = data.pose[model_idx].orientation
                return lin, ang

        p, a = extract_msg(msg)
        newmsg = PoseStamped()
        newmsg.pose.position = p
        newmsg.pose.orientation = a
        self.pose_pub.publish(newmsg)

    def cmd_callback(self, msg):
        self.cmd_pub.publish(msg)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--robot",
                        help='robot name',
                        default='case')
    args = parser.parse_args()
    bridge = Aspen2Gazebo()
    rospy.spin()
