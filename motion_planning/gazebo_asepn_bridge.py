import rospy
from geometry_msgs.msg import Twist  # Twist messages
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry  # oOdometry messages
from tf.transformations import euler_from_quaternion  # Quaternion conversions
from gazebo_msgs.msg import ModelStates

robot_name = 'tars'


class Aspen2Gazebo:
    def __init__(self):
        rospy.init_node('bridgers', anonymous=True)
        self.pose_pub = rospy.Publisher('/{}/vrpn_client_node/cohrint_tars/pose'.format(robot_name),
                                        PoseStamped, queue_size=10)
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)
        self.cmd_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.cmd_sub = rospy.Subscriber('/{}/jackal_velocity_controller/cmd_vel'.format(robot_name),
                                        Twist, self.cmd_callback)

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
        print('cmd published')
        self.cmd_pub.publish(msg)


if __name__ == '__main__':
    bridge = Aspen2Gazebo()
    rospy.spin()
