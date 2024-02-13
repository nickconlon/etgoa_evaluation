#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ff_camera import CameraWrapper
class CameraNode:
    def __init__(self, _rate=10, _device=2, _topic='/img'):
        """
        Receive position data from the robot
        :param _rate:   the read/publish rate in hz
        :param _device: the device id of the camera
        :param _topic:  the topic to publish the image to
        """
        self._cam = CameraWrapper()
        self._cam.set_cap(_device)
        self._cam.set_parameters()
        self._bridge = CvBridge()

        rospy.init_node('talker', anonymous=True)
        self._pub = rospy.Publisher(_topic, Image, queue_size=10)
        self._rate = rospy.Rate(_rate) # 10hz

    def run(self):
        while not rospy.is_shutdown():
            frame, imgs = self._cam.read()
            image_message = self._bridge.cv2_to_imgmsg(imgs[0], encoding="passthrough")

            rospy.loginfo('sent')
            self._pub.publish(image_message)
            self._rate.sleep()

if __name__ == '__main__':
    try:
        rate = 10
        deviceid = 2
        topic = '/front/left/image_raw'
        cam = CameraNode(rate, deviceid, topic)
        cam.run()
    except rospy.ROSInterruptException:
        pass