# /usr/bin/env python3


"""
project: coral_spawn_imager 
file: camera subscriber
A ROS node that subscribes to the camera image topic and replays/saves the image
author: Dorian Tsai
email: dorian.tsai@gmail.com
date: 2022/Oct/07
"""


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv


class CameraSubscriber(object):

    CAMERA_SUBSCRIBER_NODE_NAME = 'picam_subscriber'
    SUBSCRIBER_TOPIC_NAME = '/image'
    PREVIEW_WINDOW_NAME = 'picam image'

    def __init__(self):

        rospy.init_node(self.CAMERA_SUBSCRIBER_NODE_NAME, anonymous=True)

        # params
        self.image = None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(1) # Hz
        cv.namedWindow(self.PREVIEW_WINDOW_NAME, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.PREVIEW_WINDOW_NAME, 648, 486)

        # subscriber:
        rospy.Subscriber(self.SUBSCRIBER_TOPIC_NAME, Image, self.callback)

    def callback(self, msg):
        rospy.loginfo('Image received.')
        self.image = self.br.imgmsg_to_cv2(msg)

        # show the image
        cv.imshow(self.PREVIEW_WINDOW_NAME, receiver.image)
        cv.waitKey(2)


if __name__ == '__main__':
    try:
        
        receiver = CameraSubscriber()
        rospy.spin()
        cv.destroyAllWindows()

    except rospy.ROSInterruptException:
        pass
