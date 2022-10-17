#! /usr/bin/env python3

"""
project: coral_spawn_imager 
file: camera publisher
A ROS node that publishes picam status and images from HQ picam at set intervals
author: Dorian Tsai
email: dorian.tsai@gmail.com
date: 2022/Oct/07
"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
# from picamera import PiCamera
from PIL import Image as pilimage
import time
from io import BytesIO
from cv_bridge import CvBridge 
import numpy as np
from std_msgs.msg import Int32

from coral_spawn_imager.PiCameraWrapper import PiCameraWrapper


class CameraPublisher:

    NODE_NAME = 'picam publisher'
    SUBSCRIBER_TOPIC_NAME = '/publish_images'
    PUBLISHER_TOPIC_NAME = '/images'

    def __init__(self):

        rospy.init_node(self.NODE_NAME, anonymous=True)
        self.subscriber = rospy.Subscriber(self.SUBSCRIBER_TOPIC_NAME, Int32, self.callback)

        self.publisher = rospy.Publisher(self.PUBLISHER_TOPIC_NAME, Image, queue_size=10)
        self.rate = rospy.Rate(0.1)

        self.picam = PiCameraWrapper('ROS')


    def callback(self, msg):
        # trigger publishing of the images
        return False

    def capture_image(self):

        # TODO capture metadata and append to images?
        img_pil, img_name = self.picam.capture_image_stream()
        return img_pil, img_name
    

    def imager(self):
        
        while not rospy.is_shutdown():

            # get image
            rospy.loginfo('Capture image')
            img, img_name = self.capture_image()
            # print(img.size)
            # print(img.format)
            # print(img.mode)

            img = np.asarray(img)
            # print(type(img_np))
            # print(img_np.shape)

            br = CvBridge()
            self.publish(br.cv2_to_imgmsg(img))
            
            self.rate.sleep()


if __name__ == '__main__':
    try:
        CamPub = CameraPublisher()
        CamPub.imager()
    except rospy.ROSInterruptException:
        pass

