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
import os

from coral_spawn_imager.PiCameraWrapper import PiCameraWrapper


class CameraTrigger:

    CAMERA_NODE_NAME = 'picam'
    SUBSCRIBER_TOPIC_NAME = '/trigger'

    def __init__(self,img_dir=None):
        
        rospy.init_node(self.CAMERA_NODE_NAME, anonymous=True)

        # self.publisher = rospy.Publisher(self.PUBLISHER_TOPIC_NAME, Image, queue_size=10)
        self.subscriber = rospy.Subscriber(self.SUBSCRIBER_TOPIC_NAME, String, self.callback)
    
        # unsure of camera capture rate - need to check, but pertty sure it's slow atm
        self.rate = rospy.Rate(0.5) # 0.25 Hz

        # picamera object and configure based on ROS parameters
        self.picam = PiCameraWrapper('ROS')

        if img_dir is None:
            img_dir = 'images'
        os.makedirs(img_dir, exist_ok=True)
        self.img_dir = img_dir


    def callback(self, msg):
        """ read in trigger message string and interpret how many images to capture"""

        rospy.loginfo('Trigger message received:')
        print(msg)
        n_img = int(msg.data)
        # n_img = int(msg) # TODO for now, assume msg is a number

        # capture n_imges
        for i in range(n_img):
            rospy.loginfo(f'Capture image: {i}')
            img, img_name = self.capture_image()
            # save the image
            print(os.path.join(self.img_dir, img_name))

            img.save(os.path.join(self.img_dir, img_name))

            # if publish:
            # img = np.asarray(img)
            # br = CvBridge()
            # self.publisher.publish(br.cv2_to_imgmsg(img))

            self.rate.sleep()
        
        rospy.loginfo('Finished image capture. Awaiting image trigger')


    def capture_image(self):

        # TODO capture metadata and append to images?
        img_pil, img_name = self.picam.capture_image_stream()
        return img_pil, img_name
    

    # def imager(self):
        
    #     while not rospy.is_shutdown():
    #         rospy.loginfo('Waiting for image trigger')
    #         self.rate.sleep()   


if __name__ == '__main__':
    try:
        CamPub = CameraPublisher()
        # CamPub.imager()
        rospy.loginfo('Awaiting image trigger')

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

