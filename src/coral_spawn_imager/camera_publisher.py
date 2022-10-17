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

from coral_spawn_imager.PiCameraWrapper import PiCameraWrapper


class CameraPublisher:

    def capture_image(self, PiCam):

        # TODO capture metadata and append to images?
        img_pil, img_name = PiCam.capture_image_stream()
        return img_pil, img_name
    

    def imager(self):
        pub = rospy.Publisher('image', Image, queue_size=10)
        rospy.init_node('picam', anonymous=True)
        # unsure of camera capture rate - need to check, but pertty sure it's slow atm
        rate = rospy.Rate(0.5) # 0.25 Hz

        # picamera object and configure based on ROS parameters
        PiCam = PiCameraWrapper('ROS')
        
        while not rospy.is_shutdown():

            # get image
            rospy.loginfo('Capture image')
            img, img_name = self.capture_image(PiCam)
            # print(img.size)
            # print(img.format)
            # print(img.mode)

            img = np.asarray(img)
            # print(type(img_np))
            # print(img_np.shape)

            br = CvBridge()
            pub.publish(br.cv2_to_imgmsg(img))
            
            rate.sleep()


if __name__ == '__main__':
    try:
        CamPub = CameraPublisher()
        CamPub.imager()
    except rospy.ROSInterruptException:
        pass

