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
# from sensor_msgs.msg import Image
# from picamera import PiCamera
# from PIL import Image as pilimage
# from cv_bridge import CvBridge 
# import numpy as np
import os
import subprocess

from coral_spawn_imager.PiCameraWrapper import PiCameraWrapper

"""
CameraTrigger: 
When this node receives a trigger from the subscribed topic, this node captures and saves the corresponding number of images
"""

class CameraTrigger:

    CAMERA_TRIGGER_NODE_NAME = 'picam_trigger'
    SUBSCRIBER_TOPIC_NAME = 'trigger'
    SAMPLE_SIZE = 5 # number of images captured in sequence after trigger is received
    SAVE_IMAGE_DIR_SSD = '/media/pi/ssd01/images'
    SAVE_IMAGE_DIR_CARD = '/home/pi/images'

    def __init__(self, img_dir=None):
        
        rospy.init_node(self.CAMERA_TRIGGER_NODE_NAME, anonymous=True)

        # self.publisher = rospy.Publisher(self.PUBLISHER_TOPIC_NAME, Image, queue_size=10)
        self.subscriber = rospy.Subscriber(self.SUBSCRIBER_TOPIC_NAME, String, self.callback)
    
        # unsure of camera capture rate - need to check, but pertty sure it's slow atm
        self.rate = rospy.Rate(0.5) # 0.25 Hz

        # picamera object and configure based on ROS parameters
        self.picam = PiCameraWrapper('ROS')

        if img_dir is None:
            if self.check_ssd():
                # ssd is connected, we save images there:
                img_dir = os.path.join(self.SAVE_IMAGE_DIR_SSD)
            else:
                img_dir = os.path.join(self.SAVE_IMAGE_DIR_CARD)
        else:
            img_dir = img_dir
        os.makedirs(img_dir, exist_ok=True)
        self.img_dir = img_dir


    def callback(self, msg):
        """ read in trigger message string and interpret how many images to capture"""

        rospy.loginfo('Trigger message received:')
        print(msg.data)
        # n_img = int(msg.data)
        # n_img = int(msg) # TODO for now, assume msg is a number

        # capture n_imges
        for i in range(self.SAMPLE_SIZE):
            
            img, img_name = self.capture_image()
            rospy.loginfo(f'Capture image: {i}: {os.path.join(self.img_dir, img_name)}')
            img.save(os.path.join(self.img_dir, img_name))

            self.rate.sleep()
        
        rospy.loginfo('Finished image capture. Awaiting image trigger')


    def capture_image(self):

        # TODO capture metadata and append to images?
        img_pil, img_name = self.picam.capture_image_stream(save_dir=self.img_dir)
        return img_pil, img_name
    

    def check_ssd(self):

        # call system process to see if ssd is mounted
        # if connected and mounted, ssd will appear as "/media/pi/ssd01"
        # thus, we search the output for the above string, if we find it, 
        # the ssd is connected and ready for image saving

        out = subprocess.check_output(["cat", "/proc/mounts"])
        out = str(out)
        if out.find('/media/pi/ssd01') > 0:
            rospy.loginfo('SSD mounted at /media/pi/ssd01')
            return True
        else:
            return False




if __name__ == '__main__':
    try:
        CamPub = CameraTrigger()
        rospy.loginfo('Awaiting image trigger')

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

