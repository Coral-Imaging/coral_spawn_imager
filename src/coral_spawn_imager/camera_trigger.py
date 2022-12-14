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
import shutil

from coral_spawn_imager.PiCamera2Wrapper import PiCamera2Wrapper

"""
CameraTrigger: 
When this node receives a trigger from the subscribed topic, this node captures and saves the corresponding number of images
"""

class CameraTrigger:

    CAMERA_TRIGGER_NODE_NAME = 'camera_trigger'
    SUBSCRIBER_TOPIC_NAME = 'trigger'
    SAMPLE_SIZE = 20 # number of images captured in sequence after trigger is received
    SAMPLE_RATE = (1.0/4.0) # Hz

    SAVE_SSD = '/media/cslics04/cslics_ssd'
    SAVE_IMAGE_DIR_SSD = '/media/cslics04/cslics_ssd/images'
    SAVE_IMAGE_DIR_SSD_TMP = '/media/cslics04/cslics_ssd/temp'

    SAVE_IMAGE_DIR_CARD = '/home/cslics04/images'
    SAVE_IMAGE_DIR_CARD_TMP = '/tmp'

    CAMERA_CONFIGURATION_FILE = '../../launch/camera_config_seasim.json'
    CORAL_METADATA_FILE = '../../launch/coral_metadata.json'


    def __init__(self, img_dir=None):

        self.path = os.path.dirname(__file__) # get path to this file
        print('Working directory: {}'.format(self.path))


        print('Initializing picam_trigger node')
        rospy.init_node(self.CAMERA_TRIGGER_NODE_NAME, anonymous=True)

        # self.publisher = rospy.Publisher(self.PUBLISHER_TOPIC_NAME, Image, queue_size=10)
        self.subscriber = rospy.Subscriber(self.SUBSCRIBER_TOPIC_NAME, String, self.callback)
    
        # unsure of camera capture rate - need to check, but pertty sure it's slow atm
        self.rate = rospy.Rate(self.SAMPLE_RATE) # 0.25 Hz

        # picamera object and configure based on ROS parameters
        rospy.loginfo(f'camera_trigger: {os.path.join(self.path, self.CAMERA_CONFIGURATION_FILE)}')
        self.picam = PiCamera2Wrapper(config_file=os.path.join(self.path, self.CAMERA_CONFIGURATION_FILE))

        if img_dir is None:
            if self.check_ssd():
                # ssd is connected, we save images there:
                img_dir = os.path.join(self.SAVE_IMAGE_DIR_SSD)
                tmp_dir = os.path.join(self.SAVE_IMAGE_DIR_SSD_TMP)
            else:
                img_dir = os.path.join(self.SAVE_IMAGE_DIR_CARD)
                tmp_dir = os.path.join(self.SAVE_IMAGE_DIR_CARD_TMP)
        else:
            img_dir = img_dir
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(tmp_dir, exist_ok=True)
        self.img_dir = img_dir
        self.tmp_dir = tmp_dir

        self.coral_metadata = self.picam.read_custom_metadata(os.path.join(self.path, self.CORAL_METADATA_FILE))



    def callback(self, msg):
        """ read in trigger message string and interpret how many images to capture"""

        rospy.loginfo('Trigger message received:')
        print(msg.data)

        # update the coral metadata every trigger
        self.coral_metadata = self.picam.read_custom_metadata(os.path.join(self.path, self.CORAL_METADATA_FILE))

        if not os.path.isdir(self.img_dir):
            os.makedir(self.img_dir)

        # capture n_imges
        for i in range(self.SAMPLE_SIZE):
            
            img, img_name, metadata = self.capture_image()
            # print(f'callback metadata: {metadata}')
            rospy.loginfo(f'Capture image: {i}: {os.path.join(self.tmp_dir, img_name)}')
            self.picam.update_metadata(metadata, self.coral_metadata)
            # print(f'updated metadata: {metadata}')
            self.picam.save_image(img, os.path.join(self.tmp_dir, img_name), metadata)
            
            # save to tmp and then move to prevent downloading incomplete images from img_dir when saving image is in progress
            shutil.move(os.path.join(self.tmp_dir, img_name), os.path.join(self.img_dir, img_name))

            self.rate.sleep()
        
        rospy.loginfo('Finished image capture. Awaiting image trigger')


    def capture_image(self):
        img_np, img_name, metadata = self.picam.capture_image()
        return img_np, img_name, metadata
    

    def check_ssd(self):

        # call system process to see if ssd is mounted
        # if connected and mounted, ssd will appear as "/media/pi/ssd01"
        # thus, we search the output for the above string, if we find it, 
        # the ssd is connected and ready for image saving

        out = subprocess.check_output(["cat", "/proc/mounts"])
        out = str(out)
        if out.find(self.SAVE_SSD) > 0:
            rospy.loginfo(f'SSD mounted at {self.SAVE_SSD}')
            return True
        else:
            return False


if __name__ == '__main__':
    
    try:
        print('Running camera_trigger.py')    
        CamPub = CameraTrigger()
        rospy.loginfo('Awaiting image trigger')

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

