#! /usr/bin/env python3

"""
project: coral_spawn_imager 
file: camera publisher
A ROS node that publishes picam status and images from HQ picam at set intervals
newly-added is remote_focus subscriber
author: Dorian Tsai
email: dorian.tsai@gmail.com
date: 2022/Oct/07
"""

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
# from sensor_msgs.msg import Image
# from picamera import PiCamera
from PIL import Image as pilimage
# from cv_bridge import CvBridge 
import os
import subprocess
import shutil

import cv2 as cv 
import numpy as np
import time
import glob
import code
import random

from coral_spawn_imager.PiCamera2Wrapper import PiCamera2Wrapper
from ultralytics import YOLO
from ultralytics.engine.results import Results, Boxes

"""
CameraTrigger: 
When this node receives a trigger from the subscribed topic, this node captures and saves the corresponding number of images
"""

class CameraTrigger:

    CAMERA_TRIGGER_NODE_NAME = 'camera_trigger'
    SUBSCRIBER_TOPIC_NAME = 'trigger'
    REMOTE_FOCUS_SUBSCRIBER_TOPIC_NAME = 'remote_focus'
    SAMPLE_SIZE = 2 # number of images captured in sequence after trigger is received
    SAMPLE_RATE = (1.0/4.0) # Hz

    SAVE_SSD = '/media/cslics04/cslics_ssd'
    SAVE_IMAGE_DIR_SSD = '/media/cslics04/cslics_ssd/images'
    SAVE_IMAGE_DIR_SSD_TMP = '/media/cslics04/cslics_ssd/temp'

    SAVE_IMAGE_DIR_CARD = '/home/cslics04/images'
    SAVE_IMAGE_DIR_CARD_TMP = '/tmp'

    CAMERA_CONFIGURATION_FILE = '../../launch/camera_config_dev.json'
    CORAL_METADATA_FILE = '../../launch/coral_metadata.json'

    SURFACE_DETECTION_MODEL_FILE = '/home/cslics04/cslics_ws/src/ultralytics_cslics/weights/cslics_20230905_yolov8n_640p_amtenuis1000.pt'

    # when simulating image capture, default directory for simulated surface images
    IMG_SRC_DIR = '/home/cslics04/cslics_ws/src/coral_spawn_imager/sample_images/surface'

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

        # for remote focus:
        self.remote_focus_subscriber = rospy.Subscriber(self.REMOTE_FOCUS_SUBSCRIBER_TOPIC_NAME, Int16, self.remote_focus_callback)
        
        # for onboard detection:
        # TODO load detection models
        self.model = YOLO(self.SURFACE_DETECTION_MODEL_FILE)
        
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
        print(msg.data) # msg.data unused at the moment

        # update the coral metadata every trigger
        self.coral_metadata = self.picam.read_custom_metadata(os.path.join(self.path, self.CORAL_METADATA_FILE))

        if not os.path.isdir(self.img_dir):
            os.makedir(self.img_dir)

        # capture n_imges
        for i in range(self.SAMPLE_SIZE):
            # print(f'callback metadata: {metadata}')
            
            img, img_name, metadata = self.capture_image(SIM=True)
            rospy.loginfo(f'Capture image: {i}: {os.path.join(self.tmp_dir, img_name)}')
            self.picam.update_metadata(metadata, self.coral_metadata)
            
            # apply surface detection model
            pred = self.model.predict(source=img,
                                      save=True,
                                      save_txt=True, # later set to false, or figure out if can pt to text
                                      save_conf=True,
                                      imgsz=640,
                                      conf=0.5)
            # save results to text
            boxes: Boxes = pred[0].boxes   
            txt_name = img_name.rsplit('.')[0] + '.txt'
            self.save_predictions(boxes, os.path.join(self.img_dir, txt_name))
            
            # TODO apply sub-surface detection model
            
            # TODO draw/save image detections?
            
            # print(f'updated metadata: {metadata}')
            self.picam.save_image(img, os.path.join(self.tmp_dir, img_name), metadata)
            
            # save to tmp and then move to prevent downloading incomplete images from img_dir when saving image is in progress
            shutil.move(os.path.join(self.tmp_dir, img_name), os.path.join(self.img_dir, img_name))

            self.rate.sleep()
        
        rospy.loginfo('Finished image capture. Awaiting image trigger')

    def save_predictions(self, boxes: Boxes, file: str):
        """ save predictions (boxes) to text file"""  
        lines = []
        for b in boxes:
            cls = str(int(b.cls))
            conf = str(float(b.conf))
            xyxyn = b.xyxyn.numpy()[0]
            x1n = str(xyxyn[0])
            y1n = str(xyxyn[1])
            x2n = str(xyxyn[2])
            y2n = str(xyxyn[3])
            # TODO format for YOLO .txt file
            array_str = cls+' '+x1n+' '+y1n+' '+x2n+' '+y2n+' '+conf+'\n'
            lines.append(array_str)
        
        with open(file, 'w') as f:
            f.writelines(lines) 

        return True

    def remote_focus_callback(self, msg):
        """ read in remote_focus message string and interpret setting the remote focus of the arducam camera"""
        rospy.loginfo('Remote_focus message received:')
        print(msg.data)
        
        remote_focus = int(msg.data)
        rospy.loginfo(f'Set remote focus: {remote_focus}')
        self.picam.set_remote_focus(remote_focus)
        rospy.loginfo(f'Get remote focus: {self.picam.get_remote_focus()}')
        

    def capture_image(self, SIM=True):
        """ capture_image: 
        if SIM is true: grab image/metadata from folder
        else: captures an image/metadata from pi camera """
        if SIM:
            img_list = sorted(glob.glob(os.path.join(self.IMG_SRC_DIR, '*.jpg')))
            print(f'length of img_list: {len(img_list)}')
            i = random.randint(0, len(img_list))
            img_np = cv.imread(img_list[i])
            img_name = os.path.basename(img_list[i])
            image_pil = pilimage.open(img_list[i])
            metadata_exif = image_pil.getexif()
            metadata = {
                "Filename": image_pil.filename,
                "Image Size": image_pil.size,
                "Image Height": image_pil.height,
                "Image Width": image_pil.width,
                "Image Format": image_pil.format,
                "Image Mode": image_pil.mode # TODO other metadata_exif info (opt)
            }
        else:
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

