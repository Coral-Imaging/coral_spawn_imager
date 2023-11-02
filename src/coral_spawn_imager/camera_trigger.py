#!/usr/bin/env python3

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
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

# from sensor_msgs.msg import Image
# from picamera import PiCamera
from PIL import Image as pilimage
# from cv_bridge import CvBridge 
import os
import subprocess
import shutil

from cv_bridge import CvBridge
import cv2 as cv 
import numpy as np
import time
import glob
import code
import random

from coral_spawn_imager.PiCamera2Wrapper import PiCamera2Wrapper
# from ultralytics import YOLO
# from ultralytics.engine.results import Results, Boxes
from coral_spawn_counter.RedCircle_Detector import RedCircle_Detector
from coral_spawn_counter.Surface_Detector import Surface_Detector
from coral_spawn_counter.SubSurface_Detector import SubSurface_Detector

"""
CameraTrigger: 
When this node receives a trigger from the subscribed topic, this node captures and saves the corresponding number of images
"""

class CameraTrigger:

    CAMERA_TRIGGER_NODE_NAME = 'camera_trigger'
    SUBSCRIBER_TOPIC_NAME = 'trigger'
    REMOTE_FOCUS_SUBSCRIBER_TOPIC_NAME = 'remote_focus'
    IMAGE_PUBLISHER_NAME = 'image'
    # IMAGE_SUBSCRIBER_NAME = 'camera/image/compressed'
    
    SAMPLE_SIZE =30 # number of images captured in sequence after trigger is received
    SAMPLE_RATE = 2 # Hz

    SAVE_SSD = '/media/cslics04/cslics_ssd'
    SAVE_SSD_BAK = '/home/cslics04/ssd_bak'
    SAVE_IMAGE_DIR_SSD = 'images'
    SAVE_IMAGE_DIR_SSD_TMP = 'temp'

    # SAVE_IMAGE_DIR_CARD = '/home/cslics04/images'
    # SAVE_IMAGE_DIR_CARD_TMP = '/tmp'

    CAMERA_CONFIGURATION_FILE = '../../launch/camera_config_seasim_2023.json'
    CORAL_METADATA_FILE = '../../launch/coral_metadata.json'

    SURFACE_DETECTION_MODEL_FILE = '/home/cslics04/cslics_ws/src/ultralytics_cslics/weights/cslics_20230905_yolov8n_640p_amtenuis1000.pt'

    # when simulating image capture, default directory for simulated surface images
    detection_mode_options = ['surface', 'subsurface', 'redcircle']
    DEFAULT_DETECTION_MODE = detection_mode_options[0] # NOTE set detection mode

    SIMULATION_MODE=False

    def __init__(self, save_dir=None, detection_mode = DEFAULT_DETECTION_MODE, sim=SIMULATION_MODE):

        self.sim = sim
        self.path = os.path.dirname(__file__) # get path to this file
        print('Working directory: {}'.format(self.path))

        print('Initializing picam_trigger node')
        rospy.init_node(self.CAMERA_TRIGGER_NODE_NAME, anonymous=True)

        # self.publisher = rospy.Publisher(self.PUBLISHER_TOPIC_NAME, Image, queue_size=10)
        self.subscriber = rospy.Subscriber(self.SUBSCRIBER_TOPIC_NAME, String, self.camera_trigger_callback)

        # unsure of camera capture rate - need to check, but pertty sure it's slow atm
        self.rate = rospy.Rate(self.SAMPLE_RATE) # 0.25 Hz

        # picamera object and configure based on ROS parameters
        rospy.loginfo(f'camera_trigger: {os.path.join(self.path, self.CAMERA_CONFIGURATION_FILE)}')
        self.picam = PiCamera2Wrapper(config_file=os.path.join(self.path, self.CAMERA_CONFIGURATION_FILE))

        # for remote focus:
        self.remote_focus_subscriber = rospy.Subscriber(self.REMOTE_FOCUS_SUBSCRIBER_TOPIC_NAME, Int16, self.remote_focus_callback)

        # for image display via ROS
        self.image_pub = rospy.Publisher(self.IMAGE_PUBLISHER_NAME, Image, queue_size=10)
        
        # subscriber to start publishing image
        # self.image_sub = rospy.Subscriber(self.IMAGE_SUBSCRIBER_NAME, Image, self.image_preview_callback, queue_size=1)
        
        # for onboard detection:   
        # currently, if statement has only one mode of detection active.
        # TODO will eventually want both surface and subsurface models running concurrently?     
        self.detection_mode = detection_mode
        if self.detection_mode == self.detection_mode_options[0]: # surface
            meta_dir = '/home/cslics04/cslics_ws/src/coral_spawn_imager'
            img_sim_dir = '/home/cslics04/20231018_cslics_detector_images_sample/surface'
            # self.imgsave_dir = '/home/cslics04/images/surface/detections/detection_images'
            #self.txtsave_dir = '/home/cslics04/images/surface/detections/detection_textfiles'
            self.detector = Surface_Detector(meta_dir, img_dir=img_sim_dir)
            
        elif self.detection_mode == self.detection_mode_options[1]: # subsurface
            meta_dir = '/home/cslics04/cslics_ws/src/coral_spawn_imager'
            img_sim_dir = '/home/cslics04/20231018_cslics_detector_images_sample/subsurface'
            # self.imgsave_dir = '/home/cslics04/images/subsurface/detections/detection_images'
            # self.txtsave_dir = '/home/cslics04/images/subsurface/detections/detection_textfiles'
            self.detector = SubSurface_Detector(meta_dir, img_dir=img_sim_dir)
            
        else: # red circle
            meta_dir = '/home/cslics04/cslics_ws/src/coral_spawn_imager'
            img_sim_dir = '/home/cslics04/20231018_cslics_detector_images_sample/microspheres'
            # self.imgsave_dir = '/home/cslics04/images/redcircles/detections/detection_images'
            # self.txtsave_dir = '/home/cslics04/images/redcircles/detections/detection_textfiles'
            self.detector = RedCircle_Detector(meta_dir=meta_dir, img_dir=img_sim_dir)
        
        # for simulation purposes
        self.img_sim_dir = img_sim_dir
        self.sim_count = 0 # counter to iterate/index img_sim_dir images
        
        # detect if ssd is connected. If not, then use backup image/temp folders
        if save_dir is None:
            if self.check_ssd():
                # ssd is connected, we save images there:
                save_dir = os.path.join(self.SAVE_SSD)
            else:
                save_dir = os.path.join(self.SAVE_SSD_BAK)
        else:
            save_dir = save_dir

        img_dir = os.path.join(save_dir, 'images')
        tmp_dir = os.path.join(save_dir, 'temp')
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(tmp_dir, exist_ok=True)
        self.img_dir = img_dir
        self.tmp_dir = tmp_dir
        
        # NOTE for detector output, img/txt savedir
        if self.detection_mode == self.detection_mode_options[0]: # surface
            self.imgsave_dir = os.path.join(save_dir, 'images','detections_surface','detection_images')
            self.txtsave_dir = os.path.join(save_dir, 'images','detections_surface','detection_textfiles')

        elif self.detection_mode == self.detection_mode_options[1]: # subsurface
            self.imgsave_dir = os.path.join(save_dir, 'images','detections_subsurface','detection_images')
            self.txtsave_dir = os.path.join(save_dir, 'images','detections_subsurface','detection_textfiles')

        else: # if self.detection_mode == self.detection_mode_options[2]: # redcircle
            self.imgsave_dir = os.path.join(save_dir, 'images','detections_redcircle','detection_images')
            self.txtsave_dir = os.path.join(save_dir, 'images','detections_redcircle','detection_textfiles')
        
        os.makedirs(self.imgsave_dir, exist_ok=True)
        os.makedirs(self.txtsave_dir, exist_ok=True)

        self.coral_metadata = self.picam.read_custom_metadata(os.path.join(self.path, self.CORAL_METADATA_FILE))


    # def image_preview_callback(self, msg):
    #     """ subscriber that triggers image preview publishing, blocks camera_trigger until done """
        
    #     rospy.loginfo('image preview message received:')
        
    #     # get image from picam
    #     img, img_name, metadata = self.capture_image(SIM=False)
        
    #     # publish 
    #     #### Create CompressedImage ####
    #     # msg = CompressedImage()
    #     # msg.header.stamp = rospy.Time.now()
    #     # msg.format = "jpeg"
    #     # msg.data = np.array(cv.imencode('.jpeg', img)[1]).tostring()
        
    #     # Publish new image
    #     self.image_pub.publish(msg)


    def camera_trigger_callback(self, msg):
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
            
            img, img_name, metadata = self.capture_image(SIM=self.sim)
            rospy.loginfo(f'Capture image: {i}: {os.path.join(self.tmp_dir, img_name)}')
            self.picam.update_metadata(metadata, self.coral_metadata)
            
            # save RAW image
            metadata_name = img_name.split('.')[0] + '.json'
            self.picam.save_image(img, os.path.join(self.tmp_dir, img_name), metadata, os.path.join(self.tmp_dir, metadata_name))
            # save to tmp and then move to prevent downloading incomplete images from img_dir when saving image is in progress
            shutil.move(os.path.join(self.tmp_dir, img_name), os.path.join(self.img_dir, img_name))
            shutil.move(os.path.join(self.tmp_dir, metadata_name), os.path.join(self.img_dir, metadata_name))
            
            # TODO apply surface and sub-surface detection model later
            if self.detection_mode == 'subsurface':
                img_prep = self.detector.prep_img(img)
            else:
                img_prep = img
            predictions = self.detector.detect(img_prep)
            
            # save predictions
            self.detector.save_image_predictions(predictions, img, img_name, self.imgsave_dir, self.detector.class_colours, self.detector.classes) # TODO setup, so that I can call it like this
            self.detector.save_text_predictions(predictions, img_name, self.txtsave_dir, self.detector.classes)

            self.rate.sleep()
        
        rospy.loginfo('Finished image capture. Awaiting image trigger')


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
            img_list = sorted(glob.glob(os.path.join(self.img_sim_dir, '*.jpg')) +
                              glob.glob(os.path.join(self.img_sim_dir, '*.png')) + 
                              glob.glob(os.path.join(self.img_sim_dir, '*.jpeg')))
            print(f'length of img_list: {len(img_list)}')
            # i = random.randint(0, len(img_list)-1) # TODO probably better to cycle through these images
            img_np = cv.imread(img_list[self.sim_count])
            img_np = cv.cvtColor(img_np, cv.COLOR_BGR2RGB)
            img_name = os.path.basename(img_list[self.sim_count])
            image_pil = pilimage.open(img_list[self.sim_count])
            # metadata_exif = image_pil.getexif()
            metadata = {
                "Filename": image_pil.filename,
                "Image Size": image_pil.size,
                "Image Height": image_pil.height,
                "Image Width": image_pil.width,
                "Image Format": image_pil.format,
                "Image Mode": image_pil.mode # TODO other metadata_exif info (opt)
            }
            self.sim_count+=1 # increment to next image in img_sim_dir
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
        # get info on topic
        # rostopic info /trigger
        # to trigger via command-line:
        # rostopic pub /trigger std_msgs/String "data: tada"
        
        # get image from picam
        img, img_name, metadata = CamPub.capture_image(SIM=False)
        
        # try to publish image constantly?
        # publish 
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(img, encoding="rgb8")
        
        #### Create Image ####
        # msg = Image()
        # msg.header.stamp = rospy.Time.now()
        # msg.format = "jpeg"
        # msg.data = np.array(cv.imencode('.jpeg', img)[1]).tobytes()
        
        # Publish new image
        CamPub.image_pub.publish(ros_image)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

