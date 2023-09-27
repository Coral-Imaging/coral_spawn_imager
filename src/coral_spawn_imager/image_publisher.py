#!/usr/bin/env python3

""" 
create image publisher from picam2/ROS
publishes single images obtained from Picamerea2Wrapper
"""

import rospy
from sensor_msgs.msg import Image
import os
from cv_bridge import CvBridge
# import numpy as np

from coral_spawn_imager.PiCamera2Wrapper import PiCamera2Wrapper


rospy.init_node('image_pub')
rate = rospy.Rate(10)

CAMERA_CONFIGURATION_FILE = '../../launch/camera_config_dev.json'
path = os.path.dirname(__file__) # get path to this file
print('Working directory: {}'.format(path))
picam = PiCamera2Wrapper(config_file=os.path.join(path, CAMERA_CONFIGURATION_FILE))

image_pub = rospy.Publisher('/image', Image, queue_size=1)
bridge = CvBridge()

while not rospy.is_shutdown():
    rospy.loginfo('Capturing image')
    img_np, img_name, metadata = picam.capture_image()
    print(f'image size: {img_np.shape}')

    rospy.loginfo('convert image to msg')
    ros_image = bridge.cv2_to_imgmsg(img_np, encoding="rgb8")
    
    rospy.loginfo('publish image')
    image_pub.publish(ros_image)
    
    rospy.loginfo(f'number of subscribers to topic: {image_pub.get_num_connections()}')
    rate.sleep()
    
print('done')