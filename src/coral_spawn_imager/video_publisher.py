#!/usr/bin/env python3

"""
video publisher via ROS from picamera2
"""

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from picamera2 import Picamera2, Preview
import time
from sensor_msgs.msg import Image

# setup picamera
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"size": (800, 600)})
picam2.configure(preview_config)
picam2.start_preview(Preview.NULL)
picam2.start()
time.sleep(2)

# setup publisher
rospy.init_node('image_pub')
rate = rospy.Rate(30)
image_pub = rospy.Publisher('/image', Image)
bridge = CvBridge()

while not rospy.is_shutdown():
    
    
    # img = picam2.capture_array()
    img = picam2.capture_array()
    try:
        ros_img = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    except CvBridgeError as e:
        print(e)
        
    image_pub.publish(ros_img)
    
    rospy.loginfo(f'number of subscribers to topic: {image_pub.get_num_connections()}')
    rate.sleep()


print('done')
picam2.close()