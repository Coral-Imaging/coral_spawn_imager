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
from picamera import PiCamera
from PIL import Image as pilimage
import time
from io import BytesIO
from cv_bridge import CvBridge 
import numpy as np

def capture_image(camera):
    stream = BytesIO()
    camera.capture(stream, 'jpeg')
    stream.seek(0)
    image_pil = pilimage.open(stream)
    return image_pil
    

def imager():
    pub = rospy.Publisher('image', Image, queue_size=10)
    rospy.init_node('picam', anonymous=True)
    # unsure of camera capture rate - need to check, but pertty sure it's slow atm
    rate = rospy.Rate(0.1) # 0.25 Hz

    # get camera configuration parameters
    cam_idx = rospy.get_param("/camera_index")
    image_height = rospy.get_param("/image_height")
    image_width = rospy.get_param("/image_width")
    iso = rospy.get_param("/iso")
    awb_mode = rospy.get_param("/awb_mode")
    red_gain = rospy.get_param("/red_gain")
    blue_gain = rospy.get_param("/blue_gain")
    shutter_speed = rospy.get_param("/shutter_speed")
    exposure_mode = rospy.get_param("/exposure_mode")
    
    # set picamera parameters
    picam = PiCamera()
    picam.iso = iso
    # picam.sensor_mode = 2
    picam.resolution = (image_width, image_height)
    picam.shutter_speed = shutter_speed # 100 ms
    picam.awb_mode = awb_mode
    time.sleep(2) # cam some time to adjust to conditions

    while not rospy.is_shutdown():

        # get image
        # picam.capture('test.jpg')
        rospy.loginfo('Capture image')
        img = capture_image(picam)
        img = np.array(img)

        br = CvBridge()
        pub.publish(br.cv2_to_imgmsg(img))
        
        rate.sleep()


if __name__ == '__main__':
    try:
        imager()
    except rospy.ROSInterruptException:
        pass

