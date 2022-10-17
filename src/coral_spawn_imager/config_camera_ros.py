#! /usr/bin/env python3

# pi camera configure wrapper for ROS
# read camera configuration from ros launch file
# apply them to picamera

from picamera import PiCamera
import rospy
import os
from collections import namedtuple

Config = namedtuple('Config', [
    'camera_index',
    'iso',
    'image_width',
    'image_height',
    'red_gain',
    'blue_gain',
    'awb_mode',
    'shutter_speed',
    'exposure_mode'
])

# from the ros parameter server, requires launching the node
# with the correct launch file
def read_ros_param(launch_file=None):

    # TODO if ros parameter server is not on, get from launch file?

    # get camera configuration parameters
    camera_index = rospy.get_param("/camera_index", 1)
    image_height = rospy.get_param("/image_height", 1920)
    image_width = rospy.get_param("/image_width", 1080)
    iso = rospy.get_param("/iso", 400)
    awb_mode = rospy.get_param("/awb_mode", 'auto')
    red_gain = rospy.get_param("/red_gain", -1.0)
    blue_gain = rospy.get_param("/blue_gain", -1.0)
    shutter_speed = rospy.get_param("/shutter_speed", 10000)
    exposure_mode = rospy.get_param("/exposure_mode", 'auto')

    # package into namedtuple
    conf_return = Config(
        camera_index = camera_index,
        iso = iso,
        image_height = image_height,
        image_width = image_width,
        red_gain = red_gain,
        blue_gain = blue_gain,
        awb_mode = awb_mode,
        shutter_speed = shutter_speed,
        exposure_mode = exposure_mode
    )
    return conf_return