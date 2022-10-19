#!/usr/bin/env python3

""" picamera 2 wrapper """

from picamera2 import Picamera2, Preview
# from libcamera import ColorSpace
import time
import os
from PIL import Image
# import datetime
from pprint import *


# documentation: 
# https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
# github: 
# https://github.com/raspberrypi/picamera2

# from coral_spawn_imager.config_camera_json import read_json_config
# from coral_spawn_imager.config_camera_ros import read_ros_param

class PiCamera2Wrapper:

    IMAGE_WIDTH_DEFAULT = 4056      # pixels
    IMAGE_HEIGHT_DEFAULT = 3040     # pixels
    FPS_DEFAULT = 10.0              # frames per second
    ANALOGUE_GAIN_DEFAULT = 4.0     # analogue gain (1-6?)
    EXPOSURE_TIME_DEFAULT = 60000   # exposure time in microseconds
    
    EXPOSURE_LIMITS = (114, 239542228)  # microseconds
    def __init__(self,
                 config_file: str = None,
                 camera_index: int = 1,
                 image_width: int = IMAGE_WIDTH_DEFAULT,
                 image_height: int = IMAGE_HEIGHT_DEFAULT,
                 gain: float = ANALOGUE_GAIN_DEFAULT,
                 exposure_mode: str = 'auto',
                 awb_mode: str = 'auto',
                 exposure_time: int = EXPOSURE_TIME_DEFAULT):



        # set default camera configurations
        self.camera_index = camera_index
        self.camera = self.get_picamera()

        # set auto white balance
        self.set_awb()
        with self.camera.controls as ctrl:
            ctrl.AeEnable = True
        # controls to take effect
        time.sleep(2)
    

    def get_picamera(self, config=None):
        _picamera = Picamera2()
        if config is None:
            config = _picamera.create_preview_configuration()
        
        _picamera.configure(config)
        _picamera.start()
        time.sleep(2)
        return _picamera


    def show_preview(self, duration: int = 10):
        self.camera.start_preview(Preview.QTGL)
        if int > 0:
            # show preview for duration (seconds)
            time.sleep(duration)
            self.camera.stop_preview()
        # TODO how to stop/close the preview?



    def set_lowres_image_resolution(self, width: int, height: int):
        # for previews (eg. streaming across network for focusing purposes)
        self.camera.stop()
        self.camera.preview_configuration.enable_lores()
        self.camera.preview_configuration.lores.size = (width, height)
        self.camera.configure("preview")
        self.camera.start()


    def set_main_image_resolution(self, width: int, height: int):
        # for main/high-res images
        self.camera.stop()
        self.camera.preview_configuration.main.size = (width, height)
        self.camera.preview_configuration.main.format = 'RGB888'
        self.camera.preview_configuration.align()
        self.camera.configure("preview")
        self.camera.start()


    def set_fps(self, fps: float):
        self.camera.video_configuration.controls.FrameRate = fps
        self.camera.configure("video")


    def get_colour_gains(self):
        metadata = self.camera.capture_metadata()
        return metadata['ColourGains'] # (red_gain, blue_gain)


    def set_awb(self, awb_enable: bool = True, awb_mode: str = 'Auto', red_gain = None, blue_gain = None):

        awb_mode_enum = {'Auto': 0,
                         'Tungsten': 1,
                         'Fluorescent': 2,
                         'Indoor': 3,
                         'Daylight': 4,
                         'Cloudy': 5,
                         'Custom': 6}

        print('setting white balance')

        if red_gain is None and blue_gain is None:
            # automatic control
            control = {'AwbEnable': awb_enable,
                       'AwbModeEnum': awb_mode_enum[awb_mode]}
        else:
            control = {'ColourGains': (red_gain, blue_gain)}
            # setting these automatically disables AWB
        self.camera.set_controls(control)
        # there will be a delay of several frames before the controls take effect, thus we sleep for 3 seconds to allow the controls to take effect
        time.sleep(2)
    

    def get_exposure_time(self):
        metadata = self.camera.capture_metadata()
        return metadata['ExposureTime'] # ms

    def set_exposure_time(self, exposure_time: int = 10000):
        # set shutter time in ms
        with self.camera.controls as controls:
            controls.ExposureTime = exposure_time

    def get_gain(self):
        metadata = self.camera.capture_metadata()
        return metadata['AnalogueGain'] # 1-4?

    def set_gain(self, gain: float = 4.0):
        # aka iso
        with self.camera.controls as controls:
            controls.AnalogueGain = gain


    def get_params(self):
        # available sensor modes
        # sensor_modes = self.camera.sensor_modes
        # {'bit_depth': 12,
        # 'crop_limits': (0, 0, 4056, 3040),
        # 'exposure_limits': (114, 239542228),
        # 'format': SRGGB12_CSI2P,
        # 'fps': 10.0,
        # 'size': (4056, 3040),
        # 'unpacked': 'SRGGB12'}
        metadata = self.camera.capture_metadata()
        config = self.camera.camera_config
        controls = self.camera.camera_controls
        # merge the two
        # params = controls.update(config)
        params = {'metadata': metadata, 
                  'config': config, 
                  'controls': controls}
        return params


    def print(self):
        # print('sensor modes:')
        # pprint(self.camera.sensor_modes)
        print('camera configuration: ')
        config, controls = self.get_params()
        pprint(config)
        print('camera controls: ')
        pprint(controls)


if __name__ == "__main__":

    print('PiCamera2Wrapper.py')

    picam = PiCamera2Wrapper()
    picam.print()

    # print('set low res')
    # picam.set_lowres_image_resolution(420, 320)
    # picam.print()

    # print('set higher res')
    # picam.set_main_image_resolution(1920, 1080)
    # picam.print()

    picam.show_preview(5)

# set camera configuration, parameters of the camera that are set at runtime and cannot be changed until the camera is restarted
# image resolution for main stream & low-res stream
# color_space
# display
# transform

# generate preview configuration:
# picam2 = Picamera2()
# config_preview = picam2.create_preview_configuration()
# picam2.configure(config_preview)
# picam2.start()

# # generate high-res still image configuration:
# config_still = picam2.create_still_configuration()
# picam2.configure(config_still)
# picam2.start()

# set camera controls, parameters of the camera that can be changed during runtime
# brightness, contrast