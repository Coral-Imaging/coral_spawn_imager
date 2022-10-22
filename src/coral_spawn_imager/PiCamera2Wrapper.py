#!/usr/bin/env python3

""" picamera 2 wrapper """

from picamera2 import Picamera2, Preview
# from libcamera import ColorSpace
import time
import os
from PIL import Image as pil_image
import datetime
from pprint import *
import io
from libcamera import controls
import matplotlib.pyplot as plt
import cv2 as cv
import numpy as np


# documentation: 
# https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
# github: 
# https://github.com/raspberrypi/picamera2

from coral_spawn_imager.config_camera2_json import read_json_config
# from coral_spawn_imager.config_camera_ros import read_ros_param

class PiCamera2Wrapper:

    IMAGE_WIDTH_DEFAULT = int(4056/4)      # pixels
    IMAGE_HEIGHT_DEFAULT = int(3040/4)     # pixels
    FPS_DEFAULT = 10.0              # frames per second
    ANALOGUE_GAIN_DEFAULT = 4.0     # analogue gain (1-6?)
    EXPOSURE_TIME_DEFAULT = 60000   # exposure time in microseconds
    INITIAL_PREVIEW_DURATION = 10   # initial preview time before turning off preview
    
    EXPOSURE_LIMITS = (114, 239542228)  # microseconds
    # IMAGE_BUFFER_COUNT = 2         # buffer count, # of images allowed to memory (see 4.2.1.3 on docs)

    def __init__(self,
                 config_file: str = None,
                 camera_index: int = 1,
                 image_width: int = IMAGE_WIDTH_DEFAULT,
                 image_height: int = IMAGE_HEIGHT_DEFAULT,
                 gain: float = ANALOGUE_GAIN_DEFAULT,
                 exposure_mode: str = 'auto',
                 awb_mode: str = 'auto',
                 exposure_time: int = EXPOSURE_TIME_DEFAULT,
                 preview_type: str = 'remote'):


        # TODO read in config file from config_camera2_json.py
        
        # set default camera configurations
        self.camera_index = camera_index
        self.camera = Picamera2()
        self.preview_config = self.camera.create_preview_configuration()
        self.capture_config = self.camera.create_still_configuration()

        if preview_type == 'remote':
            # allows port-forwarding for viewing remotely, but less efficient
            self.camera.start_preview(Preview.QT) # QT costly, but designed for remote viewing
        elif preview_type == 'null':
            # no preview
            self.camera.start_preview(Preview.NULL)
        else:
            # default to be run when running picamera locally
            self.camera.start_preview(Preview.QTGL)

        # configure camera
        self.camera.configure(self.preview_config)
        
        # config file for camera:
        if config_file is not None:
            conf = read_json_config(config_file)
            self.apply_conf(conf)
            self.camera.start()
        else:

            # set image resolution (must be done before camera.start())
            self.set_image_resolution(image_width, image_height) # setting image resolution must be called before camera.start()
            self.camera.start()

            # set auto white balance
            self.set_awb()
            with self.camera.controls as ctrl:
                ctrl.AeEnable = True
            # controls to take effect
            time.sleep(2)
        print('init end')

    
    def apply_conf(self, conf):
        print('applying config')
        self.camera_index = conf.camera_index
        self.image_width = conf.image_width
        self.image_height = conf.image_height
        self.set_image_resolution(self.image_width, self.image_height) # setting image resolution must be called before camera.start()
        self.set_exposure_mode(conf.ae_enable) # TODO aeconstraint mode not yet working
        self.set_gain(conf.gain)
        self.set_awb(conf.awb_enable, conf.awb_mode, conf.red_gain, conf.blue_gain)
        self.set_contrast(conf.contrast)
        self.set_exposure_time(conf.exposure_time)
        # self.set_exposure_value() # not yet implemented
        # self.set_fps() # todo - receive from conf
        # print('config frame duration')
        # self.set_frame_duration_limits(conf.frame_duration_limits_min, conf.frame_duration_limits_max)
        # print('config noise reduction')
        # self.set_noise_reduction_mode(conf.noise_reduction_mode)
        self.set_saturation(conf.saturation)
        self.set_sharpness(conf.sharpness)
        
        time.sleep(2)



    # def show_preview(self, duration: int = 10, preview_type='remote'):
    #     # NOTE: only seems to be working for local
    #     print('starting preview')
    #     self.camera.start_preview(Preview.QT)
    #     if duration > 0:
    #         time.sleep(duration)
    
    #     print('stopping preview')
    #     self.camera.stop_preview()


    def stop_preview(self, duration: int = 10):
        # might need Preview.DRM for non X Windows?
        # start_preview needs to be called before start
        # self.camera.start_preview(Preview.QTGL)
        if duration > 0:
            # show preview for duration (seconds)
            time.sleep(duration)
            self.camera.stop_preview()
        else:
            pass
        # TODO how to stop/close the preview?


    def set_image_resolution(self, width: int, height: int, stream: str='preview'):
        # for main/high-res images, note: must be run before camera.start()
        # width, height must be no less than 64
        self.camera.preview_configuration.main.size = (width, height)
        self.camera.preview_configuration.align()
        self.camera.configure(stream)


    def set_fps(self, fps: float):
        self.camera.video_configuration.controls.FrameRate = fps
        self.camera.configure("video")


    # def set_frame_duration_limits(self, frame_duration_min, frame_duration_max):
    #     self.camera.set_controls({"FrameDurationLimits": (frame_duration_min, frame_duration_max)})


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

        if (red_gain is None and blue_gain is None) or (red_gain < 0.0 and blue_gain < 0.0):
            # automatic control
            control = {'AwbEnable': awb_enable,
                       'AwbMode': awb_mode_enum[awb_mode]}
        else:
            control = {'ColourGains': (red_gain, blue_gain)}
            # setting these automatically disables AWB
        self.camera.set_controls(control)
        # there will be a delay of several frames before the controls take effect, thus we sleep for 3 seconds to allow the controls to take effect
        time.sleep(2)
    

    def get_expsosure_mode(self):
        controls = self.camera.camera_controls
        aeEnable = controls['AeEnable']
        aeConstraintMode = controls['AeConstraintMode']
        return aeEnable, aeConstraintMode

    
    def set_exposure_mode(self, 
                          ae_enable=None):
                        #   ae_constraint_mode=None):

        # ae_mode_enum = {'Normal': controls.AeConstraintModeEnum.Normal,
        #                  'Highlight': controls.AeConstraintModeEnum.Highlight,
        #                  'Shadows': controls.AeConstraintModeEnum.Shadows,
        #                  'Custom': controls.AeConstraintModeEnum.Custom}

        if ae_enable is not None:
            if type(ae_enable) is bool:
                self.camera.set_controls({'AeEnable': ae_enable})
            else:
                raise TypeError('ae_enable is not a valid bool')
        
        # if ae_constraint_mode is not None:
        #     self.camera.set_controls({"AeConstraintMode": ae_mode_enum[ae_constraint_mode]})

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


    def get_contrast(self):
        metadata = self.camera.capture_metadata()
        return metadata['Contrast']


    def set_contrast(self, contrast: float = 1.0):
        with self.camera.controls as controls:
            controls.Contrast = contrast


    def get_noise_reduction_mode(self):
        metadata = self.camera.capture_metadata()
        return metadata['NoiseReductionMode']


    # def set_noise_reduction_mode(self, noise_reduction_mode):
    #     self.camera.set_controls({"NoiseReductionMode": noise_reduction_mode})
        # with self.camera.controls as controls:
        #     controls.NoiseReductionMode = noise_reduction_mode


    def get_saturation(self):
        metadata = self.camera.capture_metadata()
        return metadata['Saturation']


    def set_saturation(self, saturation):
        self.camera.set_controls({'Saturation': saturation})


    def get_sharpness(self):
        metadata = self.camera.capture_metadata()
        return metadata['Sharpness']


    def set_sharpness(self, sharpness):
        self.camera.set_controls({'Sharpness': sharpness})


    def get_params(self):
        print('getting parameters')
        metadata = self.camera.capture_metadata()
        config = self.camera.camera_config
        controls = self.camera.camera_controls
        properties = self.camera.camera_properties
        print('parameters received')
        # merge the two
        # params = controls.update(config)
        params = {'metadata': metadata, 
                  'config': config, 
                  'controls': controls,
                  'properties': properties}
        return params


    def print(self):
        # print('sensor modes:')
        # pprint(self.camera.sensor_modes)
        
        params = self.get_params()
        print('CAMERA properties: ')
        pprint(params['properties'])
        print()
        print('CAMERA configuration: ')
        pprint(params['config'])
        print()
        print('CAMERA controls: ')
        pprint(params['controls'])
        print()
        print('CAMERA metadata:')
        pprint(params['metadata'])
        print()

    
    def capture_image_np(self):
        """ capture image as numpy array"""
        return self.camera.capture_array('main')
    

    def capture_image_pil(self):
        """ capture image as pil image """
        return self.camera.capture_image('main')


    def capture_with_config(self):
        """ switches mode from fast framerate for display/preview to 
            slow framerate for hi-res image capture 
            automatically switches back to preview mode after capture """
        img_np = self.camera.switch_mode_and_capture_array(self.capture_config, 'main')
        return img_np


    def capture_image_file(self, img_name, save_dir):
        """
        capture image and save directly to file
        """
        self.camera.switch_mode_and_capture_file(self.capture_config,
                                                 os.path.join(save_dir, img_name))


    def capture_image(self, img_name=None, save_dir=None, format='jpeg'):
        """
        capture and return a single image using numpy arrays
        """
        if img_name is None:
            datestr = datetime.datetime.now()
            img_name = datestr.strftime("%Y%m%d_%H%M%S_%f") + '_img.' + format

        if save_dir is not None:
            img_name = os.path.join(save_dir, img_name)

        img = self.capture_with_config()
        metadata = self.camera.capture_metadata()

        return img, img_name, metadata

    def save_image(self, img, img_name):

        base_path = os.path.basename(img_name)
        if not os.path.isdir(base_path):
            os.mkdir(base_path)
        
        if isinstance(img, pil_image.Image):
            img.save(img_name)
        elif isinstance(img, np.ndarray):
            cv.imwrite(img_name, img)
        else:
            plt.imsave(img_name, img)


if __name__ == "__main__":

    print('PiCamera2Wrapper.py')

    picam = PiCamera2Wrapper(config_file='config_camera2.json')
    print('picam print')
    picam.print()

    # test image capture:
    img, img_name, metadata = picam.capture_image()
    pprint(metadata)
    print(img_name)
    print(img.shape)

    # test normal low-res image capture
    # img_lo = picam.capture_image_np()
    # print(img_lo.shape)

    # set AWB to manual/set red/blue gains
    # red_gain = 1.0
    # blue_gain = 4.0
    # picam.set_awb(red_gain=red_gain, blue_gain=blue_gain)
    # # preview should automatically change
    # time.sleep(2)
    # picam.set_awb(awb_enable=True)

    # picam.set_exposure_time(1000)
    # time.sleep(2)
    # picam.set_exposure_time(40000)
    # time.sleep(2)
    # preview should change back
    # time.sleep(10)
    # print('set higher res')
    # picam.set_main_image_resolution(1920, 1080)
    # picam.print()
    # print('waiting')
    # time.sleep(5)
    # print('stopping preview in 5 seconds')
    # picam.stop_preview(5)
    # picam.show_preview(20)

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

    import code
    code.interact(local=dict(globals(), **locals()))