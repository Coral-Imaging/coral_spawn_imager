#! /usr/bin/env python3

# pi camera wrapper object
# configure camera settings
# documentation for PiCamera module: https://picamera.readthedocs.io/en/release-1.13/api_camera.html

# import json
import os
from picamera import PiCamera
import datetime
import time
from PIL import Image
# import rospy
from io import BytesIO

from coral_spawn_imager.config_camera_json import read_json_config
from coral_spawn_imager.config_camera_ros import read_ros_param


class PiCameraWrapper:

    def __init__(self, 
                 config_file: str = None,
                 camera_index: int = 1,
                 image_width: int = 2592,
                 image_height: int = 1944,
                 iso: int = 400,
                 exposure_mode: str = 'auto',
                 shutter_speed: int = 10000,
                 awb_red_gain: float = -1.0,
                 awb_blue_gain: float = -1.0):
        # TODO document input units
        # shutterspeed in [mpip3s]
        # iso is image gain, (100-800)
        # init Camera object (wrapper for picam)

        PiCamera.CAPTURE_TIMEOUT = 266.0    # double how long we are willing to wait for the camera to capture

        if config_file == 'ROS':
            conf = read_ros_param()
            self.camera = self.get_picamera(conf.image_width, conf.image_height)
            self.apply_conf(conf)

        elif config_file is not None:
            conf = read_json_config(config_file)
            self.camera = self.get_picamera(conf.image_width, conf.image_height)
            self.apply_conf(conf)

        else:
            # get default camera configuration
            self.camera_index = camera_index
            self.camera = self.get_picamera(image_height, image_width)

            resolution = (image_height, image_width)
            if self.camera.resolution != resolution:
                # apply desired resolution
                self.camera.resolution = resolution

            self.camera.iso = iso
            self.camera.exposure_mode = 'auto'    
            self.camera.shutter_speed = shutter_speed
        
            if awb_red_gain > 0 and awb_blue_gain > 0:
                self.camera.awb_mode = 'off'
                self.camera.awb_gains = (awb_red_gain, awb_blue_gain)
            else:
                self.camera.awb_mode = 'auto'


    def apply_conf(self, conf):
        # apply config to picamera
        self.set_camera_index(conf.camera_index)
        self.set_resolution(conf.image_width, conf.image_height)
        self.set_awb_gains(conf.red_gain, conf.blue_gain)
        self.set_iso(conf.iso)
        self.set_shutter_speed(conf.shutter_speed)
        self.set_exposure_mode(conf.exposure_mode)


    def set_exposure_mode(self, exposure_mode: str = 'auto'):
        # TODO: valid exposure modes
        self.camera.exposure_mode = exposure_mode


    def turn_off_auto_adjust(self):
        """
        turn off auto adjustment (exposure mode to not)
        exposure mode to off
        awb off
        """
        self.camera.exposure_mode = 'off'
        self.camera.awb_mode = 'off'


    def reset_auto_adjust(self):
        """
        rest the camera to using auto-exposure and auto white balance.
        Undoes any fixed awb gains or custom auto adjust
        """
        self.camera.iso = 0
        self.camera.shutter_speed = 0
        self.camera.exposure_mode = 'auto'
        self.camera.awb_mode = 'auto'


    def set_shutter_speed(self, shutter_speed=None):
        if shutter_speed is None:
            return
        # framerate vs shutter_speed
        # current_framerate = self.camera.framerate
        new_framerate = int(1000000 / float(int(shutter_speed))
        )
        self.camera.framerate = min(new_framerate, 30)
        self.camera.shutter_speed = shutter_speed

    
    def set_camera_index(self, camera_index):
        self.camera_index = camera_index


    def set_iso(self, iso: int = None):
        if iso is None:
            return
        valid_iso_range = {0, 100, 200, 320, 400, 500, 640, 800}
        if iso not in valid_iso_range:
            print(f'warning: iso not in valid iso range: {iso}, default set to 0')
            iso = 0
        self.camera.iso = iso


    def set_resolution(self, width=None, height=None):
        # TODO valid resolutions for HQ PiCam
        if width is None or height is None:
            return
        self.camera.resolution = (width, height)

    
    def set_awb_gains(self, red=None, blue=None):
        # valid gains from 0.0 - 8.0
        if red is None and blue is None:
            self.camera.awb_mode = 'auto'
            return
        if red is None:
            red = self.camera.awb_gains[0]
        if blue is None:
            blue = self.camera.awb_gains[1]
        self.camera.awb_mode = 'off'
        if (red > 0.0 and red < 8.0) and (blue > 0.0 and blue < 8.0):
            self.camera.awb_gains = (red, blue)
        else:
            print(f'Warning: awb_gains red and blue are (red={red}, blue={blue}), must be between 0.0-8.0')
            print(f'Setting default awb_gains to (0, 0)')
            self.camera.awb_gains = (1.0, 1.0)

    
    def capture_image(self, img_name=None, save_dir=None, format='jpeg'):
        """
        capture and return a single image using PIL
        """
        if img_name is None:
            datestr = datetime.datetime.now()
            img_name = datestr.strftime("%Y%m%d_%H%M%S") + '_img.' + format

        if save_dir is not None:
            img_name = os.path.join(save_dir, img_name)

        # TODO right now, returning image name vs image is not good
        # TODO make consistent output

        self.camera.capture(img_name, format=format)
        img_pil = Image.open(img_name)

        return img_pil, img_name

    def capture_image_stream(self, stream=None, img_name=None, save_dir=None, format='jpeg'):
        """
        capture image from io stream and return a single image using PIL 
        """
        if img_name is None:
            datestr = datetime.datetime.now()
            img_name = datestr.strftime("%Y%m%d_%H%M%S") + '_img.' + format

        if save_dir is not None:
            img_name = os.path.join(save_dir, img_name)

        if stream is None:
            stream = BytesIO()
        
        self.camera.capture(stream, format)
        stream.seek(0)
        img_pil = Image.open(stream)
        
        return img_pil, img_name


        
    
    def get_picamera(self, image_width=None, image_height=None):
        resolution = (image_width, image_height)
        __picamera = PiCamera(resolution=resolution)
        time.sleep(2) # sleep for 2 seconds, camera warm-up time
        return __picamera


    def get_params(self):
        # get picamera settings
        resolution = self.camera.resolution
        cam_params = {
            'sensor_mode': self.camera.sensor_mode,
            'zoom': self.camera.zoom,
            'roi': {
                'x': self.camera.zoom[0],
                'y': self.camera.zoom[1],
                'w': self.camera.zoom[2],
                'h': self.camera.zoom[3],
            },
            'iso': self.camera.iso,
            'exposure_mode': self.camera.exposure_mode,
            'shutter_speed': {
                'nominal': self.camera.shutter_speed,
                'actual': self.camera.exposure_speed
            },
            'awb_mode': self.camera.awb_mode,
            'awb_gains': {
                'red': {
                    'numerator': self.camera.awb_gains[0].numerator,
                    'denominator': self.camera.awb_gains[0].denominator
                },
                'blue': {
                    'numerator': self.camera.awb_gains[1].numerator,
                    'denominator': self.camera.awb_gains[1].denominator
                }
            },
            'resolution': {
                'width': resolution[0],
                'height': resolution[1]
            },
            'analog_gain': {
                'numerator': self.camera.analog_gain.numerator,
                'denominator': self.camera.analog_gain.denominator
            },
            'digital_gain': {
                'numerator': self.camera.digital_gain.numerator,
                'denominator': self.camera.digital_gain.denominator
            },
            'exposure_compensation': self.camera.exposure_compensation,
            'brightness': self.camera.brightness,
            'contrast': self.camera.contrast,
            'saturation': self.camera.saturation,
            'sharpness': self.camera.sharpness,
            'hflip': self.camera.hflip,
            'vflip': self.camera.vflip,
            'rotation': self.camera.rotation,
            'image_denoise': self.camera.image_denoise,
            'revision': self.camera.revision
        }
        return cam_params



if __name__ == "__main__":

    # main function
    # create camera object
    # set parameters
    # capture an image

    # camera_index = 1
    # resolution = (2592, 1944)
    # iso = 100
    # shutter_speed = 10000 # [us]
    # awb_gains = (0.5, 1.5)


    print('setting picamerawrapper')
    PiCam = PiCameraWrapper('ROS')

    # PiCam = PiCameraWrapper(camera_index, resolution, iso, shutter_speed, awb_gains)
    # config_file = 'config_camera.json'
    # PiCam = PiCameraWrapper(config_file=config_file)

    # check camera settings:
    param = PiCam.get_params()
    print(param['resolution'])
    print(param['exposure_mode'])
    print(param)

    # capture image
    print('capturing image')
    img_pil, img_name = PiCam.capture_image(save_dir=None)

    # set parameters (turn off auto)
    PiCam.turn_off_auto_adjust()
    param_new = PiCam.get_params()
    print(param_new['exposure_mode'])


    print('done')

    import code
    code.interact(local=dict(globals(), **locals()))