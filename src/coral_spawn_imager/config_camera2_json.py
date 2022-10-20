#! /usr/bin/env python3

"""
read configuration file of camera settings, return dictionary
updated for picamera2
"""

import json
from pathlib import Path
from collections import namedtuple


# configuration file definition
Config = namedtuple('Config', [
    'camera_index',
    'image_width',
    'image_height',
    'ae_constraint_mode',
    'ae_enable',
    'ae_exposure_mode',
    'ae_metering_mode',
    'gain',
    'awb_enable',
    'awb_mode',
    'brightness',
    'red_gain',
    'blue_gain',
    'contrast',
    'exposure_time',
    'exposure_value',
    'frame_duration_limits_min',
    'frame_duration_limits_max',
    'noise_reduction_mode',
    'saturation',
    'sharpness'
])

def read_json_config(config_file: str = None):
    # read config file from json file
    if config_file is None:
        config_file = Path(__file__).parent / 'config_camera2.json'
        print(config_file)
    else:
        config_file = Path(config_file)
    if config_file.exists():
        with config_file.open('r') as fp:
            conf = json.load(fp)
    else:
        # create a template config file and exit with exception
        from json import dump as json_dump
        with config_file.open('w') as fp:
            json_dump({
                    "camera_index": 1,
                    "image_width": 1920,
                    "image_height": 1080,
                    "AeConstraintMode": "Shadows",
                    "AeEnable": 1,
                    "AeExposureMode": "Long",
                    "AeMeteringMode": "Matrix",
                    "AnalogueGain": 6.0,
                    "AwbEnable": 1,
                    "AwbMode": "Auto",
                    "Brightness": 0.0,
                    "ColourGains_Red": 2.3,
                    "ColourGains_Blue": 2.3,
                    "Contrast": 1.0,
                    "ExposureTime": 10000,
                    "ExposureValue": 0.0,
                    "FrameDurationLimits_Min": 1000,
                    "FrameDurationLimits_Max": 80000,
                    "NoiseReductionMode": "Fast",
                    "Saturation": 1.0,
                    "Sharpness": 1.0
            }, fp, indent=4)
        raise FileNotFoundError(f"No config file available, one has been created at '{config_file}'. Please fill it out.")

    # extract configuration file values
    camera_index = int(conf.get('camera_index', -1))

    # resolution_def = (2592, 1944)
    image_width_def = 2592
    image_height_def = 1944
    image_width = conf.get('image_width', image_width_def)
    image_height = conf.get('image_height', image_height_def)
    
    # auto exposure setup
    ae_constraint_mode = str(conf.get('AeConstraintMode', 'Shadows'))
    ae_enable = bool(conf.get('AeEnable', True))
    ae_exposure_mode = str(conf.get('AeExposureMode', 'Long'))
    ae_metering_mode = str(conf.get('AeMeteringMode', 'Matrix'))

    # image gain/iso
    gain = float(conf.get('AnalogueGain', 1.0))

    # white balance setup
    awb_enable = bool(conf.get('AwbEnable', True))
    awb_mode = str(conf.get('AwbMode', 'Auto'))
    red_gain = float(conf.get('ColourGains_Red', -1.0))
    if red_gain < 0:
        red_gain = None
    blue_gain = float(conf.get('ColourGains_Blue', -1.0))
    if blue_gain < 0:
        blue_gain = None

    # other image settings
    brightness = float(conf.get('Brightness', 0.0))
    contrast = float(conf.get('Contrast', 1.0))
    exposure_time = int(conf.get('ExposureTime', 10000))
    exposure_value = float(conf.get('ExposureValue', 0.0))

    frame_duration_limits_min = float(conf.get('FrameDurationLimits_Min', 1000))
    frame_duration_limits_max = float(conf.get('FrameDurationLimits_Max', 80000))

    noise_reduction_mode = str(conf.get('NoiseReductionMode', 'Fast'))
    saturation = float(conf.get('Saturation', 1.0))
    sharpness = float(conf.get('Sharpness', 1.0))
    
    # HACK STOPPED HERE
    # package into namedtuple
    conf_return = Config(
    camera_index = camera_index,
    image_width = image_width,
    image_height = image_height,
    ae_constraint_mode = ae_constraint_mode,
    ae_enable = ae_enable,
    ae_exposure_mode = ae_exposure_mode,
    ae_metering_mode = ae_metering_mode,
    gain = gain,
    awb_enable = awb_enable,
    awb_mode = awb_mode,
    brightness = brightness,
    red_gain = red_gain,
    blue_gain = blue_gain,
    contrast = contrast,
    exposure_time = exposure_time,
    exposure_value = exposure_value,
    frame_duration_limits_max = frame_duration_limits_max,
    frame_duration_limits_min = frame_duration_limits_min,
    noise_reduction_mode = noise_reduction_mode,
    saturation = saturation,
    sharpness = sharpness)

    return conf_return


if __name__ == '__main__':
    config = read_json_config()
    print('Read configuration: ')
    # print(config)
    print(config)