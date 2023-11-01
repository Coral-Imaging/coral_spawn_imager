#! /usr/bin/env python3

""" short script to show camera preview over ethernet/qt """

from picamera2 import Picamera2, Preview
# from libcamera import controls
import time
from coral_spawn_imager.PiCamera2Wrapper import PiCamera2Wrapper
import os



SAVE_IMAGE_DIR_SSD = '/media/cslics04/cslics_ssd/images'
CAMERA_CONFIGURATION_FILE = '../../launch/camera_config_preview_2023.json'
CORAL_METADATA_FILE = '../../launch/coral_metadata.json'

print('creating picamera2 object & preview')
path = os.path.dirname(__file__) # get path to this file
cam = PiCamera2Wrapper(config_file=os.path.join(path, CAMERA_CONFIGURATION_FILE))

coral_metadata = cam.read_custom_metadata(os.path.join(path, CORAL_METADATA_FILE))

# wait for keyboard input to exit
# this should hold the window open 
# for as long as needed
value = 'o'
while not value == 'x':
    value = input("Type ``x`` to stop preview, ``y`` to take an image:\n")
    print(f'value entered {value}')
    if value == 'y':
        img_np, img_name, metadata = cam.capture_image(save_dir=SAVE_IMAGE_DIR_SSD)
        cam.update_metadata(metadata, coral_metadata)

        metadata_name = os.path.join(SAVE_IMAGE_DIR_SSD, img_name.split('.')[0] + '.json')
        cam.save_image(img_np, os.path.join(SAVE_IMAGE_DIR_SSD, img_name), metadata, metadata_name)
        # cam.capture_image('main')
        # img_name = 'test.png'
        print(f'captured image {img_name} in {SAVE_IMAGE_DIR_SSD}')



print('Closing preview')
# cam.stop_preview()
cam.stop_preview(1)



# note: all settings should be on auto


