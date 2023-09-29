#!/usr/bin/env python3

""" short script to show camera preview over ethernet using PiCamaera2 wrapper with keyboard input for remote focus """
# added image capture

import os
from coral_spawn_imager.PiCamera2Wrapper import PiCamera2Wrapper
import time

print('creating PiCamera2Wrapper object')

# remote preview setup
CAMERA_CONFIGURATION_FILE = '../../launch/camera_config_preview.json'
# SAVE_IMAGE_DIR_SSD = '/media/cslics04/cslics_ssd/images'
SAVE_IMAGE_DIR_SSD = '/home/cslics04/images'
CORAL_METADATA_FILE = '../../launch/coral_metadata.json'

path = os.path.dirname(__file__) # get path to this file
picam = PiCamera2Wrapper(config_file=os.path.join(os.path.join(path, CAMERA_CONFIGURATION_FILE)))
coral_metadata = picam.read_custom_metadata(os.path.join(path, CORAL_METADATA_FILE))

# TODO filter input for remote focus change, up or down?
value = 'o'
print('Input format:')
print('r: increment focus')
print('t: decrement focus')
print('g: get focus value')
print('y: capture an image')
print('x: stop preview, end camera operation')

focus_increment = 100
while not value == 'x':
    value=input('Type input r, t, g, y, or x:\n')
    # print(f'value entered {value}')
    
    if value == 'r':
        print(f'increment focus')
        focus = picam.get_remote_focus()
        picam.set_remote_focus(focus + focus_increment)
        
    elif value == 't':
        print(f'decrement focus')
        focus = picam.get_remote_focus()
        picam.set_remote_focus(focus - focus_increment)
        
    elif value == 'g':
        print(f'get focus value')
        print(f'focus value: {picam.get_remote_focus()}')
        
    elif value == 'y':
        print(f'capture an image')
        img_np, img_name, metadata = picam.capture_image(save_dir=SAVE_IMAGE_DIR_SSD)
        picam.update_metadata(metadata, coral_metadata)
        picam.save_image(img_np, os.path.join(SAVE_IMAGE_DIR_SSD, img_name), metadata)
        
    elif value == 'x':
        print(f'stop preview')
    else:
        print(f'invalid input, try again')
    
print('closing preview, ending camera operation')
picam.end_camera_operation()