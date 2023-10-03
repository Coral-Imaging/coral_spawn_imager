#!/usr/bin/env python3

""" short script to show camera preview over ethernet using PiCamaera2 wrapper with keyboard input for remote focus """

import os
from coral_spawn_imager.PiCamera2Wrapper import PiCamera2Wrapper
import time

print('creating PiCamera2Wrapper object')

# remote preview setup
CAMERA_CONFIGURATION_FILE = '../../launch/camera_config_preview.json'
picam = PiCamera2Wrapper(config_file=os.path.join(CAMERA_CONFIGURATION_FILE))
# should start camera preview?

# TODO filter input for remote focus change, up or down?
value = 'o'
print('Input format:')
print('r: increment focus')
print('t: decrement focus')
print('g: get focus value')
print('x: stop preview, end camera operation')

focus_increment = 100
while not value == 'x':
    value=input('Type input r,t,g, or x:\n')
    print(f'value entered {value}')
    
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
        
    elif value == 'x':
        print(f'stop preview')
    else:
        print(f'invalid input, try again')
    
print('closing preview, ending camera operation')
picam.end_camera_operation()