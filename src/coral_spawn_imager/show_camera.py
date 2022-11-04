#! /usr/bin/env python3

""" short script to show camera preview over ethernet/qt """

from picamera2 import Picamera2, Preview
# from libcamera import controls
import time


print('creating picamera2 object & preview')
cam = Picamera2()
preview_config = cam.create_preview_configuration()
cam.configure(preview_config)

cam.start_preview(Preview.QT)


print('starting camera')
cam.start()
time.sleep(2)



# cam.start_preview(Preview.QT)

# wait for keyboard input to exit
# this should hold the window open 
# for as long as needed
value = 'o'
while not value == 'x':
    value = input("Type ``x`` to stop preview:\n")
    print(f'value entered {value}')

print('Closing preview')
cam.stop_preview()



# note: all settings should be on auto


