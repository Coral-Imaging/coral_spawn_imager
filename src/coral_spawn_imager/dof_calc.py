#!/usr/bin/env python3

""" depth of field calculation
Determine the near and far DoF so that we can estimate the volume for CSLICS calculations

lens info:
https://aico-lens.com/product/12mm-10mp-f2-8-m12-cctv-board-4k-lens-ach1228mac/
camera info:
https://www.uctronics.com/raspberry-pi-high-quality-camera-autofocus-motorized-focus-imx477.html
formulae:
https://www.omnicalculator.com/other/depth-of-field
"""

import numpy as np

# parameters

# sensor size
width_pix = 4056
height_pix = 3040
pix_size = 1.55 # um
sensor_width = width_pix * pix_size / 1000.0 # mm
sensor_height = height_pix * pix_size / 1000.0 # mm

# focal length
f = 12 # mm

# aperture number, N
n = 2.8
print(f'N = {n}')

# circle of confusion limit, c
# there's a more involved way of determining this number (TODO later), but in practice, most people just take a value from 0.01mm to 0.2mm
# smaller c with smaller sensor size results in smaller DoF
# NOTE: calculating the circle of confusion for photography is different than calculating
# the CoC for 
# See "Determining a circle of confusion diameter from the object field" Wiki
# https://en.wikipedia.org/wiki/Circle_of_confusion
# for now, we take the smallest in the range
c = 0.1 # mm # hard to justify!

# hyper focal distance, H
# the focusing distance at which we get maximum depth of field
H = f + f**2 / (c * n)
# print(f'H = {H}')
# focusing distance, u
# for CSLICS, this is practically the working distance of the camera
u = 50 # mm

# depth of field, far limit
DoF_far = (H * u) / (H - (u - f))
print(f'DoF_far = {DoF_far} mm')

# depth of field, near limit
DoF_near = (H * u) / (H + (u - f))
print(f'DoF_near = {DoF_near} mm')

# depth of field
DoF = DoF_far - DoF_near
print(f'DoF = {DoF} mm')

# for horizontal and vertical FOV in mm:
wd = u # working distance is 50mm, same as focusing distance for our application
# 1.33 for refraction through water, although in reality we image through air and acrylic, and then water
# the pure water case represents a worst case
hfov = wd * sensor_height / (1.33 * f)
vfov = wd * sensor_width / (1.33 * f)

print(f'horizontal FOV = {hfov}')
print(f'vertical FOV = {vfov}')

# we can approximate the frustum as a rectangular prism, since the angular FOV is not that wide
focus_volume = hfov * vfov * DoF # mm^3
print(f'focus volume = {focus_volume} mm^3')

# given the tank volume: 500L
tank_volume = 500

# multiplication factor for sample volume to tank:
# 1L = 1000 mL = 1000000 mm^3 = 1E6
scale_factor = tank_volume / (focus_volume / 1000000)
print(f'scale factor = {scale_factor}')
# expecting a value of the order of magnitude of 5.0E4

# as expected, is a pretty big number... need to check


# possibility to use c as the airy disk?
# airy_disk in cameras
# lambda / d, where lamda is the wavelength, and d is the aperture pix_size
# with a variety of reasonable assumptions
# x = 1.22 * lambda * f/d, f/d = N
# x = 1.22 * (420) * n / 1000 # shortest visible light 420 nm
# um smallest separation that can be resolved, 1.4 um //
# print(x)

# or Zeiss' formula c = d/1730 or d/1000, where d = diagonal measure of camera sensor
# d = np.sqrt(sensor_height**2 + sensor_width**2)
# print(d)
# z = d / 1000.0
# print(z)