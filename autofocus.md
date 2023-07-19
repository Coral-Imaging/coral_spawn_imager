
# Remote Focus & Autofocus

Hardware:
- using Arducam Arducam 12MP IMX477 Motorized Focus Camera for Raspberry Pi
- https://www.arducam.com/product/arducam-12mp-imx477-motorized-focus-high-quality-camera-for-raspberry-pi/
- https://www.arducam.com/raspberry-pi-camera/autofocus/
- 

Software:
- github repo:
- https://github.com/ArduCAM/RaspberryPi
- specifically, this file/install functions:
- https://github.com/ArduCAM/RaspberryPi/tree/master/Motorized_Focus_Camera
- `python3 FocuserExample.py -i 10`

- Note: had version control issues with this repo
- Had to:
-         sudo pip uninstall opencv-python
-         sudo apt-get install libopencv-dev
-         sudp apt-get install python3-opencv
