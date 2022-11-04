# Coral Spawn Imager

As part of the Reef Restoration and Adaptation Program (RRAP), the Coral Spawn and Larvae Imaging Camera System (CSLICS) is a camera system aimed at capturing images of coral spawn for research and analysis. This coral_spawn_imager repo is specifically the code for camera control, image capture and scheduling.


## Hardware

- Raspberry Pi Model 4B (<2 GB)
- Raspberry Pi High Quality Camera
- Microscope lens (TODO: get specifications)
- external SSD named `cslics_ssd` connected to the Pi


## Installation Requirements

Using `pip install X`:

- Raspberry Pi OS version 11 (Bullseye), 64-bit
- ROS Noetic on Raspberry Pi, 64-bit
- matplotlib==3.6.1
- numpy==1.19.5
- picamera2==0.3.5
- Pillow==9.3.0
- rospy==1.15.14
- tifffile==2022.10.10 (only for metadata_tiff.py, experimental)


## Installation Instructions for Pi Setup

- The simplest and fastest route of replication is to write a new image of the existing OS; however, to start from scratch, ceate a new install of Raspberry Pi OS Version 11, 64-bit using the Pi Imager.
- Install ROS Noetic: https://wiki.qut.edu.au/pages/viewpage.action?spaceKey=cyphy&title=Technical+Handbook
- Install QCR Bring-up services and daemon: `sudo apt install ros-noetic-perception ros-noetic-robot-bringup ros-noetic-roscore-daemon`
- 


- Create a ROS workspace:

      mkdir ~/cslics_ws/src
      cd ~/cslics_ws/src
 
- Clone this repo in the src directory. Note: might have to use the HTTPS if you don't have SSH access to the repo.

      git clone git@github.com:doriantsai/coral_spawn_imager.git

- Compile the catkin workspace

      cd ~/cslics_ws
      catkin_make

- Source the workspace and packages within as below, though it is recommended to add this line to the bottom of your .bashrc file so you never have to type this line again:

      source ~/cslics_ws/devel/setup.bash

- coral_spawn_imager should be operational. 


## Usage Instructions

Running the camera trigger:

      roslaunch coral_spawn_imager camera_trigger.py


## Camera Configuration Files

See `camera_config_seasim.json` as an example:

            {
                  "preview_type": "remote",
                  "camera_index": 3,
                  "image_width": 1920,
                  "image_height": 1080,
                  "AeConstraintMode": "Shadows",
                  "AeEnable": 1,
                  "AeExposureMode": "Short",
                  "AeMeteringMode": "Matrix",
                  "AnalogueGain": 20.0,
                  "AwbEnable": 1,
                  "AwbMode": "Auto",
                  "Brightness": 5.0,
                  "ColourGains_Red": 2.3,
                  "ColourGains_Blue": 2.3,
                  "Contrast": 1.0,
                  "ExposureTime": 8000,
                  "ExposureValue": 0.0,
                  "FrameDurationLimits_Min": 1000,
                  "FrameDurationLimits_Max": 4000,
                  "NoiseReductionMode": "Fast",
                  "Saturation": 1.0,
                  "Sharpness": 4.0
            }



## Headless Auto-Run Setup

- When running headless, we want the camera trigger node to launch automatically on start-up. For this, we use the qcr robot bringup. Control for this is in `/etc/qcr/qcr-env.bash` and editting the file as follows (requires sudo):

      export ROS_WORKSPACE=/home/cslics04/cslics_ws/devel/setup.bash
      export QCR_ROBOT_LAUNCH="roslaunch coral_spawn_imager camera_bringup.launch"

To start the service:

      sudo service robot-bringup start

To stop the service:

      sudo service robot-bringup stop

To restart the service:

      sudo service robot-bringup restart

To view the output of the service:

      journalctl -u robot-bringup --follow --lines 500
