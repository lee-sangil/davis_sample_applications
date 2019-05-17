# DAVIS Sample Applications

This repo contains a few sample standalone applications for DAVIS 240C event camera.
Tested in Ubuntu 16.04LTS with OpenCV 3.4.6, libCAER 3.1.9.

* **davis_sensor_interface**: A very basic interface to DAVIS 240C event camera. Receives events and gray images and displays them using OpenCV.

* **davis_sensor_save_events**: Save the current events and gray images, and display the image using openCV.

* **davis_sensor_save_events_ros**: Save the current events, imu, vicon, and gray images, and display the image using openCV.

## Prerequisites

* OpenCV
* [libCAER](https://github.com/inivation/libcaer)
* [rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros)
* [vicon_bridge](https://github.com/ethz-asl/vicon_bridge)
## Installation

### 1. Required Dependencies

    sudo apt-get install libboost-all-dev libopencv-dev
  
### 2. Install CAER Library

In order to install libCAER 3.1.0 or newer, cmake 3.12.0 or newer is needed.

### 3. Build dvs_msgs

Build `davis_ros_driver` (https://github.com/uzh-rpg/rpg_dvs_ros) and `catkin_simple` package in the separated catkin workspace using `catkin build` described in the README.

### 4. Build vicon_bridge and DAVIS applications

Build `vicon_bridge` (https://github.com/ethz-asl/vicon_bridge) and the above application (https://github.com/lee-sangil/davis_sample_applications) using `catkin_make` in the same catkin workspace, not in the davis_ros_driver workspace.
