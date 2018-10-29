# DAVIS Sample Applications

This repo contains a few sample standalone applications for DAVIS 240C event camera.
Tested in Ubuntu 16.04LTS and OpenCV 3.4.3.

* **davis_sensor_interface**: A very basic interface to DAVIS 240C event camera. Receives events and gray images and displays them using OpenCV.

* **davis_sensor_save_events**: Save the current events and gray images, and display the image using openCV.

## Installation

### 1. Required Dependencies

    sudo apt-get install libboost-all-dev libopencv-dev
  
### 2. Build Library

    chmod 755 install_libcaer.sh
    ./install_libcaer.sh
  
this will compile a library `libcaer`, which can be linked from external projects. However, for this
OpenCV and Boost need to be installed.
