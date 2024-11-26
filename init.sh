#!/bin/bash
cd src
git clone --branch ros2 https://github.com/ros-drivers/usb_cam.git
cd ..
colcon build