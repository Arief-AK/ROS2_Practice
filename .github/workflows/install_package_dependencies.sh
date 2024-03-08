#!/bin/sh

# Extracting arguments
ROS_DISTRO=$1

sudo apt-get update && sudo apt-get install python3-colcon-common-extensions
which colcon && which rosdep
sudo rosdep init && rosdep update
rosdep "install -i --from-path src --rosdistro ${ROS_DISTRO} -y"