#!/bin/sh

GITHUB_ACTOR=$1
ROS_DISTRO=$2

echo "${GITHUB_ACTOR} is attempting to install ROS2 dependencies"
sudo apt-get update && sudo apt-get install python3-colcon-common-extensions
which colcon && which rosdep
sudo rosdep init && rosdep update
rosdep "install -i --from-path src --rosdistro ${ROS_DISTRO} -y"
echo "${GITHUB_ACTOR} has installed ROS2 dependencies"