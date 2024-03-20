#!/bin/sh

GITHUB_ACTOR=$1
ROS_DISTRO=$2

echo "${GITHUB_ACTOR} is attempting to install ROS2"
sudo apt install ros-${ROS_DISTRO}-desktop