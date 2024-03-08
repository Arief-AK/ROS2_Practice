#!/bin/sh

GITHUB_ACTOR=$1
ROS_DISTRO=$2

echo "${GITHUB_ACTOR} is attempting to install ROS2"
sudo apt install ros-${ROS_DISTRO}-desktop
source ./opt/ros/${ROS_DISTRO}/setup.bash && ros2 run --help
echo "$GITHUB_ACTOR successfully installed ROS2-$ROS_DISTRO in system"