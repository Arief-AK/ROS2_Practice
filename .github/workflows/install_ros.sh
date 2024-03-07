#!/bin/sh
echo "${GITHUB_ACTOR} is attempting to install ROS2"
sudo apt install ros-iron-desktop
source /opt/ros/iron/setup.bash && ros2 run --help
echo "$GITHUB_ACTOR successfully installed ROS2-iron in system"