#!/bin/sh
sudo "apt install ros-${ROS_DISTRO}-desktop"
source "/opt/ros/${ROS_DISTRO}/setup.bash && ros2 run --help"