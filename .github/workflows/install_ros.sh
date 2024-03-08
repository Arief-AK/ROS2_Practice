#!/bin/sh
sudo apt install ros-iron-desktop
source "/opt/ros/${ROS_DISTRO}/setup.bash && ros2 run --help"