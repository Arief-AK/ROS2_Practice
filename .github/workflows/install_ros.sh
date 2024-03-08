#!/bin/sh
sudo "apt install ros-${ROS_VERSION}-desktop"
source "/opt/ros/${ROS_VERSION}/setup.bash && ros2 run --help"