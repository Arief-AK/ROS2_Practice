#!/bin/sh
echo "${GITHUB_ACTOR} is attempting to install ROS2 dependencies"
sudo apt-get update && sudo apt-get install python3-colcon-common-extensions
which colcon && which rosdep
sudo rosdep init && rosdep update
rosdep install -i --from-path src --rosdistro iron -y
echo "${GITHUB_ACTOR} has installed ROS2 dependencies"