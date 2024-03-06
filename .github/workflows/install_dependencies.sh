#!/bin/sh
locale  # check for UTF-8
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
echo "${GITHUB_ACTOR} has set up ubuntu with updated locale config"
sudo apt install software-properties-common
sudo add-apt-repository universe
echo "${GITHUB_ACTOR} has updated ubuntu repositories"
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo "${GITHUB_ACTOR} has pulled ROS2 repositories"
sudo apt update && sudo apt install ros-dev-tools
echo "${GITHUB_ACTOR} has installed ROS tools"
sudo apt update
sudo apt upgrade
echo "${GITHUB_ACTOR} has updated and upgraded Ubuntu"