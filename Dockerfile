# Pull the ros-base image
FROM osrf/ros:iron-desktop

# Update the linux environment
RUN sudo apt update -y \
    && sudo apt upgrade -y

# Create a workspace
RUN cd home \
    && mkdir -p user/ros2_ws \
    && cd user/ros2_ws

# Set working directory as this directory
WORKDIR /home/user/ros2_ws

# Copy source contents of this project to src directory
ADD src /home/user/ros2_ws/src

	
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build

# Source the ROS2
RUN . /home/user/ros2_ws/install/setup.sh

# Test packages
RUN colcon test --event-handler=console_direct+