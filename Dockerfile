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
COPY . /home/user/ros2_ws

# Source ROS2 and build packages	
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build

# Source ROS2 and test packages
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon test --event-handler=console_direct+