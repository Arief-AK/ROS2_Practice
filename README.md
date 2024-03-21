# ROS2 Practice
The purpose of this repository is to familiarise and document the practice of using ROS2. This repostiory follows the [ROS2 Iron Tutorials](https://docs.ros.org/en/iron/Tutorials.html) starting from the [Beginner: Client Libraries](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries.html) section.

### Pre-requisites
This repository is intended to be used with the ROS2-Iron distribution. Build files are ignored, please review the [ignore file](.gitignore).

### Installing
Clone the git repository into a new workspace by using `git clone` and build the packages with the following command.
```
colcon build
```

To ensure that the functionality of the packages are proper, run the following command.
```
colcon test
```
If you want to test a specific package, run the following command.
```
colcon test --packages-select <package_name> --event-handler=console_direct+
```
This should run the GTest testing suite on the packages.