![ROS2 setup with tooling](https://github.com/Arief-AK/ROS2_Practice/actions/workflows/setup_ros_with_tooling.yml/badge.svg) ![ROS2 setup with custom scripts](https://github.com/Arief-AK/ROS2_Practice/actions/workflows/setup_ros_bare_metal.yml/badge.svg) ![Publish container to GHCR](https://github.com/Arief-AK/ROS2_Practice/actions/workflows/publish_docker_package.yml/badge.svg) ![Publish to Docker Hub](https://github.com/Arief-AK/ROS2_Practice/actions/workflows/publish_docker_image.yml/badge.svg)

# ROS2 Practice
The purpose of this repository is to familiarise and document the practice of using ROS2 with CI and CD best practices applied.

## Quickstart
Clone the git repository into a new workspace by using `git clone` and build the packages with the following command.
```bash
colcon build
```
To ensure that the functionality of the packages are proper, run the following command.
```bash
colcon test
```
If you want to test a specific package, run the following command.
```bash
colcon test --packages-select <package_name> --event-handler=console_direct+
```
This should run the GTest testing suite on the packages.

## Finding Dependencies
To find the respective dependencies that packages utilise, head to [pkg_deps](pkg_deps) folder and run the `get_package_dependencies.sh` script
```bash
cd pkg_deps
./get_package_dependencies.sh
```
This script produces a text-file that showcases the dependencies of a package. You can find more information [here](pkg_deps/Managing%20Dependencies.md).