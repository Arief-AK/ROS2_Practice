# Managing Dependencies
To provide better transparency with package dependencies, the `get_package_dependencies.sh` and `dependencies_formatter.py` scripts are created to showcase all dependencies used within each package in the repository.

## Quickstart
We are going to run the script, but before that, make sure that we give the executable permission to the file.
```shell
sudo chmod +x get_package_dependencies.sh
```
Now we can run it!
```shell
./get_package_dependencies.sh
```
This should produce multiple text-files that showcase the different depdencies for each package.

## Background
The dependencies are formatted into a text-file, that describes all the required dependencies, build/test configurations, and metadata of the package. This is actually just the output of the `colcon info` command. This is useful when creating new packages that function similarly to the current packages found in the repository.

Take the following example for the `tutorial_interfaces` package.

```shell
path: src/tutorial_interfaces
  type: ros.ament_cmake
  name: tutorial_interfaces
  dependencies:
    build: ament_cmake geometry_msgs rosidl_default_generators
    run: geometry_msgs rosidl_default_runtime
    test: ament_lint_auto ament_lint_common
  metadata:
    maintainers: ['arief <mariefak22@gmail.com>']
    version: 0.0.0
```

The idea is that it gives the developer better reference to the different dependencies that needs to be considered.