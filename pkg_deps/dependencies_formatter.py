# Imports
import os
import argparse
from dataclasses import dataclass

# Global
parser = argparse.ArgumentParser(description="Script to format dependencies and output them into separate text-files.")
parser.add_argument("-a", "--append", help="Append to the dependencies", action='store_true')
parser.add_argument("-c", "--clean", help="clean text-files for parsing", action="store_true")
parser.add_argument("file", help="the file", type=str, default="packages.txt")

@dataclass
class Package:
    name: str
    type: str
    deps: list

def create_ros_pkg_text(package: Package) -> str:
    # Format the dependencies
    dependencies = ""
    for dep in package.deps:
        dependencies += f"{dep} "

    # Go through the dependencies
    if package.type == "ros.ament_cmake":
        ros_string = f"ros2 pkg create --build-type ament_cmake --license Apache-2.0 {package.name} --dependencies {package.deps}"

    return ros_string

def clean_file(file:str) -> None:
    input_file = f"{os.getcwd()}/{file}"
    
    # Read lines from the input file and append "text" to each line
    with open(input_file, 'r') as file:
        lines = file.readlines()

    # Modify the lines by removing the leading ../src/ to each line
    modified_lines = []
    for line in lines:
        if "../src/" in line:
            line = line.replace("../src/","")
            modified_lines.append(line)

    # Write the modified lines back to the same file
    with open(input_file, 'w') as file:
        file.writelines(modified_lines)

    print(f"Overwriten packages.txt to contain only package names")

def append_file(file:str):
    input_file = f"{os.getcwd()}/{file}"
    
    # Read lines from the input file and append "text" to each line
    with open(input_file, 'r') as file:
        packages = file.readlines()

    for package in packages:
        package = package.rstrip("\n")
        input_file = f"{os.getcwd()}/{package}_packages.txt"        
        
        with open(input_file, 'r') as package_file:
            package_file_lines = package_file.readlines()
        
        for line in package_file_lines:
            if "name:" in line:
                name = line.split(":")[1].strip()
            elif "type:" in line:
                type = line.split(":")[1].strip()
            elif "build:" in line:
                deps = line.split(":")[1].strip()

        # Create a new package
        new_package = Package(name,type,deps)
        ros_string = create_ros_pkg_text(new_package)

        with open(input_file, "a") as package_file:
            package_file.write(f"\nUSE THE COMMAND BELOW TO (RE)-CREATE THIS PACKAGE:")
            package_file.write(f"\n{ros_string}")
        

if __name__ == "__main__":
    arguments = parser.parse_args()
    if arguments.clean:
        clean_file(arguments.file)
    elif arguments.append:
        print(arguments.file)
        append_file(arguments.file)