# Imports
import os, sys, subprocess

# Global
input_file = f"{os.getcwd()}/packages.txt"

def format_text() -> None:
    # Read lines from the input file and append "text" to each line
    with open(input_file, 'r') as file:
        lines = file.readlines()

    # Modify the lines by removing the leading ../src/ to each line
    modified_lines = []
    for line in lines:
        if "../src/" in line:
            line = line.replace("../src/","")
            modified_lines.append(line)

    print(f"Succesfully removed leading path")

    # Write the modified lines back to the same file
    with open(input_file, 'w') as file:
        file.writelines(modified_lines)

    print(f"Overwriten packages.txt to contain only package names")

def run_linux_command(command):
    try:
        # Run the command and capture its output
        output = subprocess.check_output(command, shell=True, stderr=subprocess.STDOUT)
        # Decode the output from bytes to string
        output = output.decode('utf-8')
        return output
    except subprocess.CalledProcessError as e:
        # If the command returns a non-zero exit status, print the error
        print("Error:", e.output.decode('utf-8'))

def parse() -> None:
    # output = run_linux_command("source /opt/ros/iron/setup.bash")
    # print(output)
    output = run_linux_command("cd ..")
    print(output)
    output = run_linux_command("colcon build")
    print(output)

def main() -> None:
    # Get the command line arguments
    args = sys.argv[1:]

    # Handle
    for arg in args:
        if arg == "--format":
            format_text()
        elif arg == "--parse":
            parse()
        else:
            print("Undefined command")

if __name__ == "__main__":
    main()