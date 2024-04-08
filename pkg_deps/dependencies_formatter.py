# Imports
import os

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

if __name__ == "__main__":
    format_text()