#!/bin/sh

# Install figlet for fun!
sudo apt install figlet
echo "figlet version: $(which figlet)"

# Display hello message
figlet "_____________"
figlet "PACKAGE DEPENDENCIES"
figlet "_____________"

# Define the target directory and output file
directory="../src"
output_file="packages.txt"

# Create the output file
touch "$output_file"

# Check if the target is not a directory
if [ ! -d "$directory" ]; then
  echo "Target $directory is not a directory"
  exit 1
else
    > "$output_file"
fi

# Loop through files in the target directory
for file in "$directory"/*; do
    echo "$file" && echo "$file" >> packages.txt
done

# Run the python script for formatting the text file
python3 dependencies_formatter.py

# Check if the file exists
if [ -f "$output_file" ]; then
    # Read each word from the file and print it
    while IFS= read -r word; do
        cd ..
        colcon info $word > "pkg_deps/${word}_packages.txt"
        cd pkg_deps
    done < "$output_file"
else
    echo "File $output_file does not exist."
fi