#!/usr/bin/bash

# Get the directory for the evaluation
package_directory="$(catkin locate --this)"
evaluation_directory="${1:-"$package_directory/results/test"}"

# Validate if directory exists
if [ ! -d "$evaluation_directory" ]; then
    echo "Error: Directory '$evaluation_directory' does not exist."
    exit 1
fi

# Save the current PYTHONPATH and change it to prioritize the ROS package directory
OLD_PYTHONPATH="$PYTHONPATH"
devel_space=$(catkin locate --devel)
export PYTHONPATH="$devel_space/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages"

# Remove existing evaluations in the folder if they exist
if [ -d "$evaluation_directory/plots" ]; then rm -rf "$evaluation_directory/plots"; fi
if [ -d "$evaluation_directory/saved_results" ]; then rm -rf "$evaluation_directory/saved_results"; fi

# Run the evaluation script
echo "Running evaluation script for directory: $evaluation_directory"
rosrun rpg_trajectory_evaluation analyze_trajectory_single.py "$evaluation_directory"

# Restore the PYTHONPATH
export PYTHONPATH="$OLD_PYTHONPATH"