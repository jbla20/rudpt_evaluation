#!/usr/bin/bash

# Get the directory for the evaluation
package_directory="$(catkin locate --this)"
evaluation_directory="${1:-"$package_directory/eval_data/test"}"
if [ -n "$2" ] && [ -d "$(dirname "$2")" ]; then
    save_directory="$2"
    if [ ! -d "$save_directory" ]; then
        mkdir -p "$save_directory"
    fi
    echo "Save directory: $save_directory"
fi

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

# Save the results if a save directory is provided
if [ -n "$save_directory" ]; then
    echo "Saving results to directory: $save_directory"
    cp -r "$evaluation_directory" "$save_directory"
    # Rename the directory to include the date and time
    mv "$save_directory/$(basename "$evaluation_directory")"\
       "$save_directory/$(basename "$evaluation_directory")_$(date +"%Y-%m-%d_%H-%M-%S")"
fi