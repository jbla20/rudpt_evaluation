#!/usr/bin/bash

# Get the directory for the evaluation
evaluation_directory="${1:-"/home/v-slam/vslam_ws/src/rudpt_evaluation/rudpt_orb_slam3/results/1,1_0_0"}"

# Validate if directory exists
if [ ! -d "$evaluation_directory" ]; then
    echo "Error: Directory '$evaluation_directory' does not exist."
    exit 1
fi

# Save the current PYTHONPATH and change it to prioritize the ROS package directory
OLD_PYTHONPATH="$PYTHONPATH"
export PYTHONPATH="/home/v-slam/svo_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages"

# Run the evaluation script
echo "Running evaluation script for directory: $evaluation_directory"
rosrun rpg_trajectory_evaluation analyze_trajectory_single.py "$evaluation_directory"

# Restore the PYTHONPATH
export PYTHONPATH="$OLD_PYTHONPATH"