#!/usr/bin/bash

# Function to close screen sessions
cleanup() {
    # Send SIGINT to the screen sessions to close them gracefully
    screen -S rosbag_session -X stuff "^C"
    screen -S roslaunch_session -X stuff "^C"
    screen -S traj_evaluation -X stuff "^C"
    screen -wipe
}

# Set up signal handler to call the cleanup function on SIGINT
trap cleanup SIGINT

# Function to check if a process is running
is_process_running() {
    local pid="$1"
    if ps -p "$pid" > /dev/null; then
        # Unix-like systems and shell scripting, particularly with processes and exit codes, 
        # the convention is to return 0 for success and a non-zero value for failure.
        return 0  # Process is running
    else
        return 1  # Process is not running
    fi
}

# Get the directory containing files
package_directory="$(rospack find rudpt_evaluation)"
workspace_directory="$(catkin locate --src)/.."
input_directory="${1:-"$workspace_directory/test/rudpt"}"
output_directory="${2:-"$package_directory/eval_data"}"
algorithm_name="${3:-"svo"}"
echo "Output directory: $output_directory"
if [ -n "$4" ]; then 
    save_directory="$4"-$(date +"%Y-%m-%d_%H-%M-%S")
    echo "Save directory: $save_directory"
fi

# Validate if directory exists
if [ ! -d "$input_directory" ] || [ ! -d "$output_directory" ]; then
    echo "Error: Directory '$input_directory' or '$output_directory' does not exist."
    exit 1
fi

# Loop through each file in the directory
for bag_file in "$input_directory"/*; do
    # Check if it's a file
    if [ ! -f "$bag_file" ] || [[ ! "$bag_file" =~ \.bag$ ]]; then
        continue
    fi

    # Start processing the file
    echo "----------------------------------------"
    echo "Processing file: $bag_file"

    # Run ROS launch file in a separate terminal and capture PID
    echo "Launching $algorithm_name environment in Rviz"
    output_file="$output_directory/$(basename "$bag_file" .bag)/stamped_traj_estimate.txt"
    echo "Output file: $output_file"

    # Create the parent directory if it doesn't exist
    if [ ! -d "$(dirname "$output_file")" ]; then
        echo "Creating directory $(dirname "$output_file")"
        mkdir -p "$(dirname "$output_file")"
    fi
    
    # Launch the ROS launch file
    launch_file="${algorithm_name}_euroc_save_pose.launch"
    screen -d -m -S roslaunch_session bash -c "roslaunch rudpt_evaluation \"$launch_file\" output_file:=\"$output_file\""
    roslaunch_pid=$(screen -ls | grep roslaunch_session | awk '{print $1}' | cut -d. -f1)

    # Wait for some time to ensure the first command has started (adjust as needed)
    sleep 10
    
    # Run ROS bag play in a separate terminal and capture PID
    echo "Playing ros bag file"
    eval_duration=100.0
    screen -d -m -S rosbag_session bash -c "rosbag play \"$bag_file\" -u \"$eval_duration\"" # Can add "-s START_TIME" or "-u DURATION" for debug
    rosbag_pid=$(screen -ls | grep rosbag_session | awk '{print $1}' | cut -d. -f1)
    # echo "PID of rosbag_pid is $rosbag_pid"
    rosbag_duration=$(rosbag info --yaml --key=duration $bag_file)
    rosbag_start=$(rosbag info --yaml --key=start $bag_file)
    
    # Check if the rosbag play process has finished
    time=0
    while is_process_running $rosbag_pid; do        
        if ! is_process_running $roslaunch_pid; then
            echo "Error: roslaunch process has stopped unexpectedly."
            exit 1
        fi
        
        sleep 1
        time=$((time + 1))
        
        # Get number of lines in the log file
        echo -ne "Ros bag playing: "$time"/"$rosbag_duration" seconds.\r"
    done

    # Kill the process running in the screen session by sending a SIGTERM signal
    echo -e "\nFinished processing file. Killing terminals.\n"
    screen -S roslaunch_session -X stuff "^C"

    # Sleep to allow time for terminals to close before proceeding to the next iteration
    sleep 2
    
    # Set the start and end times based on the rosbag duration
    python3 -c "import sys; sys.path.append('$package_directory/scripts/helpers');\
                import set_eval_times; set_eval_times.set_eval_times(\
                    eval_folder='$(dirname "$output_file")', traj_duration=$rosbag_duration, start_time=$rosbag_start, duration=$eval_duration\
                );"

    # Compare the trajectory with the groundtruth using rpg_trajectory_evaluation
    echo "Evaluating the trajectory"
    if [ -n "$save_directory" ]; then
        "$package_directory/scripts/compare_results.sh" "$(dirname "$output_file")" "$save_directory"
    else
        "$package_directory/scripts/compare_results.sh" "$(dirname "$output_file")"
    fi

done

# Compare boxplots between the different tests
if [ -n "$save_directory" ]; then
    rosrun rpg_trajectory_evaluation boxplot_comparison.py "$save_directory"
    rosrun rpg_trajectory_evaluation boxplot_comparison_individual.py "$save_directory" "--alg_type=$algorithm_name"
fi
