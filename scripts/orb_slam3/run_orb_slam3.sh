# Set up logging
log_file="$(rospack find rudpt_evaluation)/logs/run_orb_slam3.log"

package_directory="$(rospack find rudpt_evaluation)"
workspace_directory="$(catkin locate --src)/.."

algorithm_name="orb_slam3"
input_directory="${1:-"$workspace_directory/test/rudpt/1,2"}"
output_directory="$package_directory/eval_data/$algorithm_name"
script -c "bash $package_directory/scripts/run.sh \"$input_directory\" \"$output_directory\" \"$algorithm_name\"" "$log_file"

# Post-process the log file to remove unwanted characters
col -b < "$log_file" > "${log_file}.tmp"
mv "${log_file}.tmp" "$log_file"