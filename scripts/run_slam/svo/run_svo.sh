# Set up logging
log_file="$(rospack find rudpt_evaluation)/logs/run_svo.log"

package_directory="$(rospack find rudpt_evaluation)"
workspace_directory="$(catkin locate --src)/.."

algorithm_name="svo"
input_directory="${1:-"$workspace_directory/test/rudpt/1,2"}"
output_directory="$package_directory/eval_data/$algorithm_name"
save_directory="${2:-"/storage/data/results/svo/1,2"}"
comment="${3:-""}"
script -c "bash $package_directory/scripts/run.sh \
                \"$input_directory\" \
                \"$output_directory\" \
                \"$algorithm_name\" \
                \"$save_directory\" \
                \"$comment\"" "$log_file"

# Post-process the log file to remove unwanted characters
col -b < "$log_file" > "${log_file}.tmp"
mv "${log_file}.tmp" "$log_file"