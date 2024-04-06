# Set up logging
log_file="$(rospack find rudpt_svo)/logs/run_svo.log"
script -c 'bash run_svo.sh' "$log_file"

# Post-process the log file to remove unwanted characters
col -b < "$log_file" > "${log_file}.tmp"
mv "${log_file}.tmp" "$log_file"