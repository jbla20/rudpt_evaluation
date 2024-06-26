# /usr/bin/env python3

import yaml
from pathlib import Path
from typing import Optional

def set_eval_times(eval_folder : str, 
                   traj_duration : float, 
                   start_time : float = 0, 
                   duration : Optional[float] = None):
        
    eval_file = Path(eval_folder + "/start_end_time.yaml")
    if not eval_file.is_file():
        print("Incorrect path to the result folder")
        return

    print("Setting evaluation times for the result folder: ", eval_folder)
    eval_times_dict = {}
    eval_times_dict["start_time_sec"] = start_time
    if duration and duration < traj_duration:
        eval_times_dict["end_time_sec"] = start_time + duration
    
    with open(eval_file, "w") as file:
        yaml.dump(eval_times_dict, file, sort_keys=False)
    

if __name__ == "__main__":
    set_eval_times("/home/v-slam/vins_ws/src/rudpt_evaluation/eval_data/vins/1,1_0_0",
                   109.0,
                   0,
                   None)