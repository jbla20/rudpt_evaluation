# /usr/bin/env python3

from pathlib import Path
import pandas as pd

def fix_timestamp(eval_folder : str):
    groundtruth_file = Path(eval_folder + "/stamped_groundtruth.txt")
    estimate_file = Path(eval_folder + "/stamped_traj_estimate.txt")
    if not groundtruth_file.is_file or not estimate_file.is_file:
        print("Incorrect structure of the result folder")
        return
    
    print("Fixing timestamps for the result folder: ", eval_folder)
    initial_timestamp = None
    for file in [groundtruth_file, estimate_file]:        
        # Read the file into a DataFrame
        df = pd.read_csv(file, sep=" ", header=None, skiprows=1)

        # Rename the columns
        df.columns = ["timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]

        # Modify the timestamp column as needed
        initial_timestamp = df["timestamp"][0] if initial_timestamp is None else initial_timestamp
        df["timestamp"] = df["timestamp"] - initial_timestamp
        
        # Write the modified DataFrame back to a text file
        with open(str(file), "w") as f:
            f.write("# ")
            df.to_csv(f, sep=" ", index=False, header=True)

if __name__ == "__main__":
    fix_timestamp("/home/v-slam/orbslam_ws/src/rudpt_evaluation/eval_data/orb_slam3/MH_01_easy")