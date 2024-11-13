# Standard library imports


# Type imports
from typing import Tuple

# Constants
TEST_MAP = {'1,1' : '1rect',
            '1,2' : '1circ',
            '1,3' : '1rand',
            '2,1' : '2feat',
            '2,2' : '2tilt'}
CONDITION_MAP = {'t' :
                    {'0': '0ml', '1': '50ml', '2': '100ml'},
                'ms' :
                    {'0': '0.0g', '1': '1.5g', '2': '3.0g', '3': '4.5g'}}

def num_to_name(traj_num_identifier : str) -> str:
    # Determine trajectory type
    if not traj_num_identifier[:3] in ['1,1', '1,2', '1,3', '2,1', '2,2']:
        raise ValueError("Invalid trajectory type: " + traj_num_identifier[:3])
    traj_type = TEST_MAP[traj_num_identifier[:3]]

    # Determine turbidity
    if not traj_num_identifier[4] in ['0', '1', '2']:
        raise ValueError("Invalid turbidity level: " + traj_num_identifier[4])
    turbidity = f"t={CONDITION_MAP['t'][traj_num_identifier[4]]}"

    # Determine marine snow
    if not traj_num_identifier[6] in ['0', '1', '2', '3']:
        raise ValueError("Invalid marine snow level: " + traj_num_identifier[6])
    marine_snow = f"ms={CONDITION_MAP['ms'][traj_num_identifier[6]]}"

    return f"{traj_type}_{turbidity}_{marine_snow}"

def num_to_levels(traj_num_identifier : str) -> Tuple[str, str]:
    # Determine trajectory type
    if not traj_num_identifier[:3] in ['1,1', '1,2', '1,3', '2,1', '2,2']:
        raise ValueError("Invalid trajectory type: " + traj_num_identifier[:3])

    # Determine turbidity
    if not traj_num_identifier[4] in ['0', '1', '2']:
        raise ValueError("Invalid turbidity level: " + traj_num_identifier[4])
    turbidity = int(traj_num_identifier[4])

    # Determine marine snow
    if not traj_num_identifier[6] in ['0', '1', '2', '3']:
        raise ValueError("Invalid marine snow level: " + traj_num_identifier[6])
    marine_snow = int(traj_num_identifier[6])

    return (turbidity, marine_snow)