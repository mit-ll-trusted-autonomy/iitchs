'''
General purpose utility functions.

Author: James Usevitch (james.usevitch@ll.mit.edu)
        MIT LL, 2021
'''

# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

# This material is based upon work supported by the Under Secretary of Defense for 
# Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any 
# opinions, findings, conclusions or recommendations expressed in this material 
# are those of the author(s) and do not necessarily reflect the views of the Under 
# Secretary of Defense for Research and Engineering.

# (C) 2021 Massachusetts Institute of Technology.

# Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

# The software/firmware is provided to you on an As-Is basis

# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS 
# Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. 
# Government rights in this work are defined by DFARS 252.227-7013 or DFARS 
# 252.227-7014 as detailed above. Use of this work other than as specifically 
# authorized by the U.S. Government may violate any copyrights that exist in this 
# work.


import dill as pickle
from pathlib import Path
import shutil
import os


# # # Preprocessing Utilities

def get_labels_and_counts_dict(ts):
    '''
    Given an input transition system, returns a dictionary mapping label names 
    to number of states each label occurs in.

    Args:
        ts:     Lomap transition system.

    Returns:
        (dict):     Dictionary mapping all unique label names to the
                    number of states they each occur in.
    '''
    labels_dict = dict()

    for label_list in ts.g.node.values():
        for label in label_list['prop']:
            if label not in labels_dict.keys():
                labels_dict[label] = 1
            else:
                labels_dict[label] += 1

    return labels_dict




# # # Postprocessing Utilities

def sort_output_files(directory):
    '''
    Sorts and moves output files into three directories: Feasible, Infeasible, and Other.

    Requires the input directory to contain the files ``DataFrame_Feasible.pkl``, 
    ``DataFrame_Infeasible.pkl``, and ``DataFrame_Other.pkl``.
    '''
    with open(f"{directory}/DataFrame_Feasible.pkl","rb") as file:
        data_feasible = pickle.load(file)

    with open(f"{directory}/DataFrame_Infeasible.pkl",'rb') as file:
        data_infeasible = pickle.load(file)

    with open(f"{directory}/DataFrame_Other.pkl",'rb') as file:
        data_other = pickle.load(file)

    # # Move all .mps / .lp / .sol files into feasible / infeasible / other folders

    feasible_path = f"{directory}/feasible"
    infeasible_path = f"{directory}/infeasible"
    other_path = f"{directory}/other"

    Path(feasible_path).mkdir(exist_ok=True)
    Path(infeasible_path).mkdir(exist_ok=True)
    Path(other_path).mkdir(exist_ok=True)

    # Feasible
    for ts_id in data_feasible['Trial_ID']:
        lp = f"{directory}/TS{ts_id}.lp.gz"
        mps = f"{directory}/TS{ts_id}.mps.gz"
        sol = f"{directory}/TS{ts_id}.sol.gz"

        lp_dest = f"{directory}/feasible/TS{ts_id}.lp.gz"
        mps_dest = f"{directory}/feasible/TS{ts_id}.mps.gz"
        sol_dest = f"{directory}/feasible/TS{ts_id}.sol.gz"

        if os.path.isfile(lp):
            shutil.move(lp, lp_dest)
        if os.path.isfile(mps):
            shutil.move(mps, mps_dest)
        if os.path.isfile(sol):
            shutil.move(sol, sol_dest)

    # Infeasible
    for ts_id in data_infeasible['Trial_ID']:
        lp = f"{directory}/TS{ts_id}.lp.gz"
        mps = f"{directory}/TS{ts_id}.mps.gz"
        sol = f"{directory}/TS{ts_id}.sol.gz"

        lp_dest = f"{directory}/infeasible/TS{ts_id}.lp.gz"
        mps_dest = f"{directory}/infeasible/TS{ts_id}.mps.gz"
        sol_dest = f"{directory}/infeasible/TS{ts_id}.sol.gz"

        if os.path.isfile(lp):
            shutil.move(lp, lp_dest)
        if os.path.isfile(mps):
            shutil.move(mps, mps_dest)
        if os.path.isfile(sol):
            shutil.move(sol, sol_dest)
    
    # Other
    for ts_id in data_other['Trial_ID']:
        lp = f"{directory}/TS{ts_id}.lp.gz"
        mps = f"{directory}/TS{ts_id}.mps.gz"
        sol = f"{directory}/TS{ts_id}.sol.gz"

        lp_dest = f"{directory}/other/TS{ts_id}.lp.gz"
        mps_dest = f"{directory}/other/TS{ts_id}.mps.gz"
        sol_dest = f"{directory}/other/TS{ts_id}.sol.gz"

        if os.path.isfile(lp):
            shutil.move(lp, lp_dest)
        if os.path.isfile(mps):
            shutil.move(mps, mps_dest)
        if os.path.isfile(sol):
            shutil.move(sol, sol_dest)