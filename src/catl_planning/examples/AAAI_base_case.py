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

# SPDX-License-Identifier: BSD-3-Clause

import numpy as np
import os
from lomap import Ts

# For creating Output File Directories, if they don't exist.
from pathlib import Path

##########################################################
# File Names
##########################################################
ts_filename = './catl_planning/TransitionSystems/demo_grave.yaml'

# Paths to save output to.
save_filename = './catl_planning/OutputFiles/AAAI_base_case/AAAI_base_case'
output_path = './catl_planning/OutputFiles/AAAI_base_case'

# Check if directories for save_filename and output_path exist.
# If not, create them.
#Path(save_filename).mkdir(parents=True, exist_ok=True)
Path(output_path).mkdir(parents=True, exist_ok=True)

#This reloads prior constraints or solutions.
load_old_files = False
reload_file = None #Would be a string file name

##########################################################
# Final Output Options
##########################################################
sim_dynamics = False #Should we run pure pursuit?
sim_wheel_base = 0.0 #describes turning radius of agents in simulator

k = 0.5  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 2.3  # speed proportional gain
dt = 0.8  # [s]
L = 20.0  # [m] wheel base of vehicle

#old_nearest_point_index = None
show_animation = False


kml_output = False
corner_coor = (41.98,-70.65) #Plymouth Rock
# Boston Harbor: (42.36,-70.95)

txt_out = False


##########################################################
# Specification
##########################################################


formula = 'F[0,10](T(3,A,{(C1,2)}) || T(3,B,{(C1,2),(C2,2)})) && F[20,40](T(3,C,{(C2,1)}) U[5,10] (T(3, A, {(C1, 1),(C2,1)}) && T(3,B,{(C1,1),(C2,1)})))'
agents = [('q0',{'C1','C2'}),('q0',{'C1'}),('q0',{'C1'}),('q0',{'C1','C2'}),('q0',{'C1','C2'}),('q0',{'C2'}),('q0',{'C1','C2'}),('q0',{'C1','C2'}),('q0',{'C1'}),('q0',{'C2'})]

show_visuals = True # originally False

robust = True
regularize=True
alpha = 0.5
upper_bound = True
num_agents = len(agents)

#Different numbers mean they operate at diffent levels and wont collide
cap_height_map = np.zeros(num_agents,'int')
agent_radius = 0.5*np.ones(num_agents,'int')

##########################################################
# Lower Level Trajectory Planning Variables
##########################################################

local_obstacles = [[
        [0,0,0],
        [0,0,0]
    ]]

'''
These are obstalces not in the optimization but need to be considered in trajectories.
They are circles (x,y,r) and then each outter array is for a given height.
obstaclesList  = [[
        [0,0,0],
        [0,0,0]
    ],
    [
        [0,0,0],
        [0,0,0]
    ],
    [
        [0,0,0],
        [0,0,0]
    ]
    ]
'''
#Number of restarts allowed for one trajectory before restarting from scratch
max_attempts = 1
#Time allowed for single tree to be built before restart
max_rrt_time = 2

#These are reference frames for the rrt algorithm - change them to fit the world you want to plan in:
plot_region_bounds = [0,100,0,50]
world_max = [100,50]

#Number of steps in between each optimization step time (this is used to convert MILP edge weights into real valued distance.
planning_step_time = 12

#Show generates a plot of all of the trajectories - It is not automatically saved
show_sol = True

#Record generates a video of the trajectories being executed in time - it is saved automatically
record_sol = False


##########################################################
# Replanning Variables
##########################################################


replan_req = True
replan_time = 2
replan_region = 'q7'
replan_grave = 'q9'
replan_agent_loss = 1
replan_cap = 2
grave_state = 9
