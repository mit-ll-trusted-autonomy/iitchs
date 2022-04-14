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
from lomap import Ts


# For creating Output File Directories, if they don't exist.
from pathlib import Path

##########################################################
# File Names
##########################################################
ts_filename='./catl_planning/TransitionSystems/two_region_world.yaml'
save_filename = './catl_planning/OutputFiles/cs_two_region/'
output_path = './catl_planning/OutputFiles/cs_two_region/'

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

old_nearest_point_index = None
show_animation = False


kml_output = False
corner_coor = (41.98,-70.65) #Plymouth Rock
# Boston Harbor: (42.36,-70.95)

txt_out = False


##########################################################
# Specification and Agents
##########################################################

grave_state = 3
replan_grave = 'q3'

#Key Objectives:

# Agent 1 with C1 starts in blue region q1
# Agent 2 with C2 starts in white region q2
# Goal is for both agents to swap regions and then remain there
# Breakdown:
#   1. F[0,10] T(2,blue,{(C2,1)})           :   Makes agent 1 go to blue region
#   2. F[0,10] T(2,white,{(C1,1)})          :   Makes agent 2 go to white region
#   3. F[0,10] G[0,10] T(10,blue,{(C2,1)})  :   Makes agent 1 remain in blue region once it arrives
#   4. F[0,10] G[0,10] T(10,white,{(C1,1)}) :   Makes agent 2 remain in white region once it arrives
formula = 'F[0,10] T(2,blue,{(C2,1)}) && F[0,10] T(2,white,{(C1,1)}) && F[0,10] G[0,10] T(10,blue,{(C2,1)}) && F[0,10] G[0,10] T(10,white,{(C1,1)})'

agents = [('q1', {'C1'}), ('q2', {'C2'})]


##########################################################
#ROS Setup
##########################################################

ros_name_index_mapping = ['robot1','robot2']
sim_2_world_converts = [-0.4, -0.1, -0.059, 0.058] #[x_off,y_off,scaley,scalex]


#Show Transition World
show_visuals = False

#Show generates a plot of all of the trajectories - It is not automatically saved
show_sol = True

#Record generates a video of the trajectories being executed in time - it is saved automatically
record_sol = False

robust = True
regularize=True
alpha = 0.5
upper_bound = True
num_agents = 2

#Different numbers mean they operate at diffent levels and wont collide
cap_height_map = [0,0,0,0]
agent_radius = .5*np.ones(num_agents,'int')
##CODE CURRENTLY CONSIDERS ALL AGENTS AT HEIGHT MAP 0 - NEEDED TO PARALLELIZE EASILY

##########################################################
# Lower Level Trajectory Planning Variables
##########################################################

local_obstacles = [[
        [0,0,0],
        [0,0,0]
    ],
    [
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
plot_region_bounds = [0,30,0,10]
world_max = [30,10]

#Number of steps in between each optimization step time (this is used to convert MILP edge weights into real valued distance.
planning_step_time = 6


##########################################################
# Replanning Variables
##########################################################


replan_req = False
replan_time = None
replan_region = None
replan_num = None
replan_cap = None

##########################################################
# Decomposition Variables
##########################################################
lambda_wt = 0.0
v_ind_wt = 0.0

