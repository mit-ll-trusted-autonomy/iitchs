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

from pathlib import Path

##########################################################
# File Names
##########################################################
ts_filename='./catl_planning/TransitionSystems/decomp_test.yaml'
output_path = './catl_planning/OutputFiles/decomp_test'
save_filename = f'{output_path}/decomp_test'

# Check if directories for save_filename and output_path exist.
# If not, create them.
Path(output_path).mkdir(parents=True, exist_ok=True)

#This reloads prior constraints or solutions.
load_old_files = False
reload_file = None #Would be a string file name

##########################################################
# Final Output Options
##########################################################
sim_dynamics = False #Should we run pure pursuit?
sim_wheel_base = 0.0 # describes turning radius of agents in simulator

k = 0.5  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 2.3  # speed proportional gain
dt = 0.8  # [s]
L = 20.0  # [m] wheel base of vehicle

kml_output = False
corner_coor = (41.98,-70.65) #Plymouth Rock

txt_out = False

##########################################################
# Specification and Agents
##########################################################

formula = '( F [0.0, 20.0]( ( ( T(3,A,{(S,1)}) U [5.0, 10.0] T(3,B,{(H1,1)}) ) || ( T(3,C,{(M,1)}) && T(3,D,{(S,1)}) ) ) ) && F [10.0, 30.0]( ( T(5,A,{(S,2)}) U [5.0, 10.0] ( T(3,B,{(W,1),(H1,1)}) && T(3,C,{(H2,1)}) ) ) ) && G [10.0, 50.0]( F [0.0, 20.0]( ( T(3,D,{(M,1)}) && T(3,A,{(S,2)}) ) ) ) && F [10.0, 30.0]( ( T(3,B,{(S,1)}) && ( T(3,C,{(D,1),(W,1)}) || T(3,D,{(D,2)}) ) ) ) )'

agents = [('q0', set(['H2', 'M'])), ('q0', set(['H2', 'H1'])), ('q0', set(['S'])), ('q0', set(['H2'])), ('q0', set(['H2'])), ('q0', set(['S', 'H1'])), ('q0', set(['S', 'D'])), ('q0', set(['H2', 'W'])), ('q0', set(['H2', 'D'])), ('q0', set(['D'])), ('q0', set(['S', 'M'])), ('q0', set(['H2', 'M'])), ('q0', set(['M'])), ('q0', set(['S', 'M'])), ('q0', set(['S', 'H1'])), ('q0', set(['M', 'W'])), ('q0', set(['H1', 'W'])), ('q0', set(['W'])), ('q0', set(['M', 'D'])), ('q0', set(['H2', 'S'])), ('q0', set(['S', 'D'])), ('q0', set(['D', 'W'])), ('q0', set(['H2', 'W'])), ('q0', set(['H1', 'D'])), ('q0', set(['H2', 'W']))]

# Show Transition World
show_visuals = True

robust = True
regularize = True
alpha = 0.5
upper_bound = True
num_agents = len(agents)

# Different numbers mean they operate at diffent levels and wont collide
cap_height_map = np.zeros(num_agents,'int')
agent_radius = 0.5*np.ones(num_agents,'int')


##########################################################
# Lower Level Trajectory Planning Variables
##########################################################

local_obstacles = [[
        [0,0,0],
        [0,0,0]
    ]]

max_attempts = 1
max_rrt_time = 2

plot_region_bounds = [0,50,0,50]
world_max = [50,50]

planning_step_time = 12

show_sol = False
record_sol = False

##########################################################
# Replanning Variables
##########################################################

replan_req = True
replan_time = 2
replan_region = 'q7' # TODO check if this works copied from case1.
                     #      Does it matter?
replan_grave = 'q25'
replan_agent_loss = 1
replan_cap = 2
grave_state = 25

