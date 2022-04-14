# -*- coding: utf-8 -*-

#--------------------------------------------------------------------------------

#LL Copyright
#This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.
#© 2019 Massachusetts Institute of Technology.
#The software firmware is provided to you on an As-Is basis
#Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
#LL Copyright

#--------------------------------------------------------------------------------

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

#--------------------------------------------------------------------------------

import numpy as np
from numpy.random import randint
import pandas as pd
from lomap import Ts
import networkx as nx
import pickle
import sys
from catl_planning.reduced_ts import ReducedTs
from catl_planning.route_planning import compute_agent_classes, compute_capability_bitmap
from catl_planning.rrt_planning import RRT_Get_Path
from catl_planning.utils import state2int, int2state


if sys.flags.debug:
    import pdb

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


#FOR VIDEO - Uncomment two lines below
import matplotlib
matplotlib.use("TkAgg")
# from matplotlib.animation import FFMpegWriter
import matplotlib.pyplot as plt

from catl_planning.visualization import show_environment_agents
from catl_planning.visualization import show_transition_agents
from catl_planning.visualization import show_world
import time as timer

###############################################################################
#Parse Data from sol file
###############################################################################
def read_sol_data(data,ts):
    #This is the data file to parse - designed for gurobi.sol files
    #data = pd.read_csv("isrr2019.sol",sep='\n')
    # Format of the `states` variable:
    #   column 1:   State number
    #   column 2:   Timestep
    #   column 3:   Capability number
    #   column 4:   Number of agents in the state with that capability at that timestep
    #
    # **The states matrix only records information for states and timesteps with nonzero column 4 entries.**
    # It essentially tracks all timesteps and states that have nonzero capabilities in that state.  
    
    var_data = data.to_numpy()
    states = []
    edges = []

    # obstacleList[0] - Ground obstacles
    # obstacleList[1] - Mid-air obstacles
    # obstacleList[2] - top-Air obstacles

    #temp_edge_weights = nx.get_edge_attributes(ts.g,'weight')
    
    #record last time 
    end_time = 0
    
    #record_max_weight
    max_weight = 0
    
    #This loop extracts the states and edges from the solution and puts them into a numpy array
    for i in range(1,len(var_data)):

        if var_data[i,0][0] == 'z' and var_data[i,0][2] == 'q':
            #States
            if var_data[i,0].count('q')==1:
                vars = var_data[i,0].split("_")
                time_state = vars[3].split(" ")
                time = time_state[0]
                # Change any 'n' characters to a negative sign, due to PuLP naming conventions
                if time[0] == 'n':
                    time = "-"  + time[1:]

                num = time_state[1]
                state = vars[1].replace("q","")
                cap = vars[2]
                # DEBUG
                try:
                    int(float(time))
                except:
                    print("CHECK time VARIABLE")
                # \DEBUG

                if int(float(time)) > end_time:
                    end_time = int(float(time))
                if int(float(num)) > 0:
                    states.append([int(float(state)),int(float(time)),int(float(cap)),int(float(num))])
            #Edges
            if var_data[i,0].count('q')==2:
                vars = var_data[i,0].split("_")
                time_state = vars[4].split(" ")
                time = time_state[0]
                # Change any 'n' characters to a negative sign, due to PuLP naming conventions
                if time[0] == 'n':
                    time = "-"  + time[1:]

                num = time_state[1]
                state1 = vars[1].replace("q","")
                state2 = vars[2].replace("q","")
                cap = vars[3]
                if not 'grave' in ts.g.node['q'+str(state1)]['prop'] and not 'grave' in ts.g.node['q'+str(state2)]['prop']:
                    weight = ts.g.edge['q'+str(state1)]['q'+str(state2)]['weight']
                    if weight > max_weight:
                        max_weight = weight
                else:
                    weight = 1
                if int(float(time)) > end_time:
                    end_time = int(float(time))
                #weight = temp_edge_weights[('q'+str(state1),'q'+str(state2))]
                if int(float(num)) > 0:
                    edges.append([int(float(state1)),int(float(state2)),int(float(time)),weight,int(float(cap)),int(float(num))])
                
    # populate an array with p_vars
    # entries are label, predicate, time, state, count
    p_vars = [x[0] for x in var_data if x[0][0:2]=='p_']
    # preds = []
    # map(lambda x: preds.append([x.split('_')[1],x.split('_')[2],x.split('_')[3],x.split('_')[4].split(' ')[0],x.split('_')[4].split(' ')[1]]), p_vars)
    preds =  list(map(lambda x:[x.split('_')[1],x.split('_')[2],x.split('_')[3],x.split('_')[4].split(' ')[0],x.split('_')[4].split(' ')[1]], p_vars))
    np.asarray(preds)
    
    #Make them numpy arrays
    states = np.asarray(states)
    #print(states)
    edges = np.asarray(edges)
    print('time_params',end_time,max_weight)
    return states,edges,(end_time+max_weight),preds
    
    
###############################################################################
#Check if point is in past trajectory obstacle
###############################################################################    
def collision_check(point,current_time,radius,past_paths=None):
    in_obs = False
    if past_paths != None:
        past_obs = []
        for i in range(0,len(past_paths)):
            #print(past_paths)
            past_obs.append((past_paths[i][(current_time)]+[radius*2]))
        for j in range(len(past_obs)):
            d = np.power((np.power(past_obs[j][0]-point[0],2)+np.power(past_obs[j][1]-point[1],2)),0.5)
            if d < 2*radius:
                in_obs = True
    return in_obs


###############################################################################
#Find bounds for planning between regions
###############################################################################
def split_bounds_for_rrt(region1,region2,bounds,overlaps):
    new_bounds = []
    for region,set in bounds:
        if region != region1 and region!= region2 and region not in overlaps[region1] and region not in overlaps[region2]:
            new_bounds.append((set))
    return(new_bounds)

###############################################################################
#Find bounds for planning between regions with box world
###############################################################################
def split_box_bounds_for_rrt(region1,region2,box_bounds,overlaps):
    new_box_bounds = []
    for region,set in box_bounds:
        if region != region1 and region!= region2 and region not in overlaps[region1] and region not in overlaps[region2]:
            new_box_bounds.append(set)
    return new_box_bounds

###############################################################################
#Find bounds of single region
###############################################################################
def single_region_bounds_for_rrt(region,bounds,overlaps):
    new_bounds = []
    for regions,set in bounds:
        if regions != region and regions not in overlaps[region]:
            new_bounds.append((set))
    return(new_bounds)


###############################################################################
#Find bounds of single region in box world
###############################################################################
def single_region_box_bounds_for_rrt(region,box_bounds,overlaps):
    new_box_bounds = []
    for regions,set in box_bounds:
        if regions != region and regions not in overlaps[region]:
            new_box_bounds.append((set))
    return(new_box_bounds)


###############################################################################
#Find a random point in a given region given past trajectories
###############################################################################
def random_point_in_region(region,radius=1,past_paths=None,current_time=None,other_obstacles=[],over_inc=None,plot_region_bounds=None):
    
    if plot_region_bounds == None:
        bounds = [0+radius+.1, 160-radius-.1, 0+radius+.1, 60-radius-.1]
    else:
        bounds = [plot_region_bounds[0]+radius+.1, plot_region_bounds[1]-radius-.1, plot_region_bounds[2]+radius+.1, plot_region_bounds[3]-radius-.1]
    if len(other_obstacles) > 0:
        non_region_obstacles = other_obstacles
    else:
        non_region_obstacles = [[0,0,0],[0,0,0]]
    past_obs = []
    if over_inc == None:
        if past_paths != None and current_time != None:
            for i in range(0,len(past_paths)):
                past_obs.append((past_paths[i][current_time]+[radius]))
            non_region_obstacles = np.concatenate((non_region_obstacles,past_obs))
    else:
        if past_paths != None and current_time != None:
            for i in range(0,len(past_paths)):
                for y in range(0,over_inc):
                    if current_time-y > 0:
                        #print current_time
                        #print y
                        past_obs.append((past_paths[i][(current_time-y)]+[radius]))
                    past_obs.append((past_paths[i][(current_time+y)]+[radius]))
            non_region_obstacles = np.concatenate((non_region_obstacles,past_obs))
    no_point = True
    count = 0
    while no_point == True:
        if count > 1000:
            #print('Point Stuck in Unforseen Obstacle - Please Try Again')
            break
        else:
            randx = np.multiply((np.multiply(np.random.rand(),2)-1),(region[2])) + region[0]
            randy = np.multiply((np.multiply(np.random.rand(),2)-1),(region[2])) + region[1]
            in_obs = False
            for ox,oy,size in non_region_obstacles:
                d = np.power((np.power(ox-randx,2)+np.power(oy-randy,2)),0.5)
                if d < size+radius+1:
                    in_obs = True
            if in_obs == False:

                if randx < bounds[0] or randx > bounds[1] or randy < bounds[2] or randy > bounds[3]:
                    no_point = True
                else:
                    no_point = False
            count = count+1


    rand_point = [randx,randy]
    #print(non_region_obstacles,rand_point)
    return(rand_point)




###############################################################################
#Generate Individual Region Trajectories
###############################################################################
def assign_caps_to_trajs(states,edges,preds,sim_time,replan_agent=None,grave_state=None,start_time=0,replan_agent_idx=None,replanning_time=None):
    #grave_state needs to be just an int
    #
    # Format of the `states` variable:
    #   column 1:   State number
    #   column 2:   Timestep
    #   column 3:   Capability number
    #   column 4:   Number of agents in the state with that capability at that timestep
    #
    # **The states matrix only records information for states and timesteps with nonzero column 4 entries.**
    # It essentially tracks all timesteps and states that have nonzero capabilities in that state.  
    
    end_time = sim_time
    caps = np.unique(states[states[:,1]==0,2])
    agent_caps = []
    num_caps = np.zeros(len(caps))
    cap_idx = np.zeros(len(caps))
    counts = 0
    init_states = states[states[:,1]==0]

    time_stamps = np.asarray(states[:,1])

    #Count agents and capabilities
    agent_caps, num_caps = get_agent_capabilities(states)
    agent_caps = np.array(agent_caps)

    num_agents = np.sum(num_caps).astype(int)
    agent_positions = np.empty((end_time,num_agents),dtype=object)
    transit_times = np.zeros((end_time,num_agents))

    # initialize trajectory
    # NOTE: The initial row agent_positions[0] corresponds to MILP time step 0 when start_time = 0.
    for i in caps:
        num_of_type = 0
        choice_vals = np.where(states[states[:,1]==0,2]==i)
        for j in choice_vals[0]:
            past_num = num_of_type
            num_of_type = num_of_type + init_states[j,3]
            region_of_type = init_states[j,0]
            cap_idxs = np.where(agent_caps == i)
            for k in range(past_num,num_of_type):
                agent_positions[0,cap_idxs[0][k]] = (region_of_type.astype(int))

    edge_timer = np.zeros(num_agents)
    
    for t in range(start_time,sim_time-1):

        current_edge = edges[edges[:,2]==t]
        already_assigned = np.zeros(num_agents)
        #Update Edge Timing
        transit_times[t] = edge_timer
        edge_timer = edge_timer-1
        edge_timer[edge_timer<0] = 0
        #Prevent moving agents from being assigned
        already_assigned[edge_timer>0] = 1

        # Handle all of the failed agents.
        # This assigns them to the grave state and ensures they are unavailable
        # for assignment to other states / trajectories.
        if not replan_agent_idx == None and t >= replanning_time:
            for dx in range(0,len(replan_agent_idx)):
                already_assigned[replan_agent_idx[dx]] = 1
                agent_current_state = agent_positions[t][replan_agent_idx[dx]]
                if isinstance(agent_current_state, list):
                    # The dead agent should be traversing the edge to the grave state.
                    # Have it arrive at the grave state at the next time step.
                    agent_positions[t+1][replan_agent_idx[dx]] = grave_state
                else:
                    if agent_current_state != grave_state:
                        # Put the dead agent on the edge from its current state to the grave state.
                        agent_positions[t+1][replan_agent_idx[dx]] = [agent_current_state, grave_state, 1]
                    else:
                        # The agent is already in the grave state; have it continue there.
                        agent_positions[t+1,replan_agent_idx[dx]] = grave_state

        # Handle all non-failed agents.
        #insert line where if edge_timer > 0 with -1 then assigned
        for i in caps:
            num_of_type = 0
            choice_vals = np.where(current_edge[:,4]==i)
            for j in choice_vals[0]:
                eval_edge = current_edge[j,:]
                # Skip this edge if it's the edge connecting the grave state to itself.
                # The dead agents on this edge were already assigned there above.
                if eval_edge[1] == grave_state:
                    continue

                not_assigned = np.where(already_assigned== 0)
                current_cap = np.where(agent_caps==eval_edge[4])
                not_assigned = np.intersect1d(not_assigned,current_cap)
                free_agents = np.where(agent_positions[t,not_assigned] == eval_edge[0])
                available = not_assigned[free_agents[0]]
                available = np.asarray(available)

                # TODO: Ignore the grave_state self-edges.

                if available.size < eval_edge[5]:
                    pdb.set_trace() if sys.flags.debug else None
                    print('error time: ',t)
                    print('edge requested: ',eval_edge)
                    print('agent_positions: ', agent_positions[t])
                    print('available:',available)
                    print('available: ',available.size,' asking for: ',eval_edge[5])
                    exit('Requested Agents Not Available')
                for itr in range(0,eval_edge[5]):
                    if eval_edge[0] == eval_edge[1]:
                        #Agents stay in the same place
                        already_assigned[available[itr]] = 1
                        edge_timer[available[itr]] = 1
                        agent_positions[t+1][available[itr]] = eval_edge[0]
                    else:
                        #Agents enter edge
                        already_assigned[available[itr]] = 1
                        edge_timer[available[itr]] = eval_edge[3]
                        for idx in range(0,eval_edge[3]):
                            time_idx = t+idx+1
                            assert time_idx < sim_time
                            if idx >= eval_edge[3]-1:
                                agent_positions[time_idx][available[itr]] = eval_edge[1]
                            else:
                                agent_positions[time_idx][available[itr]] = [eval_edge[0],eval_edge[1],eval_edge[3]]

    # Update edge timer at last timestep
    transit_times[sim_time-1] = edge_timer

    #Catch agents that stay at states for multiple time steps
    for t in range(0,sim_time):
        for n in range(0,num_agents):
            if transit_times[t,n] == 0:
                if agent_positions[t,n] == None:
                    agent_positions[t,n] = agent_positions[t-1,n]

    return agent_positions,caps,num_agents,transit_times,agent_caps


###############################################################################
#Get agent capabilities
###############################################################################
def get_agent_capabilities(states):
    '''
    Returns a list of agent capabilities. 
    
    The order of capabilities in the list corresponds to the columns in 
    agent_positions, but does **not** correspond to the agent order in the
    casefile agent list.
    '''
    capabilities = np.unique(states[states[:,1]==0,2])
    agent_capabilities = []
    num_capabilities = np.zeros(len(capabilities))
    counts = 0
    init_states = states[states[:,1]==0]

    #Count agents and capabilities
    for i in capabilities:
        choice_vals = np.where(states[states[:,1]==0,2]==i)
        num_capabilities[counts]=np.sum(init_states[choice_vals,3])
        for j in range(0,num_capabilities[counts].astype(int)):
            agent_capabilities.append(i)
        counts = counts+1

    return agent_capabilities, num_capabilities


###############################################################################
#Get a consistent mapping from agent index to trajectory
###############################################################################
def get_agent_map(agents, agent_positions, agent_caps, classes):
    # Input:
    # list of agents
    # agent_positions and agent_caps from assign_caps_to_trajs
    # classes from compute_agent_classes

    # Output: a  dictionary mapping agents to trajectories

    # list of agents that haven't been assigned yet
    unmapped_idx = [x for x in range(len(agents))]
    # build list of agent init_states and caps
    init_states = [int(ag[0].split('q')[1]) for ag in agents]
    ag_caps = [classes[frozenset(ag[1])] for ag in agents]

    ag_dict = {}

    # for each agent, give them an index in agent_positions (and thereby agent_caps)
    for x in range(len(agents)):
        c = ag_caps[x]
        s = init_states[x]
        ag_idx = [y for y in unmapped_idx if agent_positions[0][y]==s and agent_caps[y]==c][0]
        unmapped_idx.remove(ag_idx)
        ag_dict[x] = ag_idx

    # return map of agent to index in agent_positions
    return ag_dict

###############################################################################
#Determine Which Agents to Drop Out From Request
###############################################################################
def which_agent_down(the_plan,ts,agent_caps,where,quant,cap,time,step_time,world_max):
    #need to determine dropout agents given cap,qant,time,where
    #dropout_agents is [#,#,#] with # as agent index
    dropout_agents = []
    for a in range(0,len(agent_caps)):
        if agent_caps[a] in cap:
            for test_points in the_plan[a][time:time+step_time]: 
                #print(test_points)              
                point = Point(test_points[0],world_max[1]-test_points[1])
                polygon = Polygon(ts.g.node[where]['shape'])
                #print(ts.g.node[where]['shape'])
                #print(polygon.contains(point))
                if polygon.contains(point):
                    if len(dropout_agents) < quant:
                        dropout_agents.append(a)
                        break  
    return dropout_agents


###############################################################################
#Generate Region Bound Definitions
#We also determine overlapping regions here
###############################################################################
def define_region_bounds(ts,state):
    
    region_bounds = []
    box_bounds_obstacleList = []
    overlapping_regions = []
    for state in ts.g.node:
        overlapping_regions.append([])
        #print(state)
        state_int = np.int(state[1:len(state)])
        bounds = ts.g.node[state]['shape']
        num_points = len(bounds)
        x_max = -10000
        x_min = 10000
        y_max = -10000
        y_min = 10000
        for idx in range(0,num_points):

            if bounds[idx][0] < x_min:
                x_min = bounds[idx][0]
            if bounds[idx][0] > x_max:
                x_max = bounds[idx][0]
            if bounds[idx][1] < y_min:
                y_min = bounds[idx][1]
            if bounds[idx][1] > y_max:
                y_max = bounds[idx][1]
        dia_x = np.divide((x_max - x_min),2)
        dia_y = np.divide((y_max - y_min),2)
        cent_x = np.average([x_min,x_max])
        cent_y = np.average([y_min,y_max])

        diameter = np.min([dia_x,dia_y])
        region_bounds.append([state_int,[cent_x,cent_y,diameter]])
        box_bounds_obstacleList.append([state_int,[x_min,x_max,y_min,y_max]])

    for region1 in box_bounds_obstacleList:
        for region2 in box_bounds_obstacleList:
            if not region1[0] == region2[0]:
                x1_min = region1[1][0]
                x1_max = region1[1][1]
                y1_min = region1[1][2]
                y1_max = region1[1][3] 
                x2_min = region2[1][0]
                x2_max = region2[1][1] 
                y2_min = region2[1][2]
                y2_max = region2[1][3]
                if not ((x1_min >= x2_max) or (x1_max <= x2_min) or (y1_max <= y2_min) or (y1_min >= y2_max)):
                    overlapping_regions[region1[0]].append(region2[0])
    return region_bounds,box_bounds_obstacleList,overlapping_regions


###############################################################################
#Generate Trajectories from a .sol file
###############################################################################
def run_planner(m,ts,data,other_past_paths=[], show_sol=None):
    start = timer.time()
    
    #get all the environment variables from m
    plot_region_bounds = m.plot_region_bounds
    obstaclesList = m.local_obstacles
    max_attempts = m.max_attempts
    max_rrt_time = m.max_rrt_time
    agent_radius = m.agent_radius
    planning_step_time = m.planning_step_time
    cap_height_map = m.cap_height_map
    record_sol = m.record_sol
    if show_sol is None:
        show_sol = m.show_sol
    grave_state = m.grave_state
    
    
    for u, v in ts.g.edges():
        assert 'grave' in ts.g.node[v]['prop'] or ts.g.has_edge(v,u)

    #Take the data and parse it into state and edge transitions
    states,edges,end_time,preds = read_sol_data(data,ts)
    sim_time = end_time

    print('time_vals',sim_time, planning_step_time)

    ###############################################################################
    #Generate individual region level trajectories
    ###############################################################################
    agent_positions,caps,num_agents,transit_times,agent_caps = assign_caps_to_trajs(states,edges,preds,sim_time)

    if isinstance(ts, ReducedTs):
        # this TS has been reduced (some states removed)
        # need to update the agent positions by reinserting states/edges
        # and then use the original TS
        expand_agent_positions(ts, agent_positions)
        ts = ts.orig_ts
    
    ###############################################################################
    #Generate Region Bound Definitions
    ###############################################################################
    region_bounds,box_bounds_obstacleList,overlapping_regions = define_region_bounds(ts,states)
   

    ###############################################################################
    #Obstacles and capability differentiation
    ###############################################################################
    #Include any unforseen or hard coded obstacles or force agents to be on different planes of operation
    #cap_height_map = np.array([1,0,0,0,0,0,0,2,2,2])
   
    #obstaclesList  = [[
    #    [0,0,0],
    #    [0,0,0]
    #],
    #[
    #    [0,0,0],
    #    [0,0,0]
    #],
    #[
    #    [0,0,0],
    #    [0,0,0]
    #]
    #]
    
    ###############################################################################
    #Generate low level trajectories
    ###############################################################################
    #I do this sequentally so it is less book keeping. It shouldnt really matter
    #since the paths are computed sequentally in rrt anyway.

    #agent_radius = [6,4,4,4,4,4,4,2,2,2]
    #agent_radius = np.multiply(np.ones(10),2)

    #planning_step_time = 10
    found_the_plan = False
    restart_main_loop =False
    #Main Loop Begins Here:
    while found_the_plan == False:
        restart_main_loop = False
        the_plan = np.empty((num_agents,int(np.multiply(planning_step_time,sim_time))),dtype=object)
        past_paths = []
        count_idx = 0
        past_a = []

        for a in np.random.permutation(range(0,num_agents)):
            if restart_main_loop == True:
                found_the_plan = False
                print('found: ',a)
                break
            #print(a)
        ################################################################################
        #Determine which past paths to consider given agent capabilities (flying or not)
        ################################################################################
            past_path_idxs = np.where(cap_height_map == cap_height_map[a])
            to_avoid = np.intersect1d(past_path_idxs,past_a)
            
            cap_past_paths = []
            #if other_past_paths == []: 
            #    if len(to_avoid) != 0:
            #        for num in range(0,len(to_avoid)):
            #            index = np.where(past_a == to_avoid[num])
            #            cap_past_paths.append(past_paths[index[0][0]])
            #    else:
            #        cap_past_paths = None
            
            
            
        ####################### MAY NEED TO REMOVE THIS ################################
        #THIS MAKES ALL AGENTS INTO LARGE OBSTACLES FOR EXPERIMENT#
            #else: 
            #    cap_past_paths.append(past_paths)
            #print('other_shape: ',np.shape(other_past_paths),'orig_shape ',np.shape(past_paths))
            for paths in other_past_paths: 
                cap_past_paths.append(paths)
            for paths in past_paths:
                cap_past_paths.append(paths)
            obstacles = obstaclesList[cap_height_map[a]]    
            #print(np.shape(cap_past_paths))
        ################################################################################
        #Cycle through actions for agent
        ################################################################################

            for t in range(0,sim_time):
                if restart_main_loop == True:
                    found_the_plan = False
                    break
                start_point = None
                time_idx = int(np.multiply(planning_step_time,t))
                if np.size(the_plan[a,time_idx]) < 2:
                    action = agent_positions[t,a]
                    #if a == 6:
                    #    print(agent_positions[:,6],action,t,time_idx)
                    if time_idx-1 >= 0:
                        if np.size(the_plan[a,time_idx-1])>1:
                            start_point = the_plan[a,time_idx-1]
                        else:
                            start_point = None
                    else:
                        start_point = None


                    ##Find Random Start Positions:
        ###############################################################################
        #Determine start positions
        ###############################################################################

                    if np.size(action) == 1 and not (grave_state in [action]):
                        if np.size(start_point) > 1:
                            collision = False
                            for inc in range(0,planning_step_time):
                                if (collision_check(start_point,time_idx+inc,agent_radius[a],cap_past_paths)):
                                        collision = True
                            if collision:
                                for region,set in region_bounds:
                                    if region == action:
                                        if count_idx>0:
                                            other_regions = single_region_bounds_for_rrt(action,region_bounds,overlapping_regions)
                                            other_regions = np.concatenate((obstacles,other_regions))
                                            other_box_regions = single_region_box_bounds_for_rrt(action,box_bounds_obstacleList,overlapping_regions)
                                            rand_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,current_time=time_idx,other_obstacles=other_regions,over_inc=planning_step_time,plot_region_bounds=plot_region_bounds)
                                        else:
                                            other_regions = single_region_bounds_for_rrt(action,region_bounds,overlapping_regions)
                                            other_regions = np.concatenate((obstacles,other_regions))
                                            other_box_regions = single_region_box_bounds_for_rrt(action,box_bounds_obstacleList,overlapping_regions)
                                            rand_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,other_obstacles=other_regions,over_inc=planning_step_time,plot_region_bounds=plot_region_bounds)

                                if cap_past_paths!=None:
                                    timed_past_paths = []
                                    for w in range(0,len(cap_past_paths)):
                                        timed_past_paths.append(cap_past_paths[w][(time_idx):])
                                else:
                                    timed_past_paths = None

                                other_regions = np.concatenate((other_regions,obstacles))
                                no_short_path = True
                                collision_idx = 0
                                while no_short_path == True:
                                    candidate_path = RRT_Get_Path(regions_to_avoid=other_regions,start=start_point,goal=rand_point,past_paths=timed_past_paths,start_region=None,goal_region=None,agentnum=a,agent_radius=agent_radius,expandDis=1,bounds=plot_region_bounds,max_rrt_time=max_rrt_time,box_bounds_obs_regions=other_box_regions)

                                    len_cand_path = len(candidate_path)
                                    if len_cand_path > planning_step_time or len_cand_path == 0:
                                        collision_idx +=1
                                        if collision_idx >4:
                                            restart_main_loop = True
                                            found_the_plan = False
                                            break
                                        for region,set in region_bounds:
                                            if region == action:
                                                if count_idx>0:
                                                    other_regions = single_region_bounds_for_rrt(action,region_bounds,overlapping_regions)
                                                    other_regions = np.concatenate((obstacles,other_regions))
                                                    rand_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,current_time=time_idx,other_obstacles=other_regions,over_inc=planning_step_time,plot_region_bounds=plot_region_bounds)
                                                else:
                                                    other_regions = single_region_bounds_for_rrt(action,region_bounds,overlapping_regions)
                                                    other_regions = np.concatenate((obstacles,other_regions))
                                                    rand_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,other_obstacles=other_regions,over_inc=planning_step_time,plot_region_bounds=plot_region_bounds)

                                    elif len_cand_path == planning_step_time:
                                        no_short_path = False
                                        the_plan[a,time_idx:time_idx+planning_step_time] = candidate_path
                                    else:
                                        diff_len = planning_step_time-len_cand_path
                                        the_plan[a,time_idx:time_idx+len_cand_path]=candidate_path
                                        for dif_inc in range(0,diff_len):
                                            the_plan[a,time_idx+len_cand_path+dif_inc] = candidate_path[len_cand_path-1]
                                        no_short_path = False

                            else:
                                for inc in range(0,planning_step_time):
                                    the_plan[a,time_idx+inc] = start_point
                                #Check if this point is in any new past paths
                        else:
                            for region,set in region_bounds:
                                if region == action:
                                    if count_idx>0:
                                        other_regions = single_region_bounds_for_rrt(action,region_bounds,overlapping_regions)
                                        other_regions = np.concatenate((obstacles,other_regions))
                                        rand_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,current_time=time_idx,other_obstacles=other_regions,over_inc=planning_step_time,plot_region_bounds=plot_region_bounds)
                                    else:
                                        other_regions = single_region_bounds_for_rrt(action,region_bounds,overlapping_regions)
                                        #print other_regions
                                        #print obstacles
                                        other_regions = np.concatenate((obstacles,other_regions))
                                        rand_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,other_obstacles=other_regions,over_inc=planning_step_time,plot_region_bounds=plot_region_bounds)
                            for inc in range(0,planning_step_time):
                                the_plan[a,time_idx+inc] = rand_point
        #If agent needs to transistion, this determines start and end points
                    elif np.size(action) == 3 and not (grave_state in [action]):
                        #transition
                        if time_idx-1>=0:
                            if np.size(the_plan[a,time_idx-1])>1:
                                start_point = np.asarray(the_plan[a,time_idx-1])
                                start_region = None
                                for region,set in region_bounds:
                                    if region == action[1]:
                                        other_regions = single_region_bounds_for_rrt(action[1],region_bounds,overlapping_regions)
                                        other_regions = np.concatenate((obstacles,other_regions))
                                        goal_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,other_obstacles=other_regions,over_inc=planning_step_time,plot_region_bounds=plot_region_bounds)
                                        if count_idx > 0:
                                            other_regions = single_region_bounds_for_rrt(action[1],region_bounds,overlapping_regions)
                                            other_regions = np.concatenate((obstacles,other_regions))
                                            goal_point = random_point_in_region(set,agent_radius[a],past_paths =cap_past_paths,current_time =time_idx+((action[2]-1)*planning_step_time),other_obstacles=other_regions,over_inc=planning_step_time,plot_region_bounds=plot_region_bounds)
                            else:
                                start_point = None
                                for region,set in region_bounds:
                                    if region == action[0]:
                                        other_regions = single_region_bounds_for_rrt(action[0],region_bounds,overlapping_regions)
                                        other_regions = np.concatenate((obstacles,other_regions))
                                        start_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,current_time = time_idx,other_obstacles=other_regions,over_inc=planning_step_time,plot_region_bounds=plot_region_bounds)
                                    if region == action[1]:
                                        other_regions = single_region_bounds_for_rrt(action[1],region_bounds,overlapping_regions)
                                        other_regions = np.concatenate((obstacles,other_regions))
                                        goal_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,other_obstacles=other_regions,plot_region_bounds=plot_region_bounds)
                                        if count_idx > 0:
                                            goal_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,current_time = time_idx+((action[2]-1)*planning_step_time),other_obstacles=other_regions,over_inc=planning_step_time,plot_region_bounds=plot_region_bounds)
                        else:
                            start_point = None
                            for region,set in region_bounds:
                                if region == action[0]:
                                    other_regions = single_region_bounds_for_rrt(action[0],region_bounds,overlapping_regions)
                                    other_regions = np.concatenate((obstacles,other_regions))
                                    goal_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,current_time = 0,other_obstacles=other_regions,over_inc=planning_step_time,plot_region_bounds=plot_region_bounds)
                                if region == action[1]:
                                    other_regions = single_region_bounds_for_rrt(action[1],region_bounds,overlapping_regions)
                                    other_regions = np.concatenate((obstacles,other_regions))
                                    goal_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,current_time = 0,other_obstacles=other_regions,over_inc=planning_step_time,plot_region_bounds=plot_region_bounds)
                        regions_to_avoid = split_bounds_for_rrt(action[0],action[1],region_bounds,overlapping_regions)
                        box_regions_to_avoid = split_box_bounds_for_rrt(action[0],action[1],box_bounds_obstacleList,overlapping_regions)
                        #print 'obstacles: ',obstacles
                        #print 'regions_to_avoid: ',regions_to_avoid
                        #print 'action: ', action
                        if len(regions_to_avoid):
                            regions_to_avoid = np.concatenate((obstacles,regions_to_avoid))

                        if cap_past_paths!=None:
                            timed_past_paths = []
                            for w in range(0,len(cap_past_paths)):
                                timed_past_paths.append(cap_past_paths[w][(time_idx):])
                        else:
                            timed_past_paths = None


        ################################################################################
        #Compute RRT path
        ################################################################################

                        no_viable_path = True
                        loop_idx = 0
                        while no_viable_path==True:
                            loop_idx += 1
                            if loop_idx > max_attempts:
                                restart_main_loop = True
                                found_the_plan = False
                                break
                            if count_idx > 0:
                                #regions_to_avoid=regions_to_avoid
                                candidate_path = RRT_Get_Path(regions_to_avoid=regions_to_avoid,start=start_point,goal=goal_point,past_paths=timed_past_paths,start_region=None,goal_region=None,agentnum=a,agent_radius=agent_radius,expandDis=1,bounds=plot_region_bounds,max_rrt_time=max_rrt_time,box_bounds_obs_regions=box_regions_to_avoid)
                            else:
                                candidate_path = RRT_Get_Path(regions_to_avoid=regions_to_avoid,start=start_point,goal=goal_point,start_region=None,goal_region=None,agentnum=a,agent_radius=agent_radius,expandDis=1,bounds=plot_region_bounds,max_rrt_time=max_rrt_time,box_bounds_obs_regions=box_regions_to_avoid)
                            len_cand_path = len(candidate_path)
                            plan_time_bound = np.multiply(action[2]-1,planning_step_time)
                            if len_cand_path == 0:
                                if time_idx-1>=0:
                                    if np.size(the_plan[a,time_idx-1])>1:
                                        start_point = np.asarray(the_plan[a,time_idx-1])
                                        start_region = None
                                        for region,set in region_bounds:
                                            if region == action[1]:
                                                other_regions = single_region_bounds_for_rrt(action[1],region_bounds,overlapping_regions)
                                                other_regions = np.concatenate((obstacles,other_regions))
                                                goal_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,other_obstacles=other_regions,plot_region_bounds=plot_region_bounds)
                                                if count_idx > 0:
                                                    other_regions = single_region_bounds_for_rrt(action[1],region_bounds,overlapping_regions)
                                                    other_regions = np.concatenate((obstacles,other_regions))
                                                    goal_point = random_point_in_region(set,agent_radius[a],past_paths =cap_past_paths,current_time =t+((action[2]-1)*planning_step_time),other_obstacles=other_regions,plot_region_bounds=plot_region_bounds)
                                    else:
                                        start_point = None
                                        for region,set in region_bounds:
                                            if region == action[0]:
                                                other_regions = single_region_bounds_for_rrt(action[0],region_bounds,overlapping_regions)
                                                other_regions = np.concatenate((obstacles,other_regions))
                                                start_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,other_obstacles=other_regions,plot_region_bounds=plot_region_bounds)
                                            if region == action[1]:
                                                other_regions = single_region_bounds_for_rrt(action[1],region_bounds,overlapping_regions)
                                                other_regions = np.concatenate((obstacles,other_regions))
                                                goal_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,other_obstacles=other_regions,plot_region_bounds=plot_region_bounds)
                                                if count_idx > 0:
                                                    goal_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,current_time = t+((action[2]-1)*planning_step_time),other_obstacles=other_regions,plot_region_bounds=plot_region_bounds)
                                else:
                                    start_point = None
                                    for region,set in region_bounds:
                                        if region == action[0]:
                                            other_regions = single_region_bounds_for_rrt(action[0],region_bounds,overlapping_regions)
                                            other_regions = np.concatenate((obstacles,other_regions))
                                            goal_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,other_obstacles=other_regions,plot_region_bounds=plot_region_bounds)
                                        if region == action[1]:
                                            other_regions = single_region_bounds_for_rrt(action[1],region_bounds,overlapping_regions)
                                            other_regions = np.concatenate((obstacles,other_regions))
                                            goal_point = random_point_in_region(set,agent_radius[a],past_paths = cap_past_paths,other_obstacles=other_regions,plot_region_bounds=plot_region_bounds)

                            elif len_cand_path <= plan_time_bound:
                                #put in loop to make extra hold points, and then slap that on the path and add it to the_plan
                                temp_hold = []
                                extra_hold = plan_time_bound - len_cand_path
                                hold_point = candidate_path[len_cand_path-1]
                                for extra in range(0,extra_hold):
                                    temp_hold.append(hold_point)
                                if np.size(temp_hold) > 1:
                                    candidate_path = np.concatenate((candidate_path,temp_hold))
                                else:
                                    candidate_path = candidate_path
                                for step in range(0,plan_time_bound):
                                    the_plan[a,time_idx+step] = candidate_path[step]

                                no_viable_path = False
                            else:
                                print(len_cand_path,plan_time_bound)
                                no_viable_path = True

                    elif (grave_state in [action]):
                        print("its dead")
                        for iters in range(t*planning_step_time,sim_time*planning_step_time):
                            the_plan[a,iters] = np.asarray([-100, -100])
                        break
                    else:
                        print('Unexpected Trajectory Sequence - Edges should be [start,end,time]')
            #print(restart_main_loop,found_the_plan)
            if restart_main_loop == True:
                found_the_plan = False
                break
            else:
                found_the_plan = True
            count_idx += 1
            past_a.append(a)
                    #if a == 6:
                    #    print(the_plan[a],action,t,time_idx)
            past_formatted = []
            #print(the_plan[a])
            for k in range(0,np.multiply(planning_step_time,sim_time)):
                past_formatted.append([the_plan[a][k][0],the_plan[a][k][1]])

            past_paths.append(past_formatted)


    the_plan = np.array(the_plan)

    # Draw final path


        #plt.plot([x for (x, y) in smoothedPath], [
        #    y for (x, y) in smoothedPath], '-b')
        #plt.plot(start[i][0], start[i][1], "or")
        #plt.plot(goal[i][0], goal[i][1], "xr")
    #plt.xlim(plot_bounds[0],plot_bounds[1])
    #plt.ylim(plot_bounds[2],plot_bounds[3])
    #plt.axis(plot_bounds)
    #plt.grid(True)
    #for (x, y, size) in obstacleList:
    #    PlotCircle(x, y, size,plot_bounds)
    #plt.pause(0.01)
    #plt.grid(True)
    #plt.pause(0.01)  # Need for Mac
    #plt.show()


    #new_bounds = split_bounds_for_rrt(0,1,region_bounds)

    #RRT_Get_Path(new_bounds,start=[(5,5)],goal=[(5,50)])


    end = timer.time()

    print('runtime:')
    print(end-start)
    runtime = end-start

    if show_sol:
        ###############################################################################
        #Print the Final Trajectories
        caps_idxs = np.unique(agent_caps)
        fig = plt.figure()
        plt.clf()
        fig,viewport = show_world(ts,fig)
        for i in range(0,num_agents):
            if agent_caps[i] == caps_idxs[0]:
                plt.plot([x for (x, y) in the_plan[i] if not x == -100], [y for (x, y) in the_plan[i] if not y == -100], '-k')
            elif agent_caps[i] == caps_idxs[1]:
                plt.plot([x for (x, y) in the_plan[i] if not x == -100], [y for (x, y) in the_plan[i] if not y == -100], ':k')
            elif agent_caps[i] == caps_idxs[2]:
                plt.plot([x for (x, y) in the_plan[i] if not x == -100], [y for (x, y) in the_plan[i] if not y == -100], '--g')
            elif agent_caps[i] == caps_idxs[3]:
                plt.plot([x for (x, y) in the_plan[i] if not x == -100], [y for (x, y) in the_plan[i] if not y == -100], '-.r')
            elif agent_caps[i] == caps_idxs[4]:
                plt.plot([x for (x, y) in the_plan[i] if not x == -100], [y for (x, y) in the_plan[i] if not y == -100], '-.b')


        plt.axis(plot_region_bounds)
        plt.show(block=False)
        plt.pause(1)
        plt.waitforbuttonpress()
        plt.clf()

        ###############################################################################
        #Play a video of the Trajectories
    # if record_sol:
    #     metadata = dict(title='Traj_Sol', artist='Matplotlib',comment='Movie support!')
    #     writer = FFMpegWriter(fps=12, metadata=metadata)
    #     writer.setup(fig,'pre_replan_robust.mp4',1000)

    #     for time in range(0,(sim_time)):
    #         plt.clf()
    #         fig,viewport = show_world(ts,fig)
    #         for i in range(0,num_agents):
    #             if agent_caps[i] == 1:
    #                 plt.plot(the_plan[i][time][0], the_plan[i][time][1],color="r",marker='x',markersize=15)
    #             elif agent_caps[i] == 2:
    #                 plt.plot(the_plan[i][time][0], the_plan[i][time][1],"oc",markersize=15)
    #             elif agent_caps[i] == 3:
    #                 plt.plot(the_plan[i][time][0], the_plan[i][time][1],"*w",markersize=15)
    #             elif agent_caps[i] == 8:
    #                 plt.plot(the_plan[i][time][0], the_plan[i][time][1],"hk",markersize=12)


    #         plt.axis(plot_region_bounds)
    #         plt.title('Time: ' + str(np.divide(time,10)))
    #         writer.grab_frame()
    #         plt.show(block=False)
    #         plt.pause(0.01)


    return(the_plan,agent_caps,past_paths)
    
    
    

    
    
    
    
    
def run_planner_cpp(m,ts,data):
    start = timer.time()
    
    #get all the environment variables from m
    plot_region_bounds = m.plot_region_bounds
    obstaclesList = m.local_obstacles
    max_attempts = m.max_attempts
    max_rrt_time = m.max_rrt_time
    agent_radius = m.agent_radius
    planning_step_time = m.planning_step_time
    cap_height_map = m.cap_height_map
    record_sol = m.record_sol
    show_sol = m.show_sol
    grave_state = m.grave_state
    
    
    # for u, v in ts.g.edges():
        # assert 'grave' in ts.g.node[v]['prop'] or ts.g.has_edge(v,u)

    #Take the data and parse it into state and edge transitions
    states,edges,end_time,preds = read_sol_data(data,ts)
    sim_time = end_time

    print('time_vals',sim_time, planning_step_time)

    ###############################################################################
    #Generate individual region level trajectories
    ###############################################################################
    agent_positions,caps,num_agents,transit_times,agent_caps = assign_caps_to_trajs(states,edges,preds,sim_time)

    # Get the dictionary mapping casefile agent indices to columns in agent_positions matrix.
    agent_capabilities = get_agent_capabilities(states)[0]
    agent_classes = compute_agent_classes(m.agents, compute_capability_bitmap(m.agents))
    agent_index_dict = get_agent_map(m.agents, agent_positions, agent_capabilities, agent_classes)


    # TODO: Change to use actually useful task function instead of create_random_tasks()
    agent_tasks = create_random_tasks(agent_positions, agent_index_dict)
    output_strings = agent_positions_to_MOOS_strings(agent_positions, num_agents, agent_index_dict, agent_tasks=agent_tasks)

    return agent_positions, output_strings, agent_tasks, agent_index_dict




def agent_positions_to_MOOS_strings(agent_positions, num_agents, agent_index_dict, agent_tasks=None):
    '''
    Parses agent positions and tasks into string form to send to MOOS.

    The final format of the output strings is "state1-state1:start_time-end_time:task1; state1-state2:start_time:end_time:None; ..."

    The task "None" specifies that the agent does not have a task. Note that all agents have the task "None" when
    traversing edges between states.

    Args:
        agent_positions (list): A list of lists containing the positions of each agent at each time step. The first
            index corresponds to time step, and the second index corresponds to agent number. For example,
            ``agent_positions[ii][jj]`` is the position of the jjth agent at time step ii. If agent jj is in a state
            at time ii, then ``agent_positions[ii][jj]`` is an integer corresponding to the state (e.g. 7 for state 
            Q7). If agent jj is traversing an edge between states at time ii, then ``agent_positions[ii][jj]`` is
            a list with entries ``[state1, state2, time_duration]`` meaning it has ``time_duration`` steps to transfer
            from ``state1`` to ``state2``.
        num_agents (int): Number of agents in the network. TODO: This is redundant; we can compute this from
            agent_positions itself.
        agent_index_dict (dict): A dictionary mapping agent number to column index (second dimension index) in
            agent_positions. The ordering of agent numbers does **not** correspond to the column order; i.e.
            the iith agent does not correspond to the iith column in ``agent_positions``. Instead, it corresponds
            to the ``agent_index_dict[ii]`` index in ``agent_positions``.
        agent_tasks (list): A list of lists similar to agent_positions, but containing the task that each agent
            is doing at each time step. **Work in progress; final format TBA.**

    Returns:
        (list): A list of strings with the format specified previously.
    '''
    # Parses agent_positions into string form to send to MOOS.
    # Format of the strings is:
    #   state1:start_time-end_time; state1-state2:start_time-end_time; state2:start_time-end_time...

    max_steps = len(agent_positions)
    output_strings = ['' for kk in range(num_agents)]

    for agent_number in range(num_agents):
        agent_column_number = agent_index_dict[agent_number] # The column number of agent_positions corresponding to agent kk
        location = agent_positions[0][agent_column_number]
        if agent_tasks is None:
            task = None
        else:
            task = agent_tasks[0][agent_column_number]

        # Get rid of obnoxious Numpy integer types
        try:
            location = int(location)
        except:
            pass

        counter = 1
        # First row of agent_positions corresponds to time step 0 when start_time = 0
        # TODO: Pass in start_time as a function argument?
        start_time = 0 

        for ii in range(1,max_steps):
            next_location = agent_positions[ii][agent_column_number]
            if agent_tasks is None:
                next_task = None
            else:
                next_task = agent_tasks[ii][agent_column_number]

            # Get rid of Numpy integer types
            try:
                next_location = int(next_location)
            except:
                pass

            if next_location == location and task == next_task:
                if ii < max_steps -1:
                    counter += 1
                else:
                    # End of the array; save the data
                    end_time = start_time + counter + 1 # +1 is for convention, even though this will make end_time = max_steps+1
                    if isinstance(next_location, list):
                        output_strings[agent_number] += 'q' + str(location[0]) + '-q' + str(location[1]) + ':' + str(start_time) + '-' + str(end_time)
                    else:
                        output_strings[agent_number] += 'q' + str(location) + '-q' + str(location) + ':' + str(start_time) + '-' + str(end_time)

                    # Add the task
                    if next_task is None:
                        output_strings[agent_number] += ':' + 'NULL' + '; '
                    else:
                        output_strings[agent_number] += ':' + agent_tasks[ii][agent_column_number] + '; '
            else:
                # Since either the location has changed or the task has changed, end the current schedule chunk and begin the next one.
                end_time = start_time + counter
                if isinstance(location, list):
                    # Location is an edge. Members of the list are [start_location, end_location, max_traversal_time]
                    output_strings[agent_number] += 'q' + str(location[0]) + '-q' + str(location[1]) + ':' + str(start_time) + '-' + str(end_time)
                else:
                    output_strings[agent_number] += 'q' + str(location) + '-q' + str(location) + ':' + str(start_time) + '-' + str(end_time)

                # Add the task
                if task is None:
                    output_strings[agent_number] += ':' + 'NULL' + '; '
                else:
                    output_strings[agent_number] += ':' + task + '; '
                
                location = next_location
                start_time = end_time
                task = next_task
                counter = 1

                # Corner case for last time step
                # Add next_location to the end of the string
                if ii == max_steps-1:
                    end_time = start_time + 1 # For sake of convention, we do this even though this will make end_time = max_time + 1
                    if isinstance(next_location, list):
                        output_strings[agent_number] += 'q' + str(next_location[0]) + '-q' + str(next_location[1]) + ':' + str(start_time) + '-' + str(end_time)
                    else:
                        output_strings[agent_number] += 'q' + str(next_location) + '-q' + str(next_location) + ':' + str(start_time) + '-' + str(end_time)
                    
                    # Add the task
                    if next_task is None:
                        output_strings[agent_number] += ':' + 'NULL' + '; '
                    else:
                        output_strings[agent_number] += ':' + next_task + '; '


    return output_strings



    
    
def create_random_tasks(agent_positions, agent_index_dict):
    '''
    Creates random tasks for agents. Placeholder function.

    Task string options are:

        - NULL
        - LOITER
        - RASTER
        - DEFENSE
        - ESCORT
        - KILLCHAIN
        - BLOCKADE

    All agents have have the task "NONE" when they are traversing an edge

    Args:
        agent_positions (list): A list of lists containing the positions of each agent at each time step. The first
            index corresponds to time step, and the second index corresponds to agent number. For example,
            ``agent_positions[ii][jj]`` is the position of the jjth agent at time step ii. If agent jj is in a state
            at time ii, then ``agent_positions[ii][jj]`` is an integer corresponding to the state (e.g. 7 for state 
            Q7). If agent jj is traversing an edge between states at time ii, then ``agent_positions[ii][jj]`` is
            a list with entries ``[state1, state2, time_duration]`` meaning it has ``time_duration`` steps to transfer
            from ``state1`` to ``state2``.
        agent_index_dict (dict): A dictionary mapping agent number to column index in the ``agent_positions matrix``. The
            column in ``agent_positions`` corresponding to agent ii is given by ``agent_index_dict[ii]``.

    Returns:
        (list): List of lists agent_tasks. The first dimension indexes time step,
        the second dimension indexes agent number. For example,
        agent_tasks[ii][jj] contains the task string for the jjth agent at time step ii
        (e.g. "BLOCKADE").
    '''

    # NOTE: The 'NULL' / 'NONE' task should always be first (index 0) to make renaming it
    #       easier.
    task_list = ['NULL', 'LOITER', 'RASTER', 'DEFENSE', 'ESCORT', 'KILLCHAIN', 'BLOCKADE']
    num_tasks = len(task_list)

    num_agents = len(agent_positions[0])
    max_steps = len(agent_positions)

    # Preallocate the agent_tasks
    agent_tasks = [[None]*num_agents for _ in range(max_steps)]

    for kk in range(num_agents):
        agent_column_number = agent_index_dict[kk]
        location = agent_positions[0][agent_column_number]
        current_task = task_list[randint(num_tasks)]
        agent_tasks[0][agent_column_number] = current_task

        try:
            location = int(location)
        except:
            pass

        for ii in range(1,max_steps):
            next_location = agent_positions[ii][agent_column_number]
            
            try:
                next_location = int(next_location)
            except:
                pass

            if next_location == location:
                agent_tasks[ii][agent_column_number] = current_task
            else:
                if isinstance(next_location, list):
                    # Agent is traversing an edge.
                    # Task is always NONE when traversing an edge.
                    current_task = task_list[0] # NULL task
                else:
                    current_task = task_list[randint(num_tasks)]

                agent_tasks[ii][agent_column_number] = current_task
                location = next_location

    return agent_tasks


def expand_agent_positions(ts, agent_positions):
    """
    Edits agent_positions in-place to reinsert removed states and edges.
    This is meant for use with reduce_ts in decomposition_functions.py, which
    removes unnecessary states and combines the edge weights.

    This function takes agent paths described by agent positions and replaces
    edges with the expanded path by reinserting removed states and edges.

    Example:
      q0 -weight:2-> q1 -weight:4-> q2

      q1 was removed by reduce_ts

      solution has q0-weight:6->q2

      this function edits the solution to go through q1 again

    Args:
        ts: the reduced TS
        agent_positions: a numpy array where each row is a time and each column is an agent
                         elements can be a state or an edge
    """

    orig_ts = ts.orig_ts
    # location_tracker: maps agent indices to a tuple (edge_num, time)
    #                   where edge_num is which collapsed edge it should be in
    #                   and time is how long it has been on that edge
    location_tracker = {i: (0, 0) for i in range(agent_positions.shape[1])}
    for t in range(agent_positions.shape[0]):
        for i in range(agent_positions.shape[1]):
            if not isinstance(agent_positions[t][i], list) or \
               orig_ts.g.has_edge(int2state(agent_positions[t][i][0]), \
                                  int2state(agent_positions[t][i][1])):
                # this is either a state, or an edge that exists in the original graph
                # nothing to do but make sure location_tracker is reset
                location_tracker[i] = (0, 0)
            else:
                startstate, endstate, _ = agent_positions[t][i]
                startstate, endstate = int2state(startstate), int2state(endstate)
                assert not orig_ts.g.has_edge(startstate, endstate)
 
                edge_num, time_spent = location_tracker[i]
                collapsed_edges = ts.g.get_edge_data(startstate, endstate)['collapsed_edges']
                edge = collapsed_edges[edge_num]
                assert orig_ts.g.has_edge(edge[0], edge[1])

                # if this is the last collapsed edge, then the
                # final state should be the same as the endstate
                assert edge_num + 1 < len(collapsed_edges) or \
                       edge[1] == endstate

                if time_spent + 1 < edge[2]['weight']:
                    # need to stay on this edge
                    # Note: converting from qi -> i with int
                    # TODO create helper functions to convert from qi <-> i
                    agent_positions[t][i] = [state2int(edge[0]), state2int(edge[1]), edge[2]['weight']]
                    location_tracker[i] = (edge_num, time_spent+1)
                else:
                    # finished this edge
                    # need to stop at the end state of this collapsed edge
                    agent_positions[t][i] = state2int(edge[1])
                    # and move to next edge
                    location_tracker[i] = (edge_num+1, 0)
