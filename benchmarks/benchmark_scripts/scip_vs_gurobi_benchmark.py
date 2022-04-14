'''
Module for benchmarking Gurobi vs Scip using PuLP. 

Author: James Usevitch (james.usevitch@ll.mit.edu)
Created: 2021
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


import os
from lomap import Ts

##########################################################
# File Names
##########################################################
ts_parent_dir = './output/IJRR_Env_Size/TransitionSystems'
ts_filename= ts_parent_dir + '/TS'
if not os.path.exists(ts_parent_dir): #Where .yaml files will be saved
    os.makedirs(ts_parent_dir)
save_parent_dir = './output/IJRR_Env_Size'
save_filename = save_parent_dir + '/Solution' #where .sol and .lp files will be saved
if not os.path.exists(save_parent_dir):
    os.makedirs(save_parent_dir)
experiment_name = './output/IJRR_Env_Size/Results_part2' #Where csv files will be saved
#This reloads prior constraints or solutions.
load_old_files = False
reload_file = None #Would be a string file name



##########################################################
# Specification
##########################################################


formula = 'F[0, 20] T(1, green, {(IR, 2), (Vis, 2)}) &&  G[20,40] F[0,10] T(1, blue, {(UV, 1), (Mo, 2)}) && F[5,25] T(2,yellow,{(UV,2),(Vis,2)}) && F[3,18] T(2,orange,{(Vis,2)}) && F[20,30] T(2,orange,{(Vis,2)})'



##########################################################
# Environment Options
##########################################################
label_names = ['green','yellow','orange','blue','white'] #Which labels to use for regions in random TSs
label_probs = [0.05,0.05,0.05,0.05,0.8] #Probability of applying label with same index in label_names
edge_weight_range=[1,3] #Range of edge weights that may appear in TS
edge_prob = 0.10 #Probability of edge between two regions
capability_list = ['UV','Mo','Vis','IR'] #full set of capabilites agents amy have
num_capabilities_per_agent = 2 #how many capabilities each agent has

num_classes = 4; #Number of distinct classes of agents

##########################################################
# Monte Carlo Options
##########################################################
verbose=True #whether to print  outputs as we go
num_trials = {16:6,20:9,25:21,30:25,36:36} #Dict that maps number of states (product of dimensions) in transition system to the number of trials per solver configuration to perform
trial_idx_start = 50 #index at which to begin numbering trials
dim_list = [[4,4],[4,5],[5,5],[5,6],[6,6]] #list of grid dimensions of TSs to be iterated over
timeLimit=6e2 #max allowable optimization time
num_agents_per_class_list = [4,5,6,7,8] #number of agents generated per class

#solver options tuple: 
#   robust (True/False), regularize (True/False), alpha (between 0 and 1), upper bound (True/False)
solver_options_list=[(False, False, 0.5,False),(True, False, 0.5,False),(True, False, 0.5,True), (False, True, 0.5,False), (True, True, 0.5,True)]
# Shorter list: omits the regularization (which takes forever)
short_solver_options_list=[(False, False, 0.5,False),(True, False, 0.5,False),(True, False, 0.5,True)]

# List of solvers for PuLP to iterate through
solvers_list = ['SCIP', 'GUROBI']


mult = 1



if __name__ == "__main__":

    import sys
    import os
    sys.path.append(os.getcwd())
    sys.path.append("../src")

    from catl_planning.route_planning import route_planning
    import csv
    import copy
    import time
    if sys.flags.debug:
        import pdb

    from RandomizedCaTLExperiments import generateRandomGridTS, generateRandomAgents, generateRandomGridTS_seed, generateRandomAgents_seed, generateRandomAgentsClass_seed
    
    m = sys.modules[__name__]

    trialTimes = []
    trialCons = []
    trialVars = []
    trialVals = []

    # Keep track of how many trials are left
    total_trials = len(num_agents_per_class_list) * sum([num_trials[dimPair[0]*dimPair[1]] for dimPair in dim_list]) * len(short_solver_options_list) * len(solvers_list)
    current_trial = 0

    csvName = experiment_name+'.csv'
    with open(csvName, 'w') as outfile:
        writer = csv.writer(outfile)
        writer.writerow(['Robust','Regularize','Alpha','Upper Bound','Trial Number','TS Filename','Solution Filename','Xdim','Ydim','Formula','Agents','Run Time (s)','Linear Constraints','Variables','Objective Value','Robustness','Normalized Travel Time', 'MILP Solver Used'])
        #Categories of information saved in output csv files
        for napc in num_agents_per_class_list:
            for dimPair in dim_list:
                if 'trial_idx_start' in dir(sys.modules[__name__]): 
                    start = trial_idx_start #start trial numbers from m.trial_idx_start > 0 if we are appending new results to old results
                else: 
                    start = 0
                
                for trialIdx in  range(start,start+num_trials[dimPair[0]*dimPair[1]]):
                    tsnameTrial = ts_filename  + '_' + str(copy.copy(dimPair))  + '_'  + str(napc*num_classes) + '_' + str(trialIdx)+ '.yaml'

                    if verbose:
                        print ("building TS") #Generate random grid TS

                    tsTrial = generateRandomGridTS_seed(tsnameTrial,copy.copy(dimPair),label_names,edge_weight_range,mult,seed=trialIdx)


                    if verbose:
                       print ("building agents") #generate random agent placement

                    agentsTrial = generateRandomAgents_seed(tsTrial.g.nodes(),capability_list,num_capabilities_per_agent,napc, num_classes,seed=trialIdx)
                    for solveOpts in short_solver_options_list : #iterate over different solver configurations
                        for solver in solvers_list:
                            solnameTrial = save_filename + '_' + str(solveOpts)  + '_' + str(copy.copy(dimPair))  + '_'  + str(napc*num_classes) + '_' + str(trialIdx) + '_' + solver
                            if verbose:
                                print ("starting Optimization")
                            t = time.time()
                            try:
                                #solve model if feasible
                                PuLP_Model,replan_data = route_planning(m,tsTrial, agentsTrial, formula,file_name=solnameTrial, replan_req=False,robust=solveOpts[0], regularize=solveOpts[1],alpha=solveOpts[2],upperBound = solveOpts[3],load_previous=False, solver=solver)
                            except Exception as err:
                                #generate new instance if infeasible
                                # TODO: modify for infeasible result?
                                print("\nroute_planning did not succeed.\n")
                                pdb.set_trace() if sys.flags.debug else None
                                continue
                            runTime = time.time()-t
                            if PuLP_Model.status == 1: # If problem was solved
                            #record results if problem can be solved
                                # feasible = True                        

                                # If there was no objective value (feasibility problem), set the objective
                                # equal to 0.
                                if PuLP_Model.objective is not None:
                                    objective_value = PuLP_Model.objective.value()
                                else:
                                    objective_value = 0

                                if verbose:
                                    print ("Optimization Complete")
                                trialTimes.append(runTime)
                                trialCons.append(PuLP_Model.numConstraints())
                                trialVars.append(PuLP_Model.numVariables())
                                trialVals.append(objective_value)
                                if "rho" in PuLP_Model.variablesDict().keys():
                                    rv = PuLP_Model.variablesDict()["rho"].value()
                                else:
                                    rv = 0 # See stl2milp function; if robust = False, then rho = 0.
                                tv = PuLP_Model.variablesDict()["normTravelTime"].value()
                                writer.writerow([solveOpts[0], solveOpts[1], solveOpts[2], solveOpts[3], trialIdx, tsnameTrial, solnameTrial, dimPair[0], dimPair[1], formula, len(agentsTrial), runTime,PuLP_Model.numConstraints(), PuLP_Model.numVariables(),objective_value,rv,tv, solver])
                                if verbose:
                                    print (runTime)
                                    print (PuLP_Model.numConstraints)
                                    print (PuLP_Model.numVariables)
                                    print (objective_value)
                            else:
                                # TODO: Write a row that shows that it is infeasible or has timed out!
                                pass

                            print(f"\n\nTrial {current_trial+1} completed out of {total_trials}\n\n")
                            current_trial += 1



    # After the test trials are completed
    print("\n\nBenchmarking Complete.\n\n")