'''
Splits problem generation into three iterators:

- Transition System
- Agents
- Formulas

Also uses Dictionaries instead of named tuples.

The problem format in this file can be used when you
would like to have some aspects of the problem fixed
while varying others, e.g.:
    
    - Fixed TS, randomly generated agents, formulas, and solver options
    - Fixed TS and agents, randomly generated formulas and solver options
    - Etc.

Author: James Usevitch (james.usevitch@ll.mit.edu)
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

import sys
import os
import pathlib
import shutil
from logging import error
import numpy as np
import math
import types
import pulp
from uuid import uuid4
from hashlib import sha256
import pandas as pd
from copy import deepcopy
from timeit import default_timer as timer
import traceback
from networkx import floyd_warshall_numpy
import dill as pickle
from pathos.pools import ProcessPool
from benchmark_tools import latin_dict_iterator
from utilities import sort_output_files # The "utilities.py" module is part of IITCHS benchmarking code
from generation_tools import (probability_over_capability_cardinality,
                                class_to_capability_dict,
                                generate_agents_by_total_and_distribution, 
                                generate_random_CaTL_formula_and_info)
from catl_planning.route_planning import route_planning, computeRobustnessUpperBound
from lomap.classes.ts import Ts
from catl import CATLFormula

if sys.flags.debug:
    import pdb


# # # Load TS from file

# This is a quick hack for now. TODO: Move this to if __name__ == __main__ section



# # # Configuration options

# # TS

# SECTION
# Use this section if you want to generate a random TS
ts_iterator_options = {}
ts_iterator_options['dimensions'] = [[4,4],[4,5],[5,5],[5,6]]
ts_iterator_options['state_labels'] = [['green','yellow','orange','blue','white']]
ts_iterator_options['label_counts'] = [[2,2,2,2,2]]
ts_iterator_options['edge_weight_range'] = [[1,3]]
ts_iterator_options['edge_formation_probability_list'] = [0.10]
# END SECTION


# # Agent
agent_iterator_options = {}
agent_iterator_options['capability_list'] = [['UV','Mo','Vis','IR']]
agent_iterator_options['total_num_agents'] = [5,6,7,8,9,10]
agent_iterator_options['capability_cardinality_probability'] = [[.8,.2,0,0], [.7,.3,0,0], [.6,.4,0,0]]
num_capability_classes_list = [sum([math.comb(len(cap_list),k) for k in range(1,len(cap_list)+1)]) for cap_list in agent_iterator_options['capability_list']]
agent_iterator_options['capability_class_probability_distribution'] = [[1/num_capability_classes_list[0]]*num_capability_classes_list[0]]


# # Formula
formula_iterator_options = {}
formula_iterator_options['temporal_logic_operators'] = [['F','G','FG','GF']]
formula_iterator_options['boolean_operators'] = [['AND','OR']]
formula_iterator_options['number_of_atomics_per_formula_range'] = [[1,1],[1,2],[2,2],[2,3],[3,3],[1,3]]
formula_iterator_options['operator_interval_range'] = [[0,10],[5,15],[10,20],[0,15],[0,20]]
formula_iterator_options['task_duration_range'] = [[1,5],[5,10],[1,10],[10,15],[5,15],[1,15]]
formula_iterator_options['capabilities_per_task_range'] = [[1,1],[1,2],[2,2],[2,3],[3,3],[1,3]]
formula_iterator_options['required_agents_per_capability_range'] = [[1,1],[1,2],[2,2],[2,3],[3,3],[1,3]] # Range list


# # Solver options
solver_iterator_options = {}
solver_iterator_options['solver'] = ['SCIP', 'GUROBI']
solver_iterator_options['solver_time_limit'] = [6e2]

solver_iterator_options['robust_bool'] = [True,False]
solver_iterator_options['regularize_bool'] = [True,False]
solver_iterator_options['upper_bound_bool'] = [True,False]
solver_iterator_options['alpha_value'] = [[0.5, 0.5]] # Range list


# # # Create the iterators
# # TODO: Put this in the if __name__ == "__main__" section.
# ts_iterator = latin_dict_iterator(**ts_iterator_options)
# agent_iterator = latin_dict_iterator(**agent_iterator_options)
# formula_iterator = latin_dict_iterator(**formula_iterator_options)
# solver_iterator = latin_dict_iterator(**solver_iterator_options)



# # # Functions
# # # 
# # # Each function should have the following signature:
# # #
# # #   create_XX(options, random_generator=np.random.default_rng)


# # Function that creates TS from the ts_iterator samples
def create_ts(ts_options, random_generator=np.random.default_rng()):
    "TBA"
    pass



# # Function that loads a TS. Hardcoded nasty hack.
def load_ts(input, random_generator=np.random.default_rng()):
    '''
    Loads a TS from a file.
    '''
    if isinstance(input, str):
        filename = input
    elif isinstance(input,dict):
        filename = input['filename']

    ts = Ts.load(filename)
    return ts




def create_agents(agent_options, initial_state_list, random_generator=np.random.default_rng()):
    '''
    Creates the random agents.

    Agents' initial states are chosen randomly from the initial_state list using a uniform probability distribution.

    TODO: Allow passing in deterministic initial states for the agents.
    '''
    capability_class_distr = probability_over_capability_cardinality(agent_options['capability_cardinality_probability'])
    class_to_cap_dict = class_to_capability_dict(agent_options['capability_list'])

    # Choose the initial states
    initial_states = random_generator.choice(initial_state_list, size=agent_options['total_num_agents'])

    agents = generate_agents_by_total_and_distribution(
        agent_options['total_num_agents'],
        class_to_cap_dict,
        capability_class_distr,
        initial_states,
        random_generator=random_generator
    )
    return agents


def create_formula(formula_options, random_generator=np.random.default_rng(), ts=None, agents=None):

    # If transition system ts is passed in, determine the maximum pairwise travel time between states and add this
    # to the operator intervals. This guarantees that infeasibility won't occur simply because agents couldn't get
    # to the starting tasks on time.
    pdb.set_trace() if sys.flags.debug else None
    if ts is not None:
        # Compute max pairwise travel times using Floyd Warshall algorithm
        travel_times = floyd_warshall_numpy(ts.g)

        if agents is not None:
            # Compute maximum times only from agents' initial states to any other states
            initial_states = [agent[0] for agent in agents]
            rows = [idx for idx, val in enumerate(ts.g.nodes()) if val in initial_states]
            travel_times = travel_times[rows,:]

        travel_times = travel_times.flatten().tolist()[0]
        travel_times = [time for time in travel_times if time != np.inf] # Remove infinity values
        max_travel_time = max(travel_times)

        formula_options['operator_interval_range'] = [(entry + max_travel_time) for entry in formula_options['operator_interval_range']]

    obj_formula_options = types.SimpleNamespace(**formula_options) # The generation function assumes SimpleNamespace objects

    return generate_random_CaTL_formula_and_info(obj_formula_options, random_generator=random_generator)



def create_solver(agent_options, random_generator=np.random.default_rng()):
    '''
    Creates an output dictionary with the solver options.

    The input dictionary should have the following entries:

    - robust_bool:      Single boolean value
    - regularize bool:  Single boolean value
    - upper_bound_bool: Singe boolean value
    - alpha_value:      Range in the form [a,b], a < b, to be drawn from using a uniform distribution.
    '''
    # Choose alpha
    output = deepcopy(agent_options)
    if agent_options['alpha_value'][0] == agent_options['alpha_value'][1]:
        # The range is just one number. Use it as alpha.
        output['alpha_value'] = agent_options['alpha_value'][0]
    else:
        # Choose a random value from the range, assuming a uniform distribution
        low = agent_options['alpha_value'][0]
        high = agent_options['alpha_value'][1]
        output['alpha_value'] = random_generator.uniform(low,high,1)

    return output



# Function to solve one optimization problem

def solve_one_problem(ts_options, ts_function, 
                        agent_options, agent_function, 
                        formula_options, formula_function, 
                        solver_options, solver_option_function, 
                        static_options, 
                        seed, 
                        trial_number):

    rng = np.random.default_rng(seed)

    # # Set up initial containers.
    # # Hashes a UUID for a unique name.
    trial_ID = sha256(uuid4().bytes).hexdigest()[0:10]
    m = types.ModuleType(f"dummymodule_{trial_ID}")
    m.load_old_files = False


    # # Attempt to generate a feasible problem
    positive_upper_bound = False
    for _ in range(static_options['max_generation_attempts']):

        # # Compute the TS, agents, formula, and solver options
        ts = ts_function(ts_options, random_generator=rng)

        initial_states = [node for node in ts.g.node.keys() if ('grave' not in ts.g.node[node]['prop'] and 'obs' not in ts.g.node[node]['prop'])]  # Removes the grave state and obstacles
        agents = agent_function(agent_options, initial_states, random_generator=rng)

        formula, num_atomics, num_tasks = formula_function(formula_options, random_generator=rng, ts=ts, agents=agents)

        solver_dict = solver_option_function(solver_options, random_generator=rng)


        ts_save_filename = static_options['transition_system_filename'] + trial_ID

        # # Check the upper bound on Robustness Value.
        # If the upper bound is negative, then the problem is infeasible.
        upper_bound = computeRobustnessUpperBound(ts, agents, formula)
        if upper_bound >= 0:
            positive_upper_bound = True
            break


    if positive_upper_bound:

        # Solve the optimization problem

        timed_out = False
        error_occurred = False

        try:
            start_time = timer()
            PuLP_Model, replan_data = route_planning(m, ts, agents, formula,
                file_name=ts_save_filename,
                replan_req=False,
                robust=solver_dict['robust_bool'],
                regularize=solver_dict['regularize_bool'],
                alpha=solver_dict['alpha_value'],
                upperBound=solver_dict['upper_bound_bool'],
                load_previous=False,
                solver=solver_dict['solver'],
                compute_IIS=static_options['compute_IIS'],
                verbose=static_options['verbose'],
                solver_time_limit=solver_dict['solver_time_limit'],
                solver_threads=1
            )
            runtime = timer() - start_time


            # Objective value
            if PuLP_Model.status == pulp.LpStatusOptimal:
                if PuLP_Model.objective is not None:
                    objective_value = PuLP_Model.objective.value()
                else:
                    # Objective value is None if the MILP was a feasibility problem with no objective
                    objective_value = 0

                PuLP_Status = "LpStatusOptimal"
            elif PuLP_Model.status == pulp.LpStatusNotSolved:
                objective_value = 0
                PuLP_Status = "LpStatusNotSolved"
                if hasattr(PuLP_Model, 'solutionTime'):
                    # The model timed out.
                    timed_out = True
                else:
                    # The model didn't time out, but still wasn't solved. Error.
                    error_occurred = True
            elif PuLP_Model.status == pulp.LpStatusInfeasible:
                objective_value = 0
                PuLP_Status = "LpStatusInfeasible"
            elif PuLP_Model.status == pulp.LpStatusUnbounded:
                objective_value = 0
                PuLP_Status = "LpStatusUnbounded"
            else:
                objective_value = 0
                PuLP_Status = "(unknown_status)"

            # Robustness value
            variablesDict = PuLP_Model.variablesDict()
            # Robustness value
            if "rho" in variablesDict.keys():
                rho_value = variablesDict["rho"].value()
            else:
                # See stl2milp function; if robust = False, then rho = 0.
                rho_value = 0

            # Normalized travel time
            if "normTravelTime" in variablesDict.keys():
                normalized_travel_time = variablesDict["normTravelTime"].value()
            else:
                normalized_travel_time = None

            # # Get information about the formula
            # AST bound
            ast = CATLFormula.from_formula(formula)
            ast_bound = int(ast.bound())



            # Create the output dictionary
            output = {
                'Trial_ID': trial_ID,
                'MILP_Solver_Used': solver_dict['solver'],
                'Runtime_Seconds': runtime,
                'Timed_Out': timed_out,
                'Robust': solver_dict['robust_bool'],
                'Regularize': solver_dict['regularize_bool'],
                'Alpha': solver_dict['alpha_value'],
                'Upper_Bound': solver_dict['upper_bound_bool'],
                'Grid_TS_Xdim': ts_options['dimensions'][0],
                'Grid_TS_Ydim': ts_options['dimensions'][1],
                'Number_of_Agents': agent_options['total_num_agents'],
                'Agent_List': agents,
                'Number_of_MILP_Constraints': PuLP_Model.numConstraints(),
                'Number_of_MILP_Variables': PuLP_Model.numVariables(),
                'Objective_Value': objective_value,
                'Optimizer_Final_Status': PuLP_Status,
                'Robustness_Variable_Value': rho_value,
                'Normalized_Travel_Time': normalized_travel_time,
                'TS_Filename': ts_save_filename,
                'Error_Occurred': error_occurred,
                'Formula': formula,
                'Formula_AST_Bound': ast_bound,
                'Formula_Num_Tasks': num_tasks,
                'Formula_Num_TL_Operators': num_atomics,
                'Formula_Num_Bool_Operators': num_atomics-1,
                'Max_Generation_Attempts_Hit': False
            }

            if trial_number is not None:
                output['Trial_Number'] = trial_number

        except:
            # Error occurred while solving the optimization problem.

            output = {
                'Trial_ID': trial_ID,
                'MILP_Solver_Used': solver_dict['solver'],
                'Runtime_Seconds': -1,
                'Timed_Out': False,
                'Robust': solver_dict['robust_bool'],
                'Regularize': solver_dict['regularize_bool'],
                'Alpha': solver_dict['alpha_value'],
                'Upper_Bound': solver_dict['upper_bound_bool'],
                'Grid_TS_Xdim': ts_options['dimensions'][0],
                'Grid_TS_Ydim': ts_options['dimensions'][1],
                'Number_of_Agents': agent_options['total_num_agents'],
                'Agent_List': agents,
                'Number_of_MILP_Constraints': None,
                'Number_of_MILP_Variables': None,
                'Objective_Value': None,
                'Optimizer_Final_Status': None,
                'Robustness_Variable_Value': None,
                'Normalized_Travel_Time': None,
                'TS_Filename': ts_save_filename,
                'Error_Occurred': True,
                'Formula': formula,
                'Formula_AST_Bound': ast_bound,
                'Formula_Num_Tasks': num_tasks,
                'Formula_Num_TL_Operators': num_atomics,
                'Formula_Num_Bool_Operators': num_atomics-1,
                'Max_Generation_Attempts_Hit': False
            }

            if trial_number is not None:
                output['Trial_Number'] = trial_number


    else:
        # Could not find feasible problem in max attempts.
        # If this occurs often, check the parameters you
        # are using to generate random problem instances.

        output = {
            'Trial_ID': trial_ID,
            'MILP_Solver_Used': solver_dict['solver'],
            'Runtime_Seconds': -1,
            'Timed_Out': False,
            'Robust': solver_dict['robust_bool'],
            'Regularize': solver_dict['regularize_bool'],
            'Alpha': solver_dict['alpha_value'],
            'Upper_Bound': solver_dict['upper_bound_bool'],
            'Grid_TS_Xdim': ts_options['dimensions'][0],
            'Grid_TS_Ydim': ts_options['dimensions'][1],
            'Number_of_Agents': agent_options['total_num_agents'],
            'Agent_List': agents,
            'Number_of_MILP_Constraints': None,
            'Number_of_MILP_Variables': None,
            'Objective_Value': None,
            'Optimizer_Final_Status': None,
            'Robustness_Variable_Value': None,
            'Normalized_Travel_Time': None,
            'TS_Filename': ts_save_filename,
            'Error_Occurred': True,
            'Formula': formula,
            'Formula_AST_Bound': None,
            'Formula_Num_Tasks': num_tasks,
            'Formula_Num_TL_Operators': num_atomics,
            'Formula_Num_Bool_Operators': num_atomics-1,
            'Max_Generation_Attempts_Hit': True
        }

        if trial_number is not None:
            output['Trial_Number'] = trial_number

    print(f"Finished trial {trial_number}.")

    return output
    # END solve_one_problem()



def dicts_to_dataframe(list_of_dicts):
    '''
    Takes a list of dictionaries (with the same keys) and converts them into
    a Pandas DataFrame.

    The dictionary keys become the column names of the DataFrame.
    '''
    # Assert that all dictionaries have the same keys
    for ii in range(1,len(list_of_dicts)):
        assert list_of_dicts[0].keys() == list_of_dicts[ii].keys(), f"Error: Dictionaries {0} and {ii} have different keys."

    total_dict = {}
    for key in list_of_dicts[0]:
        total_dict[key] = [current_dict[key] for current_dict in list_of_dicts]

    return pd.DataFrame.from_dict(total_dict)



def save_output_data(data, static_options):
    data.to_pickle(f"{static_options['save_parent_dir']}/DataFrame.pkl")

    # Split data into feasible, infeasible, and timed-out runs
    data_infeasible = data.loc[data['Optimizer_Final_Status'] == 'LpStatusInfeasible']
    data_feasible = data.loc[data['Optimizer_Final_Status'] == 'LpStatusOptimal']
    data_other = data.loc[~data['Optimizer_Final_Status'].isin(['LpStatusInfeasible', 'LpStatusOptimal'])]

    data_infeasible.to_pickle(f"{static_options['save_parent_dir']}/DataFrame_Infeasible.pkl")
    data_feasible.to_pickle(f"{static_options['save_parent_dir']}/DataFrame_Feasible.pkl")
    data_other.to_pickle(f"{static_options['save_parent_dir']}/DataFrame_Other.pkl")

    # Save all the iterator options

    iterator_options = {
        'ts': ts_iterator_options,
        'agent': agent_iterator_options,
        'formula': formula_iterator_options,
        'solver': solver_iterator_options
    }
    filename = f"{static_options['save_parent_dir']}/iterator_options.pkl"
    with open(filename, 'wb') as file:
        pickle.dump(iterator_options, file)

    # Sort the output files into folders for feasible, infeasible, and other.
    # The sort_output_files function is in the "utilities.py" module.
    sort_output_files(static_options['save_parent_dir'])




def preprocess_filepath(path):
    if path[0] == '.':
        # Convert relative path to absolute path
        return os.path.abspath(path)
    else:
        return path



# Set up the static options
static_options = {
    'save_data_bool': False,
    'transition_system_dir': './output/last_run',
    'transition_system_filename': './output/last_run/TS',
    'save_parent_dir': './output/last_run',
    'save_filename': './output/last_run',
    'max_generation_attempts': 10,
    'compute_IIS': False,
    'verbose': False,
}

    


if __name__ == "__main__":
    

    if '--serial' in sys.argv:
        # Turn off parallel processing
        parallel = False
    else:
        parallel = True

    total_trials = 10000

    num_nodes = 60
    max_batch_size = num_nodes

    pool = ProcessPool(nodes=num_nodes)

    trial_numbers = range(1,total_trials+1)

    # Set up the seeds for random number generation.
    # This method ensures that each parallel process has a random number
    # generator independent of the other processes.
    # See here for details: https://numpy.org/doc/stable/reference/random/parallel.html
    seed_sequence = np.random.SeedSequence()
    seed_list = seed_sequence.spawn(total_trials)
    


    # # Create the iterators
    # TODO: Put this in the if __name__ == "__main__" section.
    ts_iterator = latin_dict_iterator(**ts_iterator_options, repetitions=total_trials)
    agent_iterator = latin_dict_iterator(**agent_iterator_options, repetitions=total_trials)
    formula_iterator = latin_dict_iterator(**formula_iterator_options, repetitions=total_trials)
    solver_iterator = latin_dict_iterator(**solver_iterator_options, repetitions=total_trials)




    ts_filename = preprocess_filepath(sys.argv[1]) # May need to change this
    print(f'Opening TS file {ts_filename}')

    # !! Hardcoded information here !!
    # Changes ts_iterator_options to match the transition system

    ts = Ts.load(ts_filename)
    ts_loaded_labels = list(set.union(*[d['prop'] for _, d  in ts.g.nodes(data=True)]))
    ts_loaded_labels = [label for label in ts_loaded_labels if label not in ['grave','obs', 'empty']]


    # List to store results in
    results = []

    # Print information
    print(f"Total trials: {total_trials}")

    if parallel:
        # Solve the optimization problems in parallel.
        #    
        # Note: Either pool.imap or pool.uimap can be used.
        #           * imap is non-blocking, but guarantees preservation of order
        #           * uimap is non-blocking and doesn't preserve order.
        #       Order doesn't matter for the results, so uimap is fine.
        trials_remaining = total_trials
        trial_idx_start = 0
        try:
            # Batch the results so the memory usage doesn't blow up


            while trials_remaining > 0:
                next_batch_size = min(trials_remaining, max_batch_size)

                next_trial_numbers = range(trial_idx_start,(trial_idx_start + next_batch_size))
                batch_seed_list = seed_list[trial_idx_start:(trial_idx_start + next_batch_size)]

                ts_inputs = [next(ts_iterator) for _ in range(next_batch_size)]
                for ts_dict in ts_inputs:
                    ts_dict['filename'] = ts_filename # Loads specific TS

                agent_inputs = [next(agent_iterator) for _ in range(next_batch_size)]

                formula_inputs = [next(formula_iterator) for _ in range(next_batch_size)]
                for formula_dict, agent_dict in zip(formula_inputs, agent_inputs):
                    formula_dict['capability_list'] = agent_dict['capability_list']
                    formula_dict['state_labels'] = ts_loaded_labels

                solver_inputs = [next(solver_iterator) for _ in range(next_batch_size)]

                # Inline function to make the call to pool.uimap more elegant
                def run_function(ts_inputs, agent_inputs, formula_inputs, solver_inputs, seed, trial_number):
                    return solve_one_problem(ts_inputs, load_ts,
                                                agent_inputs, create_agents,
                                                formula_inputs, create_formula,
                                                solver_inputs, create_solver,
                                                static_options,
                                                seed,
                                                trial_number)

                batch_results = pool.uimap(run_function,
                                            ts_inputs,
                                            agent_inputs,
                                            formula_inputs,
                                            solver_inputs,
                                            batch_seed_list,
                                            next_trial_numbers
                )
                results = results + list(batch_results)

                # Clean up memory
                pool.terminate()
                pool.restart()


                trials_remaining -= next_batch_size
                trial_idx_start += next_batch_size

            # Create final DataFrame
            data = dicts_to_dataframe(results)

            # Save the data
            save_output_data(data, static_options)

        except Exception as e:
            pool.terminate()
            print(traceback.format_exc())

    else:
        # Run experiments NOT in parallel
        # Mostly for testing

        for ii in range(total_trials):
            # Solve the MILPs
            trial_number = ii+1
            seed = seed_list[ii]
            
            ts_inputs = next(ts_iterator)
            ts_inputs['filename'] = ts_filename # Loads specific TS

            agent_inputs = next(agent_iterator)

            formula_inputs = next(formula_iterator)
            formula_inputs['capability_list'] = agent_inputs['capability_list']
            formula_inputs['state_labels'] = ts_loaded_labels
            
            solver_inputs = next(solver_iterator)
            
            next_result = solve_one_problem(
                    ts_inputs, load_ts,
                    agent_inputs, create_agents,
                    formula_inputs, create_formula,
                    solver_inputs, create_solver,
                    static_options,
                    seed,
                    trial_number
                )

            results += [next_result]

        # Create final DataFrame
        data = dicts_to_dataframe(results)

        # Save the data
        save_output_data(data, static_options)


