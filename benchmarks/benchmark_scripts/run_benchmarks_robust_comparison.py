'''
Generates random problem instances and benchmarks their solution time.

A random TS is chosen, and this TS is kept constant.

Data is saved as a Pandas Dataframe.

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


from generation_tools import probability_over_capability_cardinality, class_to_capability_dict, generate_agents_by_total_and_distribution
from RandomizedCaTLExperiments import generateRandomGridTS_seed, generateRandomAgents_seed
from catl_planning.route_planning import route_planning
import numpy as np
from timeit import default_timer as timer
import pandas as pd
from benchmark_tools import BenchmarkOptionsGenerator, solve_problem_robust_and_not_robust
import pulp
from collections import namedtuple
import sys
sys.path.append('../src')
import traceback
import dill as pickle

from benchmark_tools import OutputTuple, data_column_names, solve_one_optimization_problem

# For parallel processing
# The pathos module is used because
#   * Python's multiprocessing module uses pickle
#   * pickle doesn't work on a lot of objects (e.g. lambda functions, namedtuples)
#   * pathos uses the dill module instead which fixes these problems.
from pathos.pools import ProcessPool

# For debugging
if sys.flags.debug:
    import pdb


# # # Functions for determining multiplicity of state labels

def multiplicity_function(dimension):
    '''
    Returns 1/2 of the square root of the total number of states,
    rounded up. (This formula was chosen in a completely arbitrary manner.)
    '''
    return int(np.ceil(0.5*np.sqrt(dimension[0]*dimension[1])))



if __name__ == "__main__":

    # # # Set up a BenchmarkOptionsGenerator object

    og = BenchmarkOptionsGenerator()

    # # Transition System Options

    og.dimensions_list = [[4,4], [4,5], [5, 5], [5,6]]
    og.state_labels_list = [['green', 'yellow', 'orange', 'blue', 'white']]
    # og.state_label_probabilities_list = [[0.05, 0.05, 0.05, 0.05, 0.8]]
    og.state_label_probabilities_list = [[0.2,0.2,0.2,0.2,0.2]]

    # The multiplicity functions are defined above.
    og.state_label_multiplicity_function_list = [multiplicity_function]

    # TODO: Put this in the output DataFrame!!!
    og.edge_weight_range_list = [[1,3], [3,5],[5,7]]
    og.edge_formation_probability_list = [0.1, 0.2, 0.3]


    # # Agent and Capability Options

    og.capability_list = [['UV','Mo','Vis','IR']]

    og.total_num_agents_list = [10,15,20]
    # List of capability cardinality distributions (e.g. how likely it is
    #   for each agent to have 1, 2, 3, or 4 capabilities).
    #
    # See the function probability_over_capability_cardinality in generation_tools.py
    cap_cardinality_distr_list = [
        [.5, .4, .08, .02]
    ]
    og.capability_class_probability_distribution_list = [probability_over_capability_cardinality(distr) for distr in cap_cardinality_distr_list]
    pdb.set_trace() if sys.flags.debug else None

    # # Temporal Logic Options

    og.number_of_atomics_per_formula_range_list = [[1,1], [2,2], [3,3],[1,2],[2,3],[1,3]]
    og.operator_interval_range_list = [[0,10], [10,20], [20,30]]

    og.task_duration_range_list = [[0,5], [5,10], [10,15]]
    og.capabilities_per_task_range_list = [[1,1],[2,2],[3,3],[1,2],[2,3],[1,3]]
    og.required_agents_per_capability_range_list = [[1,1],[2,2],[3,3],[1,2],[2,3],[1,3]]

    # # Solver Options
    og.solvers_list = ['SCIP', 'GUROBI'] 
    og.robust_values_list = [True] # Needs exactly one entry for this particular script, but the value is unused.
    og.regularize_values_list = [True,False]
    og.upper_bound_boolean_list = [True,False]

    # DEBUG
    og.solver_time_limit_list = [6e2]
    # END DEBUG

    og.number_of_trials_per_combination = 4

    # # # Run the optimization.

    repetitions = 1 # Number of times we repeat the Latin Hypercube sampling routine

    # Iterator that gives option combinations for each simulation run.
    og_itr = og.create_latin_iterator(repetitions=repetitions)

    # run_function = functools.partial(run_one_optimization_problem, static_options=og.static_options, transition_system=ts)
    run_function = lambda options, trial_number, seed: solve_problem_robust_and_not_robust(options, trial_number=trial_number, static_options=og.static_options, seed=seed)

    num_nodes = 60
    # pool = ProcessPool(nodes=4)
    pool = ProcessPool(nodes=num_nodes)

    # input_options = [next(og_itr) for k in range(1)]
    input_options = og_itr
    trial_numbers = range(1, len(input_options)+1)

    # # Set up the seeds for random number generation.
    # # This method ensures that each parallel process has a random number
    # # generator independent of the other processes.
    # # See here for details: https://numpy.org/doc/stable/reference/random/parallel.html
    # seed_sequence = np.random.SeedSequence(hash(time.time())) # Not sure whether this is more / less effective than default (no arguments)
    seed_sequence = np.random.SeedSequence()
    seed_list = seed_sequence.spawn(len(input_options))



    pdb.set_trace() if sys.flags.debug else None
    # Note: Either pool.imap or pool.uimap can be used.
    #           * imap is non-blocking, but guarantees preservation of order
    #           * uimap is non-blocking and doesn't preserve order.
    #       Order doesn't matter for the results, so uimap is fine.
    try:

        # Batch the results so the memory usage doesn't blow up

        total_trials = len(og_itr)
        print(f"Total trials: {total_trials}")
        iter = 0

        batch_size = num_nodes

        results = []

        while len(og_itr) > 0:
            if len(og_itr) >= batch_size:
                trial_numbers = range(iter*batch_size, (iter+1)*batch_size)
                input_options = [next(og_itr) for k in range(batch_size)]
                batch_seed_list = seed_list[iter*batch_size:(iter+1)*batch_size]
            else:
                num_remaining = len(og_itr)
                trial_numbers = range(iter*batch_size, iter*batch_size+len(og_itr))
                input_options = [next(og_itr) for k in range(len(og_itr))]
                batch_seed_list = seed_list[iter*batch_size:iter*batch_size+num_remaining]
            

            pdb.set_trace() if sys.flags.debug else None

            batch_results = pool.uimap(run_function, input_options, trial_numbers, batch_seed_list)
            results = results + list(batch_results)

            iter += 1

            # Clean up memory
            pool.terminate()
            pool.restart()


        # Count the number of "None" entries -- these were 

        # Flatten the list--remove None values, expand sublists
        results = [item for sublist in results for item in sublist]


    

        # Save the data.
        data = pd.DataFrame(columns=data_column_names)

        for result in results:
            if result.Robust == True:
                row_name = f'{result.Trial_Number}_T'
            else:
                row_name = f'{result.Trial_Number}_F'

            data = data.append(pd.Series(name=row_name, dtype=object))
            for field in result._fields:
                data[field][row_name] = getattr(result, field)

        print(data)
        data.to_pickle(f"{og.static_options.save_parent_dir}/DataFrame.pkl")

        # Split the data into feasible and infeasible runs
        data_infeasible = data.loc[data['Optimizer_Final_Status'] == 'LpStatusInfeasible']
        data_feasible = data.loc[data['Optimizer_Final_Status'] == 'LpStatusOptimal']
        data_other = data.loc[~data['Optimizer_Final_Status'].isin(['LpStatusInfeasible', 'LpStatusOptimal'])]

        data_infeasible.to_pickle(f"{og.static_options.save_parent_dir}/DataFrame_Infeasible.pkl")
        data_feasible.to_pickle(f"{og.static_options.save_parent_dir}/DataFrame_Feasible.pkl")
        data_other.to_pickle(f"{og.static_options.save_parent_dir}/DataFrame_Other.pkl")

        # Save the configuration options
        with open(f"{og.static_options.save_parent_dir}/BenchmarkOptionsGenerator.pkl",'wb') as file:
            pickle.dump(og,file)

        # Close the pool.
        pool.terminate()
    except Exception as e:
        pool.terminate()
        print(traceback.format_exc())


