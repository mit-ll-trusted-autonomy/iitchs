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


from RandomizedCaTLExperiments import generateRandomGridTS_seed, generateRandomAgents_seed
from catl_planning.route_planning import route_planning
import numpy as np
from timeit import default_timer as timer
import types
import copy
import pandas as pd
from benchmark_tools import BenchmarkOptionsGenerator
import pulp
from collections import namedtuple
import sys
sys.path.append('../src')
import time

from benchmark_tools import OutputTuple, data_column_names, solve_one_optimization_problem

# For parallel processing
# The pathos module is used because
#   * Python's multiprocessing module uses pickle
#   * pickle doesn't work on a lot of objects (e.g. lambda functions, namedtuples)
#   * pathos uses the dill module instead which fixes these problems.
import functools
from pathos.pools import ProcessPool


# Deprecate this?


# For debugging
if sys.flags.debug:
    import pdb


# -------------------------------------------------------------------------------------------------------------
# TODO: Remove this section below and put in its own file?



if __name__ == "__main__":

    # # # Set up a BenchmarkOptionsGenerator object

    og = BenchmarkOptionsGenerator()

    # # Transition System Options

    og.dimensions_list = [[5, 5]]
    og.state_labels_list = [['green', 'yellow', 'orange', 'blue', 'white']]
    # og.state_label_probabilities_list = [[0.05, 0.05, 0.05, 0.05, 0.8]]
    og.state_label_probabilities_list = [[0.2,0.2,0.2,0.2,0.2]]

    # # Agent and Capability Options

    # # Temporal Logic Options

    # # Solver Options
    og.solvers_list = ['SCIP', 'GUROBI']
    og.robust_values_list = [True,False]
    og.regularize_values_list = [True,False]
    og.upper_bound_boolean_list = [True,False]

    og.number_of_trials_per_combination = 4

    # # Generate a random TS

    # The number of times that the generation algorithm loops over the labels
    # list and sequentially applies the state labels to random states.
    # A little complicated to explain. Set to 1 if you don't know what you're
    # doing.
    state_label_multiplicity = 5
    seed = np.random.randint(10000)
    ts = generateRandomGridTS_seed(og.static_options.transition_system_filename, og.dimensions_list[0], og.state_labels_list[0], og.edge_weight_range_list[0], state_label_multiplicity, seed)

    # # # Run the optimization.

    # Iterator that gives option combinations for each simulation run.
    og_itr = og.create_iterator()

    # run_function = functools.partial(run_one_optimization_problem, static_options=og.static_options, transition_system=ts)
    run_function = lambda options, trial_number, seed: solve_one_optimization_problem(options, trial_number=trial_number, static_options=og.static_options, transition_system=ts, seed=seed)

    pool = ProcessPool(nodes=4)
    # pool = ProcessPool(nodes=60)

    input_options = [next(og_itr) for k in range(4)]
    # input_options = og_itr
    trial_numbers = range(1, len(input_options)+1)

    # # Set up the seeds for random number generation.
    # # This method ensures that each parallel process has a random number
    # # generator independent of the other processes.
    # # See here for details: https://numpy.org/doc/stable/reference/random/parallel.html
    # seed_sequence = np.random.SeedSequence(hash(time.time())) # Not sure whether this is more / less effective than default (no arguments)
    seed_sequence = np.random.SeedSequence()
    seed_list = seed_sequence.spawn(len(input_options))


    # Note: Either pool.imap or pool.uimap can be used.
    #           * imap is non-blocking, but guarantees preservation of order
    #           * uimap is non-blocking and doesn't preserve order.
    #       Order doesn't matter for the results, so uimap is fine.
    try:
        results = pool.uimap(run_function, input_options, trial_numbers, seed_list)

        results = list(results)

        print(results)
    

        # Save the data.
        data = pd.DataFrame(columns=data_column_names)

        for result in results:
            data = data.append(pd.Series(name=result.Trial_Number, dtype=object))
            for field in result._fields:
                data[field][result.Trial_Number] = getattr(result, field)

        print(data)
        data.to_pickle(f"{og.static_options.save_parent_dir}/DataFrame.pkl")

        # Split the data into feasible and infeasible runs
        data_infeasible = data.loc[data['Optimizer_Final_Status'] == 'LpStatusInfeasible']
        data_feasible = data.loc[data['Optimizer_Final_Status'] == 'LpStatusOptimal']
        data_other = data.loc[~data['Optimizer_Final_Status'].isin(['LpStatusInfeasible', 'LpStatusOptimal'])]

        data_infeasible.to_pickle(f"{og.static_options.save_parent_dir}/DataFrame_Infeasible.pkl")
        data_feasible.to_pickle(f"{og.static_options.save_parent_dir}/DataFrame_Feasible.pkl")
        data_other.to_pickle(f"{og.static_options.save_parent_dir}/DataFrame_Other.pkl")

        # Close the pool.
        pool.terminate()
    except Exception as e:
        pool.terminate()
        print(repr(e))



    # output = run_one_optimization_problem(option_tuple, og.static_options, transition_system=ts)
#
    # trial_number=1
#
    # data = pd.DataFrame(columns=data_column_names)

    # data = data.append(pd.Series(name=trial_number, dtype=object))
#
    # for field in output._fields:
        # data[field][trial_number] = getattr(output, field)
#
    # pdb.set_trace() if sys.flags.debug else None
            #
    # print(data)
    # data.to_pickle(f"{og.static_options.save_parent_dir}/DataFrame.pkl")
