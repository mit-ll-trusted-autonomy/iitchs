'''
Module containing tools for benchmarking IITCHS MILP problem performance.

The module contains the following classes:

    * BenchmarkOptionsGenerator: Class for storing configurations options
        for benchmarking.
    * named_tuple_iterator: Similar to itertools.product, but returns an
        iterator that yields named tuples. Also maintains an internal
        count of how many items it has left.
    * sized_product_iter: Similar to itertools.product, but maintains an
        internal count of how many items it has left.

Author: James Usevitch (james.usevitch@ll.mit.edu)
Created: Aug 2021
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
import numpy as np
import pulp
from typing import List
import types
from types import SimpleNamespace
from itertools import product, starmap
from collections import namedtuple
from timeit import default_timer as timer
from catl import CATLFormula
from math import gcd
from time import time
import math

from generation_tools import class_to_capability_dict, generate_agents_by_class, generate_agents_by_total_and_distribution, generate_random_CaTL_formula_and_info, generateRandomGridTS
# from RandomizedCaTLExperiments import generateRandomGridTS_seed, generateRandomAgents_seed
from catl_planning.route_planning import route_planning, generate_MILP_problems, computeRobustnessUpperBound


from lomap import Ts

# For debugging purposes
import sys
if sys.flags.debug:
    import pdb




# # # Benchmarking objects and structures


class sized_product_iter():
    '''
    Similar to itertools.product, but maintains an internal count of the objects it has left.

    Works with next() and len().
    '''
    def __init__(self, *args, repeat=1):
        args2 = [list(arg) for arg in args]
        self._items_remaining = len(args2[0])
        for ii in range(1,len(args2)):
            self._items_remaining *= len(args2[ii])

        self._iter = product(*args2, repeat=repeat)


    def __iter__(self):
        return self


    def __next__(self):
        self._items_remaining -= 1
        return next(self._iter)


    def __len__(self):
        return self._items_remaining


class named_tuple_iterator():
    '''
    Similar to sized_product_iter, but returns the items as named tuples.

    Note: Does not take the "repeat" keyword argument used in itertools.product.
    '''

    def __init__(self, **kwargs):
        
        # Expand iterables
        kwargs_expanded = {key: list(value) for (key, value) in zip(kwargs.keys(), kwargs.values())}

        # Calculate length
        value_list = list(kwargs_expanded.values())
        self._items_remaining = len(value_list[0])
        for ii in range(1,len(value_list)):
            self._items_remaining *= len(value_list[ii])

        assert self._items_remaining > 0, "Length of items remaining is zero. Check your input lists / iterators."

        ProductTuple = namedtuple('ProductTuple', kwargs_expanded.keys())
        self._iter = starmap(ProductTuple, product(*kwargs_expanded.values()))


    def __iter__(self):
        return self


    def __next__(self):
        if self._items_remaining > 0:
            self._items_remaining -= 1
        return next(self._iter)


    def __len__(self):
        return self._items_remaining



class latin_named_tuple_iterator():
    '''
    Similar to named_tuple_iterator, but generates latin hypercube samples to reduce
    the amount of testing required.

    Method for Sampling:

        First, all input iterators are turned into lists. The number of input iterators
        defines the number of "variables" in the sample.

        The least common multiple (LCM) of all list lengths defines the dimension of the Latin
        hypercube samples; i.e. the number of discrete "bins" to be sampled from.

        A 2D Numpy array is formed in memory. Each column of the array contains the "raw"
        coordinates for one Latin hypercube sample routine. ("Raw" means that the coordinates are
        based on the LCM of all list lengths). When ``repetition`` > 1, multiple Latin hypercube
        routine results are concatenated horizontally into one matrix, with the number of routines
        equal to ``repetition``.

        When next() is called on the iterator, the raw coordinates for a hypercube sample are
        matched to the corresponding bins in the stored lists. A named tuple is returned 
        containing the entries from these bins. 
    '''

    def __init__(self, random_generator=None, repetitions=1, **kwargs):
        '''
        Args:
            random_generator:   For a numpy random generator to be passed in.

            repetitions (int):  The number of times to repeat the Latin hypercube.
                                For example, setting ``repetitions=2`` will give
                                an iterator that iterates over 2 full Latin
                                Hypercube sampling routines.
        '''

        if random_generator is None:
            rng = np.random.default_rng()
        else:
            rng = random_generator
        
        # Expand iterables
        self._kwargs_expanded = {key: list(value) for (key, value) in zip(kwargs.keys(), kwargs.values())}

        # Calculate number of iterables
        self.num_variables = len(self._kwargs_expanded)

        # Calculate total number of samples needed.
        # Uses the least common multiple of all the list lengths.
        # self._value_list = list(self._kwargs_expanded.values())
        self._key_list = list(self._kwargs_expanded.keys())
        self._list_lengths = [len(self._kwargs_expanded[key]) for key in self._key_list]
        total_samples = lcm(self._list_lengths)
        self._total_samples = total_samples


        self.ProductTuple = namedtuple('ProductTuple', self._kwargs_expanded.keys())

        # Concatenate multiple Latin hypercubes together if repetitions > 1.
        # Each repetition is a complete Latin hypercube sample.
        # Each column of self._idx_matrix contains the raw indices for a sampling of the variables.
        self._idx_matrix = np.hstack([latin_hypercube_sample(self.num_variables, self._total_samples, random_generator=rng) for jj in range(repetitions)])

        self._items_remaining = total_samples*repetitions
        assert self._items_remaining > 0, "Length of items remaining is zero. Check your input lists / iterators."
        assert self._items_remaining == self._idx_matrix.shape[1]



        # self._iter = starmap(ProductTuple, product(*kwargs_expanded.values()))


    def __iter__(self):
        return self


    def __next__(self):
        if self._items_remaining > 0:

            attr_dict = {}

            col_idx = self._total_samples - self._items_remaining
            for ii in range(self.num_variables):
                # Convert the raw hypercube index based on the LCM of all list lenghts
                # to the actual index of the list
                raw_idx = self._idx_matrix[ii,col_idx]
                list_len = self._list_lengths[ii]
                bin_size = self._total_samples // list_len
                idx = raw_idx // bin_size
                variable_name = self._key_list[ii]
                variable_value = self._kwargs_expanded[variable_name][idx]
                attr_dict[variable_name] = variable_value

            outtuple = self.ProductTuple(**attr_dict)

            self._items_remaining -= 1
            return outtuple
        else:
            return None


    def __len__(self):
        return self._items_remaining




class latin_dict_iterator():
    '''
    Creates an iterator producing Latin Hypercube samples from the given list of options. Outputs
    are in the form of dictionaries.

    Method for Sampling:

        First, all input iterators are turned into lists. The number of input iterators
        defines the number of "variables" in the sample.

        The least common multiple (LCM) of all list lengths defines the dimension of the Latin
        hypercube samples; i.e. the number of discrete "bins" to be sampled from.

        A 2D Numpy array is formed in memory. Each column of the array contains the "raw"
        coordinates for one Latin hypercube sample routine. ("Raw" means that the coordinates are
        based on the LCM of all list lengths). When ``repetition`` > 1, multiple Latin hypercube
        routine results are concatenated horizontally into one matrix, with the number of routines
        equal to ``repetition``.

        When next() is called on the iterator, the raw coordinates for a hypercube sample are
        matched to the corresponding bins in the stored lists. A named tuple is returned 
        containing the entries from these bins. 
    '''

    def __init__(self, random_generator=None, repetitions=1, **kwargs):
        '''
        Args:
            random_generator:   For a numpy random generator to be passed in.

            repetitions (int):  The number of times to repeat the Latin hypercube.
                                For example, setting ``repetitions=2`` will give
                                an iterator that iterates over 2 full Latin
                                Hypercube sampling routines.
        '''

        if random_generator is None:
            rng = np.random.default_rng()
        else:
            rng = random_generator
        
        # Expand iterables
        self._kwargs_expanded = {key: list(value) for (key, value) in zip(kwargs.keys(), kwargs.values())}

        # Calculate number of iterables
        self.num_variables = len(self._kwargs_expanded)

        # Calculate total number of samples needed.
        # Uses the least common multiple of all the list lengths.
        # self._value_list = list(self._kwargs_expanded.values())
        self._key_list = list(self._kwargs_expanded.keys())
        self._list_lengths = [len(self._kwargs_expanded[key]) for key in self._key_list]
        total_samples = lcm(self._list_lengths)
        self._total_samples = total_samples

        # Concatenate multiple Latin hypercubes together if repetitions > 1.
        # Each repetition is a complete Latin hypercube sample.
        # Each column of self._idx_matrix contains the raw indices for a sampling of the variables.
        self._idx_matrix = np.hstack([latin_hypercube_sample(self.num_variables, self._total_samples, random_generator=rng) for jj in range(repetitions)])

        self._items_remaining = total_samples*repetitions
        assert self._items_remaining > 0, "Length of items remaining is zero. Check your input lists / iterators."
        assert self._items_remaining == self._idx_matrix.shape[1]



        # self._iter = starmap(ProductTuple, product(*kwargs_expanded.values()))


    def __iter__(self):
        return self


    def __next__(self):
        if self._items_remaining > 0:

            attr_dict = {}

            col_idx = self._total_samples - self._items_remaining
            for ii in range(self.num_variables):
                # Convert the raw hypercube index based on the LCM of all list lenghts
                # to the actual index of the list
                raw_idx = self._idx_matrix[ii,col_idx]
                list_len = self._list_lengths[ii]
                bin_size = self._total_samples // list_len
                idx = raw_idx // bin_size
                variable_name = self._key_list[ii]
                variable_value = self._kwargs_expanded[variable_name][idx]
                attr_dict[variable_name] = variable_value

            self._items_remaining -= 1
            return attr_dict
        else:
            return None


    def __len__(self):
        return self._items_remaining





# # # Optimization File Generation Tools

class Latin_MILP_File_Generator():
    '''
    Object that generates IITCHS MILP .lp / .mps problem files using
    a BenchmarkOptionsGenerator and Latin Hypercube Sampling. 

    The MILP problem is NOT solved. The output location of the 
    files can be controlled by setting the 
    .static_options.transition_system_filename variable for the
    input BenchmarkOptionsGenerator.
    '''

    def __init__(self, BenchmarkOptionsGenerator, seed=None):
        if seed is None:
            self._rng = np.random.default_rng()
        else:
            self._rng = np.random.default_rng(seed)

        self._BOG = BenchmarkOptionsGenerator

        self._iterator = self._BOG.create_latin_iterator(random_generator=self._rng)


    
    def __iter__(self):
        return self


    
    def __next__(self):
        if len(self._iterator) > 0:
            options_tuple = next(self._iterator)
        else:
            # Create new Latin iterator
            self._iterator = self._BOG.create_latin_iterator(random_generator=self._rng)
            options_tuple = next(self._iterator)
        
        
        # # Set up the problem instance
        # # For all values that are ranges [min,max], choose a random value within the range.

        static_options = self._BOG.static_options

        # Generates unique filename for each TS
        ts_save_filename = static_options.transition_system_filename + str(hash(str(options_tuple) + str(time())))

        # TODO: Make it possible to use other TS generation functions
        ts_name = ts_save_filename
        dimensions = options_tuple.dimensions
        multiplicity = options_tuple.state_label_multiplicity_function(dimensions)
        ts = generateRandomGridTS(ts_name, dimensions, options_tuple.state_labels, options_tuple.edge_weight_range, multiplicity, random_generator=self._rng)


        # Agents
        # TODO: Implement more general method for choosing random agents
        states = ts.g.nodes()
        agents = generate_agents_by_class(states, options_tuple.capability_list, 1, random_generator=self._rng)

        # Formula
        formula, num_atomics, num_tasks = generate_random_CaTL_formula_and_info(options_tuple, random_generator=self._rng)

        # Alpha
        if options_tuple.alpha_value_range[0] == options_tuple.alpha_value_range[1]:
            alpha = options_tuple.alpha_value_range[0]
        else:
            low = options_tuple.alpha_value_range[0]
            high = options_tuple.alpha_value_range[1]
            alpha = self._rng.uniform(low=low, high=high)

        lp_name, mps_name = generate_MILP_problems(ts, agents, formula,
                                    bound=None,
                                    file_name=ts_save_filename,
                                    robust=True,
                                    regularize=options_tuple.regularize_value,
                                    alpha=alpha,
                                    upperBound=options_tuple.upper_bound_boolean,
                                    replan_grave=None,
                                    verbose=False,
                                    compress_files=True)

        return lp_name, mps_name



class Latin_MILP_File_Generator():
    '''
    Object that generates IITCHS MILP .lp / .mps problem files using
    a BenchmarkOptionsGenerator and Latin Hypercube Sampling. 

    The MILP problem is NOT solved. The output location of the 
    files can be controlled by setting the 
    .static_options.transition_system_filename variable for the
    input BenchmarkOptionsGenerator.
    '''

    def __init__(self, BenchmarkOptionsGenerator, seed=None):
        if seed is None:
            self._rng = np.random.default_rng()
        else:
            self._rng = np.random.default_rng(seed)

        self._BOG = BenchmarkOptionsGenerator

        self._iterator = self._BOG.create_latin_iterator(random_generator=self._rng)


    
    def __iter__(self):
        return self


    
    def __next__(self):
        if len(self._iterator) > 0:
            options_tuple = next(self._iterator)
        else:
            # Create new Latin iterator
            self._iterator = self._BOG.create_latin_iterator(random_generator=self._rng)
            options_tuple = next(self._iterator)
        
        
        # # Set up the problem instance
        # # For all values that are ranges [min,max], choose a random value within the range.

        static_options = self._BOG.static_options

        # Generates unique filename for each TS
        ts_save_filename = static_options.transition_system_filename + str(hash(str(options_tuple) + str(time())))

        # TODO: Make it possible to use other TS generation functions
        ts_name = ts_save_filename
        dimensions = options_tuple.dimensions
        multiplicity = options_tuple.state_label_multiplicity_function(dimensions)
        ts = generateRandomGridTS(ts_name, dimensions, options_tuple.state_labels, options_tuple.edge_weight_range, multiplicity, random_generator=self._rng)


        # Agents
        # TODO: Implement more general method for choosing random agents
        states = ts.g.nodes()
        agents = generate_agents_by_class(states, options_tuple.capability_list, 1, random_generator=self._rng)

        # Formula
        formula, num_atomics, num_tasks = generate_random_CaTL_formula_and_info(options_tuple, random_generator=self._rng)

        # Alpha
        if options_tuple.alpha_value_range[0] == options_tuple.alpha_value_range[1]:
            alpha = options_tuple.alpha_value_range[0]
        else:
            low = options_tuple.alpha_value_range[0]
            high = options_tuple.alpha_value_range[1]
            alpha = self._rng.uniform(low=low, high=high)

        lp_name, mps_name = generate_MILP_problems(ts, agents, formula,
                                    bound=None,
                                    file_name=ts_save_filename,
                                    robust=True,
                                    regularize=options_tuple.regularize_value,
                                    alpha=alpha,
                                    upperBound=options_tuple.upper_bound_boolean,
                                    replan_grave=None,
                                    verbose=False,
                                    compress_files=True)

        return lp_name, mps_name




# # # Problem Option Classes

# # # Problem Option Classes

class BenchmarkOptionsGenerator():
    '''
    Class used for setting up problem instance generation for IITCHS benchmarking.

    All options are in the form of lists, e.g.

        [configuration_1, configuration_2]

    Each configuration may be an object or another list.

    The function [INSERT HERE] produces an iterable that generates tuples of all possible
    combinations of configurations. These tuples can be passed to benchmarking functions
    to set up and run the optimization problems in parallel.
    
    For example, given two options

        option1, option2,

    each having configurations

        option1: [[config_1_1], [config_1_2]]
        option2: [[config_2_1], [config_2_2]],

    the iterator returns tuples of the form

        ([config_1_1], [config_2_1]), ([config_1_1], [config_2_2]), ([config_1_2],[config_2_1]), ([config_1_2], [config_2_2])

    Generating the actual problem instances from, e.g., configuration ranges for a particular
    value is left up to	the benchmarking functions running in parallel (? TBD)

    Note:
        If you only want one configuration to be tested for a particular option, simply make the (outer) list have length one, e.g.

            option_1 = [configuration]

    Conventions:

        * Anything appended with "_range_list" is in the form [[min, max],...]. It is meant to present
            a list of ranges from which samples can be drawn for a configuration value.
    '''

    def __init__(self):
        '''
        Note:

            Initialization values have been provided here which should give you somewhat sensible defaults for testing basic
            IITCHS scenarios. These fields will need to be updated manually in your code to match the system you are 
            considering.
        '''

        # # Transition System Options

        # States
        self.dimensions_list = [[4,4],[4,5],[5,5],[5,6]] # TODO: See if we can use regular TS generation instead of grid TS generation.
        #self.number_of_states_list = [[]] # For non-grid TS; not implemented yet

        # Labels
        self.state_labels_list = [['green','yellow','orange','blue','white']]
        self.state_label_probabilities_list = [[0.05,0.05,0.05,0.05,0.8]]
        # The integers in state_label_multiplicities_list determine how many times the
        # states are looped over when applying labels to states. For example, if the
        # multiplicity is 2, the state list is iterated over twice, with each state
        # receiving one random label on each iteration.
        #
        # In general, you want the multiplicity integer to grow as the
        # total number of states grows.
        #
        # Use the variable state_label_multiplicity_function_list to define a 
        # series of _functions_ that map dimensions from dimensions_list into
        # corresponding multiplicity integers. Multiple functions can be 
        # put into the list, and will be iterated over in the option combinations.
        def constant_multiplicity(dimension):
            '''
            Maps any entered dimension to a multiplicity of 1.

            This is an example function. You should define custom
            functions in your code and store them in state_label_multiplicity_function_list.
            '''
            return 1

        def state_dimension_to_multiplicity(dimension):
            '''
            Maps the dimension to an integer multiplicity of labels.

            This is an example function. You should define custom functions
            in your code and store them in state_label_multiplicity_function_list.
            '''
            if dimension[0] == 4:
                return 2
            else:
                return 3

        self.state_label_multiplicity_function_list = [constant_multiplicity, state_dimension_to_multiplicity]

        # States and Edges
        self.edge_weight_range_list = [[1,3]]
        self.edge_formation_probability_list = [0.10]


        # # Agent and Capability Options

        # Agent options
        self.capability_list = [['UV','Mo','Vis','IR']]
        self.num_capability_classes = sum([math.comb(len(self.capability_list),k) for k in range(1,len(self.capability_list)+1)]) # Number of capability classes, i.e. number of unique combinations of one or more capabilities. For example, {UV}, {Mo, IR}, {UV, Mo, Vis}, etc.

        # Currently, agent generation is performed using the generate_agents_by_total_and_distribution() function.
        self.total_num_agents_list = [5, 10, 15] # List of different total number of agents to consider in simulations. Agents will be assigned capabilities based on the probability distributions in self.capability_class_probability_distribution. 
        self.capability_class_probability_distribution_list = [[1/self.num_capability_classes]*self.num_capability_classes] # In the form [[distribution1], [distribution2],...]. A list of probability distributions over all possible capability classes.

        # Capabilities
    

        # # Temporal logic options
        # Operators
        self.temporal_logic_operators_list = [['F','G','FG','GF']] # Can also include 'U', but not included by default
        self.boolean_operators_list = [['AND', 'OR']]
        self.number_of_atomics_per_formula_range_list = [[1,1]] # In the form [[min,max],...]. The number of atomic formulas to include in each generated formula.
        self.operator_interval_range_list = [[0,10]] # In the form [[min, max],...]. Defines how we choose random time bounds for the operators; i.e. for F[a,b], a,b \in [min,max] with a < b.

        # Tasks
        self.task_duration_range_list = [[0,5]] # In the form [[min, max], [min, max]...]. Defines how we choose the duration of tasks; i.e. for a task with duration d, d \in [min,max]
        self.capabilities_per_task_range_list = [[1,1]] # In the form [[min,max]]. Defines the range from which we choose the number of unique capabilities required by a task, i.e. {(UV,1), (IR,4)} requires 2 unique capabilities.
        self.required_agents_per_capability_range_list = [[1,1]] # In the form [[min, max]...]. Number of agents per capability per task, e.g. (UV,4) has 4 agents for the capability UV.


        # # Solver options
        self.solvers_list = ['SCIP', 'GUROBI', 'COIN_CMD']
        self.solver_time_limit_list = [6e2] 

        self.robust_values_list = [True, False] # Include the 'robust' option in testing
        self.regularize_values_list = [True,False]
        self.upper_bound_boolean_list = [True,False]
        self.alpha_value_range_list = [[0.5,0.5]] 

        self.number_of_trials_per_combination = 1 # Integer. The number of trials to perform per combination of all the elements discussed above.
        




        # # Static options (e.g. save locations)
        self.static_options = SimpleNamespace()
        # TODO: Change route_planning and route_online_replanning to turn saving the data on / off as per an input variable.
        self.static_options.save_data_bool = True
        self.static_options.transition_system_dir = './output/last_run'
        self.static_options.transition_system_filename = self.static_options.transition_system_dir + '/TS'
        self.static_options.save_parent_dir = './output/last_run'
        self.static_options.save_filename = './output/last_run'
        # Suppresses computing the IIS when using Gurobi and the model is infeasible
        self.static_options.compute_IIS = False
        # Determines the maximum number of times to attempt to generate a feasible problem before giving up
        self.static_options.max_generation_attempts = 10

        # Set up the directories
        if not os.path.exists(self.static_options.save_parent_dir): #Where .yaml files will be saved
            os.makedirs(self.static_options.save_parent_dir)
        
        if not os.path.exists(self.static_options.transition_system_dir):
            os.makedirs(self.static_options.transition_system_dir)



        # # Internal state variables (not options)

        self._total_trials = 0 #


        # # Make a list of attribute names of attributes that are used directly to generate MILP problem instances.
        # # NOTE: If any additional attributes relating to MILP problem generated are added above,
        # #         be sure to add their names to this list.
        self.attribute_names = [
            "dimensions", # ................................. Transition System
            "state_labels",
            "state_label_probabilities",
            "state_label_multiplicity_function",
            "edge_weight_range",
            "edge_formation_probability",
            "num_agents_per_class_range", #................. Agents / Capabilities
            "num_capabilities_per_agent_range",
            "capability_list",
            "temporal_logic_operators", #........ CaTL Formula		
            "boolean_operators",
            "number_of_atomics_per_formula_range", 
            "operator_interval_range",
            "task_duration_range",
            "capabilities_per_task_range",
            "required_agents_per_capability_range",
            "solver", #.............................. Solver options
            "solver_time_limit",
            "robust_value", 
            "regularize_value",
            "upper_bound_boolean",
            "alpha_value_range",
        ]



    def create_iterator(self):
        '''
        Creates iterator returning namedtuples with all possible combination of solver options.
        '''
        return named_tuple_iterator(
            dimensions = self.dimensions_list, # ................................. Transition System
            state_labels = self.state_labels_list,
            state_label_probabilities = self.state_label_probabilities_list,
            state_label_multiplicity_function = self.state_label_multiplicity_function_list,
            edge_weight_range = self.edge_weight_range_list,
            edge_formation_probability = self.edge_formation_probability_list,
            capability_list = self.capability_list, #................. Agents / Capabilities
            total_num_agents = self.total_num_agents_list, 
            capability_class_probability_distribution = self.capability_class_probability_distribution_list,
            temporal_logic_operators = self.temporal_logic_operators_list, #........ CaTL Formula		
            boolean_operators = self.boolean_operators_list,
            number_of_atomics_per_formula_range = self.number_of_atomics_per_formula_range_list, 
            operator_interval_range = self.operator_interval_range_list,
            task_duration_range = self.task_duration_range_list,
            capabilities_per_task_range = self.capabilities_per_task_range_list,
            required_agents_per_capability_range = self.required_agents_per_capability_range_list,
            solver = self.solvers_list, #.............................. Solver options
            solver_time_limit = self.solver_time_limit_list,
            robust_value = self.robust_values_list, 
            regularize_value = self.regularize_values_list,
            upper_bound_boolean = self.upper_bound_boolean_list,
            alpha_value_range = self.alpha_value_range_list,
            trial_number_per_combination = range(self.number_of_trials_per_combination) 
        )
    
    def create_latin_iterator(self, random_generator=None, repetitions=1):
        '''
        Creates iterator returning latin hypercube samples of the variables in question.

        NOTE: This does NOT use the field self.number_of_trials_per_combination.
        '''
        if random_generator is None:
            rng = np.random.default_rng()
        else:
            rng = random_generator

        return latin_named_tuple_iterator(
            random_generator=rng,
            repetitions=repetitions,
            dimensions = self.dimensions_list, # ................................. Transition System
            state_labels = self.state_labels_list,
            state_label_probabilities = self.state_label_probabilities_list,
            state_label_multiplicity_function = self.state_label_multiplicity_function_list,
            edge_weight_range = self.edge_weight_range_list,
            edge_formation_probability = self.edge_formation_probability_list,
            capability_list = self.capability_list, #................. Agents / Capabilities
            total_num_agents = self.total_num_agents_list, 
            capability_class_probability_distribution = self.capability_class_probability_distribution_list,
            temporal_logic_operators = self.temporal_logic_operators_list, #........ CaTL Formula		
            boolean_operators = self.boolean_operators_list,
            number_of_atomics_per_formula_range = self.number_of_atomics_per_formula_range_list, 
            operator_interval_range = self.operator_interval_range_list,
            task_duration_range = self.task_duration_range_list,
            capabilities_per_task_range = self.capabilities_per_task_range_list,
            required_agents_per_capability_range = self.required_agents_per_capability_range_list,
            solver = self.solvers_list, #.............................. Solver options
            solver_time_limit = self.solver_time_limit_list,
            robust_value = self.robust_values_list, 
            regularize_value = self.regularize_values_list,
            upper_bound_boolean = self.upper_bound_boolean_list,
            alpha_value_range = self.alpha_value_range_list,
        )

    

    def create_latin_file_generator(self, seed=None):
        '''
        Creates a file generator that directly generates problem .mps / .lp files
        without actually solving the problem.
        '''
        return Latin_MILP_File_Generator(self, seed)


    def custom_latin_iterator(self, field_names, random_generator=None, repetitions=1):
        '''
        Creates a latin iterator from a custom list of field names.

        Each field name must be a field in the BenchmarkOptionsGenerator class.
        '''
        pass


    def ts_latin_iterator(self, random_generator=None, repetitions=1):
        '''
        Creates a latin iterator for TS options only.
        '''
        if random_generator is None:
            rng = np.random.default_rng()
        else:
            rng = random_generator

        return latin_named_tuple_iterator(
            random_generator=rng,
            repetitions=repetitions,
            dimensions = self.dimensions_list, # ................................. Transition System
            state_labels = self.state_labels_list,
            state_label_probabilities = self.state_label_probabilities_list,
            state_label_multiplicity_function = self.state_label_multiplicity_function_list,
            edge_weight_range = self.edge_weight_range_list,
            edge_formation_probability = self.edge_formation_probability_list
        )


    def agent_latin_iterator(self, random_generator=None, repetitions=1):
        '''
        Creates a latin iterator for Agent options only.
        '''
        if random_generator is None:
            rng = np.random.default_rng()
        else:
            rng = random_generator

        return latin_named_tuple_iterator(
            random_generator=rng,
            repetitions=repetitions,
            capability_list = self.capability_list, #................. Agents / Capabilities
            total_num_agents = self.total_num_agents_list, 
            capability_class_probability_distribution = self.capability_class_probability_distribution_list,
        )





    def named_tuple_product(self, **items):
        '''
        Returns

        Based on the following Stack Overflow post: https://stackoverflow.com/a/9098295
        CC BY-SA 3.0 https://creativecommons.org/licenses/by-sa/3.0/
        '''
        ProductTuple = namedtuple('ProductTuple', items.keys())
        return starmap(ProductTuple, product(*items.values()))



# # An example of a named tuple that can be used to store output values.
data_column_names = [
    'Trial_Number',
    'MILP_Solver_Used',
    'Runtime_Seconds',
    'Timed_Out',
    'Robust',
    'Regularize',
    'Alpha',
    'Upper_Bound',
    'Grid_TS_Xdim',
    'Grid_TS_Ydim',
    'Number_of_Agents',
    'Agent_List',
    'Number_of_MILP_Constraints',
    'Number_of_MILP_Variables',
    'Objective_Value',
    'Optimizer_Final_Status',
    'Robustness_Variable_Value',
    'Normalized_Travel_Time',
    'TS_Filename',
    'Solution_Filename',
    'Error_Occurred',
    'Formula',
    'Formula_AST_Bound',
    'Formula_Num_Tasks',
    'Formula_Num_TL_Operators',
    'Formula_Num_Bool_Operators',
    'Max_Generation_Attempts_Hit'
]

# Used to store output results in optimization functions
OutputTuple = namedtuple("OutputTuple", data_column_names,
                         defaults=(None,)*len(data_column_names))





# # # Benchmarking Functions

def solve_one_optimization_problem(options_tuple, static_options, trial_number=None, verbose=False, transition_system=None, formula=None, seed=None):
    '''
    Runs one optimization problem using the parameters from the named tuple `options_tuple` argument and the SimpleNamespace object `static_options`.

    The input `tuple` must be a named tuple obtained from BenchmarkOptionsGenerator.create_iterator().

    Returns:
        runtime (float): Running time of the optimization algorithm
        PuLP_Model:

    '''
    rng = np.random.default_rng(seed)

    # # Set up initial containers.
    # hash_value is to ensure name is unique.
    m = types.ModuleType(f"dummymodule_{hash(str(options_tuple)+str(trial_number))}")
    m.load_old_files = False

    # # Initialize arguments.
    # # For all values that are ranges [min,max], choose a random value within the range.

    # Generates unique filename for each TS
    ts_save_filename = static_options.transition_system_filename + str(hash(str(options_tuple) + str(trial_number)))

    if transition_system is not None:
        ts = transition_system
    else:
        # TODO: Make it possible to use other TS generation functions
        ts_name = ts_save_filename
        dimensions = options_tuple.dimensions
        multiplicity = options_tuple.state_label_multiplicity_function(dimensions)
        ts = generateRandomGridTS(ts_name, dimensions, options_tuple.state_labels, options_tuple.edge_weight_range, multiplicity, random_generator=rng)


    # Agents
    # TODO: Implement more general method for choosing random agents
    states = ts.g.nodes()
    class_dict = class_to_capability_dict(options_tuple.capability_list)
    agents = generate_agents_by_total_and_distribution(options_tuple.total_num_agents, class_dict, options_tuple.capability_class_probability_distribution, states, random_generator=rng)

    # Formula
    if formula is None:
        # If no formula was passed in, generate one.
        formula, num_atomics, num_tasks = generate_random_CaTL_formula_and_info(options_tuple, random_generator=rng)

    # Alpha
    if options_tuple.alpha_value_range[0] == options_tuple.alpha_value_range[1]:
        alpha = options_tuple.alpha_value_range[0]
    else:
        low = options_tuple.alpha_value_range[0]
        high = options_tuple.alpha_value_range[1]
        alpha = rng.uniform(low=low, high=high)

    # # Run and time the optimization problem.
    runtime = -1  # For error checking
    try:
        pdb.set_trace() if sys.flags.debug else None
        start_time = timer()
        PuLP_Model, replan_data = route_planning(m, ts, agents, formula,
                                                 file_name=ts_save_filename,
                                                 replan_req=False,
                                                 robust=options_tuple.robust_value,
                                                 regularize=options_tuple.regularize_value,
                                                 alpha=alpha,
                                                 upperBound=options_tuple.upper_bound_boolean,
                                                 load_previous=False,
                                                 solver=options_tuple.solver,
                                                 compute_IIS=static_options.compute_IIS,
                                                 verbose=verbose,
                                                 solver_time_limit=options_tuple.solver_time_limit,
                                                 solver_threads=1)
        runtime = timer() - start_time

        # # Save the resulting data

        timed_out = False
        error_occurred = False

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

        # Length of the formula


        # TODO: Make the output a more general structure? A different function may
        #		need to be written if more general fields are needed.
        output = OutputTuple(
            Trial_Number=trial_number,
            MILP_Solver_Used=options_tuple.solver,
            Runtime_Seconds=runtime,
            Timed_Out=timed_out,
            Robust=options_tuple.robust_value,
            Regularize=options_tuple.regularize_value,
            Alpha=alpha,
            Upper_Bound=options_tuple.upper_bound_boolean,
            Grid_TS_Xdim=options_tuple.dimensions[0],
            Grid_TS_Ydim=options_tuple.dimensions[1],
            Number_of_Agents=len(agents),
            Agent_List=agents,
            Number_of_MILP_Constraints=PuLP_Model.numConstraints(),
            Number_of_MILP_Variables=PuLP_Model.numVariables(),
            Objective_Value=objective_value,
            Optimizer_Final_Status=PuLP_Status,
            Robustness_Variable_Value=rho_value,
            Normalized_Travel_Time=normalized_travel_time,
            TS_Filename=ts_save_filename,
            Error_Occurred=error_occurred,
            Formula=formula,
            Formula_AST_Bound=ast_bound,
            Formula_Num_Tasks=num_tasks,
            Formula_Num_TL_Operators=num_atomics,
            Formula_Num_Bool_Operators=num_atomics-1
        )
    except:
        error_occurred = True

        if verbose:
            if trial_number is not None:
                print(
                    f"\nError in solving optimization problem in trial {trial_number}.")
            else:
                print(
                    f"\nError in solving optimization problem (no trial number given).")

            print("Route_planning did not succeed.\n")

        output = OutputTuple(
            Trial_Number=trial_number,
            MILP_Solver_Used=options_tuple.solver,
            Runtime_Seconds=runtime,
            Robust=options_tuple.robust_value,
            Regularize=options_tuple.regularize_value,
            Alpha=alpha,
            Upper_Bound=options_tuple.upper_bound_boolean,
            Grid_TS_Xdim=options_tuple.dimensions[0],
            Grid_TS_Ydim=options_tuple.dimensions[1],
            Number_of_Agents=len(agents),
            Agent_List=agents,
            Number_of_MILP_Constraints=PuLP_Model.numConstraints(),
            Number_of_MILP_Variables=PuLP_Model.numVariables(),
            Objective_Value=None,
            Optimizer_Final_Status=None,
            Robustness_Variable_Value=None,
            Normalized_Travel_Time=None,
            TS_Filename=ts_save_filename,
            Error_Occurred=error_occurred,
            Formula=formula,
            Formula_AST_Bound=ast_bound,
            Formula_Num_Tasks=num_tasks,
            Formula_Num_TL_Operators=num_atomics,
            Formula_Num_Bool_Operators=num_atomics-1
        )

    print(f"Completed trial number {trial_number}.")

    return output
    # END run_one_optimization_problem



def solve_problem_robust_and_not_robust(options_tuple, static_options, trial_number=None, verbose=False, formula=None, seed=None):
    '''
    Solves an IITCHS problem configuration for both the 'robust' and 'not robust'
    cases, keeping the transition system the same.
    '''
    rng = np.random.default_rng(seed)

    pdb.set_trace() if sys.flags.debug else None
    # # Set up initial containers.
    # hash_value is to ensure name is unique.
    m = types.ModuleType(f"dummymodule_{hash(str(options_tuple)+str(trial_number))}")
    m.load_old_files = False

    # # Initialize arguments.
    # # For all values that are ranges [min,max], choose a random value within the range.

    # Generates unique filename for each TS
    ts_save_filename = static_options.transition_system_filename + str(hash(str(options_tuple) + str(trial_number)))

    # TODO: Make it possible to use other TS generation functions
    ts_name = ts_save_filename
    dimensions = options_tuple.dimensions
    multiplicity = options_tuple.state_label_multiplicity_function(dimensions)

    # Alpha
    if options_tuple.alpha_value_range[0] == options_tuple.alpha_value_range[1]:
        alpha = options_tuple.alpha_value_range[0]
    else:
        low = options_tuple.alpha_value_range[0]
        high = options_tuple.alpha_value_range[1]
        alpha = rng.uniform(low=low, high=high)

    # # Generate the TS, agents, and formula.
    # # 
    # # This for loop generates a TS, set of agents, and CaTL formula.
    # # It then checks the robustness upper bound to see if it's positive.
    # # If so, it continues. If not, it generates a new TS, set of agents,
    # # and formula. This repeats for max_generation_attempts iterations.
    # # 
    # # If max_generation_attempts is hit, the process exits in failure.

    positive_upper_bound = False
    for _ in range(static_options.max_generation_attempts):
        # Transition System
        ts = generateRandomGridTS(ts_name, dimensions, options_tuple.state_labels, options_tuple.edge_weight_range, multiplicity, random_generator=rng)

        # Agents
        # TODO: Implement more general method for choosing random agents
        states = ts.g.nodes()
        class_dict = class_to_capability_dict(options_tuple.capability_list)
        agents = generate_agents_by_total_and_distribution(options_tuple.total_num_agents, class_dict, options_tuple.capability_class_probability_distribution, states, random_generator=rng)

        # Formula
        if formula is None:
            # If no formula was passed in, generate one.
            formula, num_atomics, num_tasks = generate_random_CaTL_formula_and_info(options_tuple, random_generator=rng)

        upper_bound = computeRobustnessUpperBound(ts, agents, formula)
        if upper_bound >= 0:
            # We can continue with the simulation
            positive_upper_bound = True
            break

    # If no positive upper bound was found, return the information from the most recently generated attempt.
    if not positive_upper_bound:
        # # Get information about the formula
        # AST bound
        ast = CATLFormula.from_formula(formula)
        ast_bound = int(ast.bound())

        output = OutputTuple(
                Trial_Number=trial_number,
                MILP_Solver_Used=options_tuple.solver,
                Runtime_Seconds=-1,
                Robust=None,
                Regularize=options_tuple.regularize_value,
                Alpha=alpha,
                Upper_Bound=options_tuple.upper_bound_boolean,
                Grid_TS_Xdim=options_tuple.dimensions[0],
                Grid_TS_Ydim=options_tuple.dimensions[1],
                Number_of_Agents=len(agents),
                Agent_List=agents,
                Number_of_MILP_Constraints=None,
                Number_of_MILP_Variables=None,
                Objective_Value=None,
                Optimizer_Final_Status=None,
                Robustness_Variable_Value=None,
                Normalized_Travel_Time=None,
                TS_Filename=ts_save_filename,
                Error_Occurred=True,
                Formula=formula,
                Formula_AST_Bound=ast_bound,
                Formula_Num_Tasks=num_tasks,
                Formula_Num_TL_Operators=num_atomics,
                Formula_Num_Bool_Operators=num_atomics-1,
                Max_Generation_Attempts_Hit=True
            )

        return [output]


    # Otherwise, continue to simulate.


    pdb.set_trace() if sys.flags.debug else None
    # # Run and time the optimization problem for both robust = True and robust = False
    runtime_robust_true = -1  # For error checking
    runtime_robust_false = -1  
    try:
        # # Robust = True
        robust_true_start_time = timer()
        PuLP_Model_robust_true, replan_data = route_planning(m, ts, agents, formula,
                                                 file_name=ts_save_filename,
                                                 replan_req=False,
                                                 robust=True,
                                                 regularize=options_tuple.regularize_value,
                                                 alpha=alpha,
                                                 upperBound=options_tuple.upper_bound_boolean,
                                                 load_previous=False,
                                                 solver=options_tuple.solver,
                                                 compute_IIS=static_options.compute_IIS,
                                                 verbose=verbose,
                                                 solver_time_limit=options_tuple.solver_time_limit,
                                                 solver_threads=1)
        runtime_robust_true = timer() - robust_true_start_time

        print(f"Completed robust configuration for trial number {trial_number}")


        # # Robust = False
        robust_false_start_time = timer()
        PuLP_Model_robust_false, replan_data = route_planning(m, ts, agents, formula,
                                                 file_name=ts_save_filename,
                                                 replan_req=False,
                                                 robust=False,
                                                 regularize=options_tuple.regularize_value,
                                                 alpha=alpha,
                                                 upperBound=options_tuple.upper_bound_boolean,
                                                 load_previous=False,
                                                 solver=options_tuple.solver,
                                                 compute_IIS=static_options.compute_IIS,
                                                 verbose=verbose,
                                                 solver_time_limit=options_tuple.solver_time_limit,
                                                 solver_threads=1)
        runtime_robust_false = timer() - robust_false_start_time


        # # Save the resulting data

        error_occurred_robust_true = False
        timed_out_robust_true = False

        # Objective value
        if PuLP_Model_robust_true.status == pulp.LpStatusOptimal:
            if PuLP_Model_robust_true.objective is not None:
                objective_value = PuLP_Model_robust_true.objective.value()
            else:
                # Objective value is None if the MILP was a feasibility problem with no objective
                objective_value = 0

            PuLP_Status = "LpStatusOptimal"
        elif PuLP_Model_robust_true.status == pulp.LpStatusNotSolved:
            objective_value = 0
            PuLP_Status = "LpStatusNotSolved"
            if hasattr(PuLP_Model_robust_true, 'solutionTime'):
                # The model timed out.
                timed_out_robust_true = True
            else:
                # The model didn't time out, but still wasn't solved. Error.
                error_occurred_robust_true = True
        elif PuLP_Model_robust_true.status == pulp.LpStatusInfeasible:
            objective_value = 0
            PuLP_Status = "LpStatusInfeasible"
        elif PuLP_Model_robust_true.status == pulp.LpStatusUnbounded:
            objective_value = 0
            PuLP_Status = "LpStatusUnbounded"
        else:
            objective_value = 0
            PuLP_Status = "(unknown_status)"

        variablesDict = PuLP_Model_robust_true.variablesDict()
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

        # Length of the formula


        # TODO: Make the output a more general structure? A different function may
        #		need to be written if more general fields are needed.
        output_robust_true = OutputTuple(
            Trial_Number=trial_number,
            MILP_Solver_Used=options_tuple.solver,
            Runtime_Seconds=runtime_robust_true,
            Timed_Out=timed_out_robust_true,
            Robust=True,
            Regularize=options_tuple.regularize_value,
            Alpha=alpha,
            Upper_Bound=options_tuple.upper_bound_boolean,
            Grid_TS_Xdim=options_tuple.dimensions[0],
            Grid_TS_Ydim=options_tuple.dimensions[1],
            Number_of_Agents=len(agents),
            Agent_List=agents,
            Number_of_MILP_Constraints=PuLP_Model_robust_true.numConstraints(),
            Number_of_MILP_Variables=PuLP_Model_robust_true.numVariables(),
            Objective_Value=objective_value,
            Optimizer_Final_Status=PuLP_Status,
            Robustness_Variable_Value=rho_value,
            Normalized_Travel_Time=normalized_travel_time,
            TS_Filename=ts_save_filename,
            Error_Occurred=error_occurred_robust_true,
            Formula=formula,
            Formula_AST_Bound=ast_bound,
            Formula_Num_Tasks=num_tasks,
            Formula_Num_TL_Operators=num_atomics,
            Formula_Num_Bool_Operators=num_atomics-1,
            Max_Generation_Attempts_Hit=False
        )

        # Save data for when robust = False

        error_occurred_robust_false = False
        timed_out_robust_false = False

        if PuLP_Model_robust_false.status == pulp.LpStatusOptimal:
            if PuLP_Model_robust_false.objective is not None:
                objective_value = PuLP_Model_robust_false.objective.value()
            else:
                # Objective value is None if the MILP was a feasibility problem with no objective
                objective_value = 0

            PuLP_Status = "LpStatusOptimal"
        elif PuLP_Model_robust_false.status == pulp.LpStatusNotSolved:
            objective_value = 0
            PuLP_Status = "LpStatusNotSolved"
            if hasattr(PuLP_Model_robust_false, 'solutionTime'):
                # The model timed out.
                timed_out_robust_false = True
            else:
                # The model didn't time out, but still wasn't solved. Error.
                error_occurred_robust_false = True

        elif PuLP_Model_robust_false.status == pulp.LpStatusInfeasible:
            objective_value = 0
            PuLP_Status = "LpStatusInfeasible"
        elif PuLP_Model_robust_false.status == pulp.LpStatusUnbounded:
            objective_value = 0
            PuLP_Status = "LpStatusUnbounded"
        else:
            objective_value = 0
            PuLP_Status = "(unknown_status)"

        variablesDict = PuLP_Model_robust_false.variablesDict()
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

        # Length of the formula


        # TODO: Make the output a more general structure? A different function may
        #		need to be written if more general fields are needed.
        output_robust_false = OutputTuple(
            Trial_Number=trial_number,
            MILP_Solver_Used=options_tuple.solver,
            Runtime_Seconds=runtime_robust_false,
            Timed_Out=timed_out_robust_false,
            Robust=False,
            Regularize=options_tuple.regularize_value,
            Alpha=alpha,
            Upper_Bound=options_tuple.upper_bound_boolean,
            Grid_TS_Xdim=options_tuple.dimensions[0],
            Grid_TS_Ydim=options_tuple.dimensions[1],
            Number_of_Agents=len(agents),
            Agent_List=agents,
            Number_of_MILP_Constraints=PuLP_Model_robust_false.numConstraints(),
            Number_of_MILP_Variables=PuLP_Model_robust_false.numVariables(),
            Objective_Value=objective_value,
            Optimizer_Final_Status=PuLP_Status,
            Robustness_Variable_Value=rho_value,
            Normalized_Travel_Time=normalized_travel_time,
            TS_Filename=ts_save_filename,
            Error_Occurred=error_occurred_robust_false,
            Formula=formula,
            Formula_AST_Bound=ast_bound,
            Formula_Num_Tasks=num_tasks,
            Formula_Num_TL_Operators=num_atomics,
            Formula_Num_Bool_Operators=num_atomics-1,
            Max_Generation_Attempts_Hit=False
        )





    except:
        output_robust_true = OutputTuple(
            Trial_Number=trial_number,
            MILP_Solver_Used=options_tuple.solver,
            Runtime_Seconds=runtime_robust_true,
            Robust=True,
            Regularize=options_tuple.regularize_value,
            Alpha=alpha,
            Upper_Bound=options_tuple.upper_bound_boolean,
            Grid_TS_Xdim=options_tuple.dimensions[0],
            Grid_TS_Ydim=options_tuple.dimensions[1],
            Number_of_Agents=len(agents),
            Agent_List=agents,
            Number_of_MILP_Constraints=PuLP_Model_robust_true.numConstraints(),
            Number_of_MILP_Variables=PuLP_Model_robust_true.numVariables(),
            Objective_Value=objective_value,
            Optimizer_Final_Status=PuLP_Status,
            Robustness_Variable_Value=rho_value,
            Normalized_Travel_Time=normalized_travel_time,
            TS_Filename=ts_save_filename,
            Error_Occurred=True,
            Formula=formula,
            Formula_AST_Bound=ast_bound,
            Formula_Num_Tasks=num_tasks,
            Formula_Num_TL_Operators=num_atomics,
            Formula_Num_Bool_Operators=num_atomics-1,
            Max_Generation_Attempts_Hit=False
        )

        output_robust_false = OutputTuple(
            Trial_Number=trial_number,
            MILP_Solver_Used=options_tuple.solver,
            Runtime_Seconds=runtime_robust_false,
            Robust=False,
            Regularize=options_tuple.regularize_value,
            Alpha=alpha,
            Upper_Bound=options_tuple.upper_bound_boolean,
            Grid_TS_Xdim=options_tuple.dimensions[0],
            Grid_TS_Ydim=options_tuple.dimensions[1],
            Number_of_Agents=len(agents),
            Agent_List=agents,
            Number_of_MILP_Constraints=PuLP_Model_robust_false.numConstraints(),
            Number_of_MILP_Variables=PuLP_Model_robust_false.numVariables(),
            Objective_Value=objective_value,
            Optimizer_Final_Status=PuLP_Status,
            Robustness_Variable_Value=rho_value,
            Normalized_Travel_Time=normalized_travel_time,
            TS_Filename=ts_save_filename,
            Error_Occurred=True,
            Formula=formula,
            Formula_AST_Bound=ast_bound,
            Formula_Num_Tasks=num_tasks,
            Formula_Num_TL_Operators=num_atomics,
            Formula_Num_Bool_Operators=num_atomics-1,
            Max_Generation_Attempts_Hit=False
        )



    print(f"Completed trial number {trial_number}.")

    return [output_robust_true, output_robust_false]





# # # Other utility functions

def lcm(number_list):
    '''
    Finds the least common multiple of a list of integers
    '''
    lcm = 1
    for a in number_list:
        lcm = lcm*a//gcd(lcm,a)

    return lcm


def latin_hypercube_sample(num_variables, num_bins, random_generator=None):
    '''
    Generates a discrete latin hypercube sampling.

    Given n = num_variables and m = num_bins, this function returns an 
    n x m numpy matrix. The iith column of the matrix contains the coordinates
    for the iith hypercube sample. All samples are guaranteed to be unique in
    their respective (multidimensional) rows and columns.

    Example: 2 variables, 10 bins. A possible return matrix is 

        array([[5, 3, 0, 9, 6, 8, 7, 4, 2, 1],
               [2, 5, 3, 4, 1, 7, 6, 8, 9, 0]])
    
        Each column is the 2D index of a sample that is unique in its row and column
        (i.e. 10 rooks on a chessboard that don't threaten each other).
    
    Example: 3 variables, 10 bins. A possible return matrix is

        array([[1, 9, 4, 2, 6, 7, 3, 0, 8, 5],
               [0, 1, 9, 2, 7, 6, 4, 3, 5, 8],
               [6, 4, 8, 3, 0, 2, 9, 7, 5, 1]])
        
        Each column is the 3D index of a sample that is unique in its 3D rows /columns
        (i.e. 10 rooks in a 3D cube chessboard that don't threaten each other)

    TODO: Move this to generation_tools.py?
    '''
    if random_generator is None:
        rng = np.random.default_rng()
    else:
        rng = random_generator

    return np.vstack([rng.permutation(num_bins) for ii in range(num_variables)])


# def latin_hypercube_iterator(*iterables, random_generator=None):
#     '''
#     Generates an iterator using Latin hypercube sampling.

#     Can take a variable number of finite iterables as input.
#     '''







if __name__ == "__main__":
    print("\nNot intended to be run as a standalone module.\n")