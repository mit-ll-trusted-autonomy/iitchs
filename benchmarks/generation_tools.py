'''
Random generation tools for IITCHS benchmarking purposes.

The generation functions fall roughly in three groups:

    * Transition System generation
    * Agent / Capability generation
    * CaTL Formula generation

Some of the functions are modifications of functions in the file RandomizedCaTLExperiments.py.

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


from os import replace
import numpy as np
import random
import itertools as it
import networkx as nx
import lomap as lm
import math

import sys
if sys.flags.debug:
    import pdb

# # # Transition System Generation

def generateRandomGridTS(tsname,dimensions,labelNames,edgeWeightRange,max_labels_per_state,random_generator=None):
    '''
    Generates a transition system in the form of a grid.

    All states

    Note:
        This function is loosely based on the function `generateRandomGridTS_seed` in RandomizedCaTLExperiments.py.
        The main differences include:

        - A random_generator is passed into this function instead of a seed number.
        - The method of assigning labels is different
    '''

    if random_generator is None:
        random_generator = np.random.default_rng()

    randomGraph = nx.DiGraph(nx.grid_graph(dimensions))
    node_nums = randomGraph.nodes()

    # Generate random labels
    for node_name in node_nums:
        num_labels = random_generator.integers(max_labels_per_state+1)
        randomGraph.node[node_name]['prop'] = set(random_generator.choice(labelNames, size=num_labels))

    # Ensure that each label is applied to at least one state
    num_nodes = len(node_nums)
    for label in labelNames:
        node_idx = random_generator.integers(num_nodes)
        node = node_nums[node_idx]
        pdb.set_trace() if sys.flags.debug else None
        if label not in randomGraph.node[node]['prop']:
            # Add the label to the state's set of labels
            randomGraph.node[node]['prop'] = set.union(randomGraph.node[node]['prop'], set([label]))

    # Generate Edge weights
    nxedges = randomGraph.edges()
    edgeWeightSet = random_generator.integers(edgeWeightRange[0],high=edgeWeightRange[1],size=len(nxedges))
    for k in range(len(nxedges)):
        randomGraph[nxedges[k][0]][nxedges[k][1]]['weight'] = edgeWeightSet[k]
    #Add in self-loops
    randomGraph.add_weighted_edges_from([(node,node,1) for node in randomGraph.nodes()])

    # Add grave state
    randomGraph.add_node('grave')
    randomGraph.node['grave']['prop'] = set(['grave'])
    # Add edges to grave state
    for x in node_nums:
        randomGraph.add_edge(x,'grave',weight = 1)
        randomGraph.add_edge('grave',x,weight = 1000)


    tsModel = lm.Ts(tsname,directed=True,multi=False)
    tsModel.g = randomGraph
    tsModel.init=[tsModel.g.nodes()[0]]
    tsModel.current=tsModel.init
    tsModel.final=tsModel.init
    tsModel.save(tsname)

    # Calculate some of the TS state label statistics.
    # This will help in determining if a given formula is infeasible for a
    # TS, e.g., if there are 5 states with "white" labels but only 4 agents
    # and the formula specifies a "white" label task.

    return tsModel




def generateGridTS_label_counts(tsname,dimensions,labelNames,edgeWeightRange,label_counts,random_generator=None):
    '''
    Generates a transition system in the form of a grid, with a specified number of occurrences for each label.

    Similar to generateRandomGridTS. The main difference is that a ``label_counts`` vector is passed in specifying
    how many times each label should occur in the transition system. States are allowed to have multiple labels.    

    Args:
        tsname (str):       Name of the transition system
        dimensions (list):  Two-element list with the x, y dimensions of the grid, e.g.  ``[xdim, ydim]``
        labelNames (list):  String names of each label
        edgeWeightRange (list):
        label_counts (list):    List of integers specifying how many times each label should occur in the transition
                                system. Must have the same length as the ``labelNames`` list.
        random_generator:       Numpy random generator object. If unspecified, a new one is created using
                                ``np.random.default_rng()``.

    Returns:
        (): A Lomap transition system
    '''

    if random_generator is None:
        random_generator = np.random.default_rng()

    randomGraph = nx.DiGraph(nx.grid_graph(dimensions))
    node_nums = randomGraph.nodes()

    # Initialize the label sets
    for node in node_nums:
        randomGraph.node[node]['prop'] = set()

    # Apply the labels to states in the graph
    pdb.set_trace() if sys.flags.debug else None
    for idx, label in enumerate(labelNames):
        count = label_counts[idx]
        chosen_node_idx = random_generator.choice(range(len(node_nums)), size=count, replace=False)
        for node_idx in chosen_node_idx:
            randomGraph.node[node_nums[node_idx]]['prop'] = set.union(randomGraph.node[node_nums[node_idx]]['prop'], set([label]))

    # # Generate random labels
    # for node_name in node_nums:
    #     num_labels = random_generator.integers(max_labels_per_state+1)
    #     randomGraph.node[node_name]['prop'] = set(random_generator.choice(labelNames, size=num_labels))

    # # Ensure that each label is applied to at least one state
    # num_nodes = len(node_nums)
    # for label in labelNames:
    #     node_idx = random_generator.integers(num_nodes)
    #     node = node_nums[node_idx]
    #     pdb.set_trace() if sys.flags.debug else None
    #     if label not in randomGraph.node[node]['prop']:
    #         # Add the label to the state's set of labels
    #         randomGraph.node[node]['prop'] = set.union(randomGraph.node[node]['prop'], set([label]))

    # Generate Edge weights
    nxedges = randomGraph.edges()
    edgeWeightSet = random_generator.integers(edgeWeightRange[0],high=edgeWeightRange[1],size=len(nxedges))
    for k in range(len(nxedges)):
        randomGraph[nxedges[k][0]][nxedges[k][1]]['weight'] = edgeWeightSet[k]
    #Add in self-loops
    randomGraph.add_weighted_edges_from([(node,node,1) for node in randomGraph.nodes()])

    # Add grave state
    randomGraph.add_node('grave')
    randomGraph.node['grave']['prop'] = set(['grave'])
    # Add edges to grave state
    for x in node_nums:
        randomGraph.add_edge(x,'grave',weight = 1)
        randomGraph.add_edge('grave',x,weight = 1000)


    tsModel = lm.Ts(tsname,directed=True,multi=False)
    tsModel.g = randomGraph
    tsModel.init=[tsModel.g.nodes()[0]]
    tsModel.current=tsModel.init
    tsModel.final=tsModel.init
    tsModel.save(tsname)

    # Calculate some of the TS state label statistics.
    # This will help in determining if a given formula is infeasible for a
    # TS, e.g., if there are 5 states with "white" labels but only 4 agents
    # and the formula specifies a "white" label task.

    return tsModel




# # # Agent generation

# # Functions for creating probability distributions over agent classes

def probability_over_capability_cardinality(cardinality_distr):
    '''
    Generates a probability distribution based on how likely it is for agents to have a _number_ of capabilities, e.g.
    1 capability vs. 3 capabilities.

    A capability cardinality class is a set of capability classes, each class having the same cardinality of capabilities.
    For example,
        - The capability classes {UV}, {IR}, {Mo} are all of cardinality class 1 (one capability per class)
        - The capability classes {UV,IR}, {UV,Mo}, {IR, Mo} are all of cardinality class 2 (two capabilities per class)
        - The capability class {UV,IR,Mo} is of cardinality class 3 (three capabilities per class)

    This function assumes that each cardinality class C_i is given a probability weight w_i (e.g., the probability of an agent
    having 1, or 2, or 3 capabilities). The probability w_i is then divided uniformly among all possible capability classes 
    within that cardinality class. 
    
    More specifically, given n capabilities, the capability cardinality class k has (n choose k) agents. If class k is given
    probability weight w_k, then each capability class in cardinality class k has probability w_k / (n choose k).

    Example:
        Suppose our capabilities are {UV, IR, Mo}. Suppose that ``cardinality_distr = [0.2, 0.3, 0.5]``. Then:
            - Each capability class of cardinality 1 ({UV}, {Mo}, {IR}) would have probability 0.2 / 3
            - Each capability class of cardinality 2 ({UV,Mo}, {Mo,IR}, {UV,IR}) would have probability 0.3 / 3
            - Each capability class of cardinality 3 ({UV,Mo,IR}) would have probability 0.5 / 1

    '''

    n = len(cardinality_distr) # Number of capabilities

    output = [[cardinality_distr[k-1]/math.comb(n,k)]*math.comb(n,k) for k in range(1,n+1)]
    output = [p for sublist in output for p in sublist] # Combine all entries into one list

    return output


def class_to_capability_dict(capabilities, presort=True):
    '''
    Creates dictionary mapping capability class to the corresponding sets of capabilities.

    For example, given the capabilities (UV, IR, Mo), the dictionary might satisfy::

        class_to_capability_dict = {
            0: set('UV'),
            1: set('IR'),
            2: set('Mo'),
            3: set(['UV','IR']),
            4: set(['IR','Mo']),
            5: set(['UV','Mo']),
            6: set(['UV','IR','Mo'])
        }
    

    Note:
        To ensure results are deterministic, by default the list of capabilities is sorted alphabetically
        in a case insensitive manner (using ``str.casefold``) before creating the dictionary. To turn off
        this sorting, use the keyword argument ``presort=False``.

    Args:
        capabilities (list):    List of capability strings.
        presort (bool):         Boolean. If True, the list of capabilities is sorted in a
                                case-insensitive manner before creating the dictionary. True
                                by default.
    '''

    if presort:
        capabilities = sorted(capabilities, key=str.casefold)


    capability_combinations = []

    for k in range(1, len(capabilities)+1):
        for combination in list(it.combinations(capabilities, k)):
            capability_combinations += [set(combination)]

    return {idx: combo for idx, combo in enumerate(capability_combinations)}





def generate_agents_by_total_and_distribution(total_agents, class_to_capability_dict, capability_class_distribution, initial_state_list, random_generator=None):
    '''
    Generates agents by specifying the total number of agents and a probability distribution over
    all possible capability classes.

    Args:
        total_agents (int):         The total number of agents to create
        class_to_capability_dict:   Dictionary mapping capability class to the corresponding sets of capabilities. For example,
                                    given the capabilities (UV, IR, Mo), the dictionary might satisfy
                                    ```python
                                    class_to_capability_dict = {
                                        0: set('UV'),
                                        1: set('IR'),
                                        2: set('Mo'),
                                        3: set(['UV','IR']),
                                        4: set(['IR','Mo']),
                                        5: set(['UV','Mo']),
                                        6: set(['UV','IR','Mo'])
                                    }
                                    ```
        capability_class_distribution (list):  List of probabilities for each capability class
        initial_state_list:                 List of potential starting states for the agents
        random_generator:           A Numpy random generator. If None, the function creates one using np.random.default_rng().

    Returns:
        (list):     A list of agents.
    '''
    if random_generator is not None:
        rng = random_generator
    else:
        rng = np.random.default_rng()

    
    classes = rng.choice(list(class_to_capability_dict.keys()), size=total_agents, p=capability_class_distribution)
    states = rng.choice(initial_state_list, size=total_agents)
    capabilities = [class_to_capability_dict[agent_class] for agent_class in classes]

    output_agents = [(states[ii], capabilities[ii]) for ii in range(total_agents)]
    return output_agents





def generate_random_agents(state_list, capability_list, num_capabilities_per_agent_range, number_of_agents_generated=1, seed=None):
    '''
    Generates random arrays of agents.

    The num_capabilities_per_agent_range is in the form [min,max]. Agents are given a random number of capabilities, where
    the number of capabilities is in this range. To specify a specific number of capabilities, set min == max.
    '''
    assert num_capabilities_per_agent_range[0] <= num_capabilities_per_agent_range[1], "Incorrect range for number of capabilities."

    if seed is not None:
        np.random.seed(seed)

    output_agents = []

    for ii in range(number_of_agents_generated):
        state = np.random.choice(state_list)

        if num_capabilities_per_agent_range[0] == num_capabilities_per_agent_range[1]:
            num_capabilities = num_capabilities_per_agent_range[0]
        else:
            num_capabilities = np.random.randint(num_capabilities_per_agent_range[0], num_capabilities_per_agent_range[1]+1)
        
        capabilities = set(np.random.choice(capability_list, size=num_capabilities, replace=False).tolist())

        output_agents += [(state,capabilities)]

    return output_agents



def generate_agents_by_class(state_list, capability_list, num_agents_per_class, random_generator=None):
    '''
    Generates an equal amount of agents per agent class.

    Initial starting states
    '''
    if random_generator is None:
        random_generator = np.random.default_rng()

    output_agents = []

    for ii in range(1,len(capability_list)+1):
        # Iterate through all possible combinations
        capability_combinations = list(it.combinations(capability_list,ii))

        for capabilities in capability_combinations:
            for _ in range(num_agents_per_class):

                state = random_generator.choice(np.array(state_list, dtype=object))
                output_agents += [(state,set(capabilities))]

    return output_agents




# # # CaTL Formula Generation

def task_capabilities_to_string(capabilities, num_required_agents):
    '''
    Converts a list of capabilities and a list of number of required agents
    to string form for CaTL task formulas. 

    Args:
        capabilities (list): A list of string capabilities, e.g. ``['UV', 'Mo', 'Vis', 'IR']``.
        num_required_agents (list): A list of integers specifying how many agents are needed for
            the corresponding entries in ``capabilities``.

    Returns:
        (string): A string representation of the capabilities needed for a task, e.g. '{(UV,1)}'
        or '{(IR,2),(Mo,3)}'.

    '''
    assert len(capabilities) == len(num_required_agents), "Length of both input lists must be equal."

    output_string = "{"
    for ii, (capability, num) in enumerate(zip(capabilities, num_required_agents)):
        output_string += f"({capability},{num})"
        if ii < len(capabilities)-1:
            output_string += ","

    output_string += "}"
    return output_string



def tasks_to_strings(durations, state_labels, capability_strings):
    '''
    Creates a Python list of task strings from the inputs.

    Args:
        durations (list): List of nonnegative integer time durations
        state_labels (list): List of string state labels
        capability_strings (list): List of capability strings, e.g. '{(UV,1)}' or '{(Vis,2),(IR,4)}'.
            Use task_capabilities_to_string() function.

    Returns:
        (list): A Python list of task strings.
    '''
    assert len(durations) == len(state_labels) == len(capability_strings), "Length of all input lists must be equal."

    output_strings = ["T("]*len(durations)

    for ii, (duration, state, capability) in enumerate(zip(durations, state_labels, capability_strings)):
        output_strings[ii] += f"{duration},{state},{capability})"

    return output_strings
    


def FG_operators_and_tasks_to_strings(operator_strings, intervals, task_strings):
    '''
    Creates a Python list of formulas from the inputs.

    
    This function only combines the operators ``F``, ``G``, ``FG``, ``GF`` with tasks.
    To combine formulas together using AND and OR, use the function [INSERT HERE].

    Args:

    Returns:
    '''
    assert len(operator_strings) == len(intervals) == len(task_strings)

    output_strings = [""]*len(operator_strings)

    for ii, (operator, interval, task) in enumerate(zip(operator_strings, intervals, task_strings)):
        if operator in ['F','G']:
            assert interval[0] <= interval[1], f"Interval for operator {operator} has invalid times."
            output_strings[ii] += f"{operator}{interval} {task}"
        elif operator in ['FG', 'GF']:
            first_interval = interval[0]
            second_interval = interval[1]
            try:
                assert first_interval[0] <= first_interval[1], f"First interval for operator {operator} has invalid times."
                assert second_interval[0] <= second_interval[1], f"Second interval for operator {operator} has invalid times."
            except:
                pdb.set_trace() if sys.flags.debug else None
            output_strings[ii] += f"{operator[0]}{first_interval} {operator[1]}{second_interval} {task}"
        else:
            raise NotImplementedError("Other operators are not implemented.")

    return output_strings


def combine_formulas_AND_OR(formulas, operators):
    '''
    Combines multiple formulas together with AND and OR operators.

    Formulas and operators must be listed in order; i.e. entry ii of 
    operators goes between entries ii and ii+1 of formulas.

    Args:
        formulas (list): List of formula strings consisting of F, G, FG, GF.
        operators (list): List of AND / OR operators in the form "AND", "OR",
            "&&", or "||". Must be listed in order of combination, i.e. entry
            ii goes between entries ii and ii+1 from ``formulas``. 

    Returns:
        (str): String of the resulting formula.

    '''
    try:
        assert len(formulas) == len(operators) + 1, "Length of formulas must be exactly one more than length of operators."
    except:
        pdb.set_trace() if sys.flags.debug else None

    output_string = formulas[0]

    for formula, operator in zip(formulas[1:], operators):
        if operator == "AND":
            operator = "&&"

        if operator == "OR":
            operator = "||"

        output_string += f" {operator} {formula}"

    return output_string



def generate_random_CaTL_formula_and_info(options_tuple, random_generator=None):
    '''
    Generates one random TL formula and returns it, along with other information about the formula.

    Args:
        options_tuple (namedtuple):
        random_generator: Numpy random generator, e.g. np.random.default_rng(). If None, then the 
            function will create one using np.random.default_rng(). Passing in a random number
            generator helps prevent similar results when running multiple benchmark simulations
            in parallel.

    Returns:
        (tuple): Output tuple including the following elements:
            - **formula** *(string)*: The CaTL formula
            - **ast_bound** *(int)*: The AST bound of the formula.
            - **number_of_atomics_per_formula** *(int)*: The number of atomics in the formula. Note that by
                the definition of an atomic, this number is identical to:
                    - The number of tasks # TODO: Only true if U isn't included
                    - The number of TL operators
                    - The number of boolean operators + 1
            - **num_tasks** *(int)*: The number of tasks that are in the formula
            - **num_tl_operators** *(int)*: The total number of F, G, FG, GF, or U operators
                found in the formula.
            - **num_bool_operators** *(int)*: The total number of AND and OR operators
                found in the formula.
    '''
    # Set up random generator.
    # Reason for pas
    if random_generator is None:
        random_generator = np.random.default_rng()

    # Determine how many atomics are included in the formula.
    if options_tuple.number_of_atomics_per_formula_range[0] == options_tuple.number_of_atomics_per_formula_range[1]:
        number_of_atomics_per_formula = options_tuple.number_of_atomics_per_formula_range[0]
    else:
        # Choose random integer within range.
        low = options_tuple.number_of_atomics_per_formula_range[0]
        high = options_tuple.number_of_atomics_per_formula_range[1]+1
        number_of_atomics_per_formula = random_generator.integers(low=low, high=high)


    # Create list of capabilities
    capability_strings = [""]*number_of_atomics_per_formula
    for jj in range(number_of_atomics_per_formula):
        # TODO: Limit capabilities required per task by some variable (e.g. max_capabilities_per_task)
        # capability_list = list(compress(self.capability_list, np.random.randint(0,2,(len(self.capability_list)),dtype=bool)))
        capability_list = random_generator.choice(options_tuple.capability_list, size=random_generator.integers(options_tuple.capabilities_per_task_range[0],options_tuple.capabilities_per_task_range[1]+1), replace=False)
        num_required_agents = random_generator.integers(options_tuple.required_agents_per_capability_range[0], options_tuple.required_agents_per_capability_range[1]+1, (len(capability_list)))
        capability_strings[jj] = task_capabilities_to_string(capability_list, num_required_agents)

    # Generate random tasks
    duration_list = random_generator.integers(options_tuple.task_duration_range[0], options_tuple.task_duration_range[1]+1, (number_of_atomics_per_formula,))
    # label_list = random.choices(options_tuple.state_labels, weights=options_tuple.state_label_probabilities, k=number_of_atomics_per_formula)
    if hasattr(options_tuple, 'state_label_probabilities'):
        state_label_probabilities = options_tuple.state_label_probabilities
    else:
        num_labels = len(options_tuple.state_labels)
        state_label_probabilities = [1/num_labels]*num_labels # Uniform distribution


    label_list = random_generator.choice(options_tuple.state_labels, p=state_label_probabilities, size=number_of_atomics_per_formula)

    tasks = tasks_to_strings(duration_list, label_list, capability_strings)

    # Generate random CaTL atomic formulas
    operators = random_generator.choice(options_tuple.temporal_logic_operators, size=number_of_atomics_per_formula)

    intervals = [[]]*number_of_atomics_per_formula
    for jj in range(number_of_atomics_per_formula):
        a = random_generator.integers(options_tuple.operator_interval_range[0], options_tuple.operator_interval_range[1]+1) # Constrain a to not be equal to the max value; otherwise b is ill-defined.
        b = random_generator.integers(a, options_tuple.operator_interval_range[1]+1)
        if operators[jj] in ['F', 'G']:
            intervals[jj] = [a,b]
        elif operators[jj] in ['FG', 'GF']:
            c = random_generator.integers(options_tuple.operator_interval_range[0], options_tuple.operator_interval_range[1]+1) # Constrain a to not be equal to the max value; otherwise b is ill-defined.
            d = random_generator.integers(c, options_tuple.operator_interval_range[1]+1)
            intervals[jj] = [[a,b], [c,d]]
        else:
            # Not implemented yet.
            raise NotImplementedError(f"The operator {operators[jj]} is not implemented yet.")
            


    atomic_formulas = FG_operators_and_tasks_to_strings(operators, intervals, tasks)

    # Combine random CaTL atomic formulas with AND / OR
    boolean_operators = random_generator.choice(options_tuple.boolean_operators, size=number_of_atomics_per_formula-1)
    output_formula = combine_formulas_AND_OR(atomic_formulas, boolean_operators)

    # TODO: The below line is only true when the operator 'U' is not used.
    # 		Change to compute this number properly when 'U' is used.
    num_tasks = number_of_atomics_per_formula

    return output_formula, number_of_atomics_per_formula, num_tasks




def generate_multiple_random_CaTL_formula(options_tuple, number_of_formulas=1):
    '''
    Generates random TL formulas.

    Args:
        number_of_formulas (int): Number of formula strings to generate.
        number_of_atomics_per_formula (int): Number of atomics to include in each formula
            string. An "atomic" is one temporal operator (e.g. `F`, `G`, `FG`, `GF`, `U`)
            combined with one task (two tasks for the `U` operator). Atomics are combined
            into one formula with the `AND` / `OR` boolean operators.

    Returns:
        (list): List of formula strings.
    '''

    output_formulas = [""]*number_of_formulas

    for ii in range(number_of_formulas):

        # Determine how many atomics are included in the formula.
        if options_tuple.number_of_atomics_per_formula_range[0] == options_tuple.number_of_atomics_per_formula_range[1]:
            number_of_atomics_per_formula = options_tuple.number_of_atomics_per_formula_range[0]
        else:
            # Choose random integer within range.
            low = options_tuple.number_of_atomics_per_formula_range[0]
            high = options_tuple.number_of_atomics_per_formula_range[1]+1
            number_of_atomics_per_formula = np.random.randint(low, high=high)


        # Create list of capabilities
        capability_strings = [""]*number_of_atomics_per_formula
        for jj in range(number_of_atomics_per_formula):
            # TODO: Limit capabilities required per task by some variable (e.g. max_capabilities_per_task)
            # capability_list = list(compress(self.capability_list, np.random.randint(0,2,(len(self.capability_list)),dtype=bool)))
            capability_list = random.sample(options_tuple.capability_list, k=np.random.randint(options_tuple.capabilities_per_task_range[0],options_tuple.capabilities_per_task_range[1]+1))
            num_required_agents = np.random.randint(options_tuple.required_agents_per_capability_range[0], options_tuple.required_agents_per_capability_range[1]+1, (len(capability_list)))
            capability_strings[jj] = task_capabilities_to_string(capability_list, num_required_agents)

        # Generate random tasks
        duration_list = np.random.randint(options_tuple.task_duration_range[0], options_tuple.task_duration_range[1]+1, (number_of_atomics_per_formula,))
        state_list = random.choices(options_tuple.state_labels, weights=options_tuple.state_label_probabilities, k=number_of_atomics_per_formula)

        tasks = tasks_to_strings(duration_list, state_list, capability_strings)

        # Generate random CaTL atomic formulas
        operators = random.choices(options_tuple.temporal_logic_operators, k=number_of_atomics_per_formula)

        intervals = [[]]*number_of_atomics_per_formula
        for jj in range(number_of_atomics_per_formula):
            a = np.random.randint(options_tuple.operator_interval_range[0], options_tuple.operator_interval_range[1]+1) # Constrain a to not be equal to the max value; otherwise b is ill-defined.
            b = np.random.randint(a, options_tuple.operator_interval_range[1]+1)
            if operators[jj] in ['F', 'G']:
                intervals[jj] = [a,b]
            elif operators[jj] in ['FG', 'GF']:
                c = np.random.randint(options_tuple.operator_interval_range[0], options_tuple.operator_interval_range[1]+1) # Constrain a to not be equal to the max value; otherwise b is ill-defined.
                d = np.random.randint(c, options_tuple.operator_interval_range[1]+1)
                intervals[jj] = [[a,b], [c,d]]
                


        atomic_formulas = FG_operators_and_tasks_to_strings(operators, intervals, tasks)

        # Combine random CaTL atomic formulas with AND / OR
        boolean_operators = random.choices(options_tuple.boolean_operators, k=number_of_atomics_per_formula-1)
        output_formulas[ii] = combine_formulas_AND_OR(atomic_formulas, boolean_operators)


    return output_formulas

