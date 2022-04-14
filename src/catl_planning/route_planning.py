# SPDX-License-Identifier: BSD-3-Clause

# Contributors:
#   Cristian Ioan Vasile (cvasile@mit.edu) (cvasile@lehigh.edu)
#   Zachary Serlin (zachary.serlin@ll.mit.edu)  
#   James Usevitch (james.usevitch@ll.mit.edu)
#   Makai Mann (makai.mann@ll.mit.edu)

import os
import sys
if sys.flags.debug:
	import pdb

import contextlib
import logging
import pulp
from catl_planning.write_sol import write_sol
# from lomap.lomap.classes.timer import Timer
from python_stl.stl.stl2milp import stl2milp_pulp #TODO: Fix this to just pulp version?
from catl import *
from catl2stl import catl2stl
from catl_planning.visualization import show_environment
import pickle
from catl_planning.history_builder import regCapHist
import gzip
import shutil



class empty_struct:
    pass



# Decorator function to supress output
def suppress_output(func):
    '''
    Prevents a function from printing output to terminal.

    Redirects the output to /dev/null. Used to suppress the output of the optimization
    solvers.
    '''
    def wrapper(*args, **kwargs):
        with open(os.devnull,'w') as dev_null:
            with contextlib.redirect_stdout(dev_null):
                return func(*args, **kwargs)


    return wrapper



def add_proposition_constraints_pulp(mpulp, stl_milp, ts, ast, capabilities,
                                agent_classes, bound, vtype=pulp.LpInteger,num_agents=1000):
    '''Adds the proposition constraints. First, the proposition-state variables
    are defined such that capabilities are not double booked. Second, contraints
    are added such that proposition are satisfied as best as possible. The
    variables in the MILP encoding of the STL formula are used for the encoding
    as the minimizers of over proposition-state variables.

    This function uses the PuLP modeling language rather than Gurobi directly.

    Input
    -----
    - The PuLP model variable.
    - The MILP encoding of the STL formula obtained from the CaTL specification.
    - The transition system specifying the environment.
    - The AST of the CaTL specification formula.
    - Dictionary of capability encoding that maps capabilities to binary words
    represented as integers.
    - The agent classes given as a dictionary from frozen sets of capabilities
    to bitmaps (integers).
    - Time bound.
    - Variable type (default: integer).'F[0, 10] T(2, orange, {(UV, 1), (Mo, 1)})'
    '''
    props = extract_propositions(ts, ast)

    # add proposition-state variables
    for u, ud in ts.g.nodes(data=True):
        ud['prop_vars'] = dict()
        for c in capabilities:
            ud['prop_vars'][c] = []
            for k in range(bound+1):
                ud['prop_vars'][c].append(dict())
                for prop in ud['prop']:
                    name = 'z_{}_{}_{}_{}'.format(prop, u, c, k)
                    ud['prop_vars'][c][k][prop] = pulp.LpVariable(name, cat=vtype, lowBound=0, upBound=num_agents)


    # constraints for relating (proposition, state) pairs to system states
    for u, ud in ts.g.nodes(data=True):
        for c in capabilities:
            for k in range(bound+1):
                equality = sum([ud['prop_vars'][c][k][prop]
                                                    for prop in ud['prop']])
                equality -= sum([ud['vars'][k][g] for g in agent_classes
                                                                    if c in g])
                mpulp += equality == 0, 'prop_state_{}_{}_{}'.format(u, c, k)

    # add propositions constraints for only those variables appearing in the
    # MILP encoding of the formula
    for prop in props:
        for c in capabilities:
            for k in range(bound+1):
                variable = '{}_{}'.format(prop, c)
                if (variable in stl_milp.variables
                                        and k in stl_milp.variables[variable]):
                    minPropTotal = -1
                    for u, ud in ts.g.nodes(data=True):
                        if prop in ud['prop']:
                            pvar = pulp.LpVariable('p_{}_{}_{}_{}'.format(prop, c, k, u), cat=pulp.LpBinary)
                            minPropTotal += pvar
                            minPropLB = (ud['prop_vars'][c][k][prop] -(1-pvar)*stl_milp.M <= stl_milp.variables[variable][k])
                            minPropUB = (ud['prop_vars'][c][k][prop] +(1-pvar)*stl_milp.M>= stl_milp.variables[variable][k])
                            min_prop = (stl_milp.variables[variable][k] <= ud['prop_vars'][c][k][prop])
                            mpulp += min_prop, 'min_prop_{}_{}_{}_{}'.format(prop, c, k, u)
                            mpulp += minPropLB, 'min_prop_lb_{}_{}_{}_{}'.format(prop, c, k, u)
                            mpulp += minPropUB, 'min_prop_ub_{}_{}_{}_{}'.format(prop, c, k, u)

                    minPropTotal = (minPropTotal == 0)
                    mpulp += minPropTotal, 'min_prop_total_{}_{}_{}_{}'.format(prop, c, k, u)


def add_system_constraints_pulp(mpulp, ts, agent_classes, capability_distribution, bound):
    '''Computes the constraints that capture the system dynamics.

    Input
    -----
    - The PuLP model object.
    - The transition system specifying the environment.
    - The agent classes given as a dictionary from frozen sets of capabilities
    to bitmaps (integers).
    - The initial distribution of capabilities at each state.
    - Time bound.

    Note
    ----
    The initial time constraints

        z_{state}_g_0 = \eta_{state}_g

    is equivalent to

        \sum_{e=(u, v) \in T} z_e_g_W(e) = \eta_{state}_g

    because of the definition of the team state at TS states,
    where \eta_{state}_g is the number of agents of class g at state {state} at
    time 0.
    '''
    # edge conservation constraints
    for u, ud in ts.g.nodes(data=True):
        for k in range(1,bound+2):
            for g, g_enc in agent_classes.items():
                conserve = sum([d['vars'][k-d['weight']][g]
                            for start,finish, d in ts.g.in_edges_iter(u, data=True)
                                if (k - 1 - d['weight']) >= 0 or start == finish])

                # node constraint: team state
                team_state_eq = (ud['vars'][k-1][g] == conserve)
                mpulp += (ud['vars'][k-1][g] == conserve), 'team_{}_{}_{}'.format(u, g_enc, k)

                # flow balancing constraint
                # Note: This includes self-loops!
                conserve -= sum([d['vars'][k][g]
                            for _, _, d in ts.g.out_edges_iter(u, data=True)])
                mpulp += conserve == 0, 'conserve_{}_{}_{}'.format(u, g_enc, k)


    # initial time constraints - encoding using state variables
    for u, ud in ts.g.nodes(data=True):
        for g, g_enc in agent_classes.items():
            for start, finish, d in ts.g.in_edges_iter(u, data=True):
                if start == finish:
                    conserve = (d['vars'][0][g] == capability_distribution[u][g_enc])
                    mpulp += conserve, 'init_distrib_{}_{}_{}'.format(start,finish,g_enc)
                else:
                    conserve = (d['vars'][0][g] == 0)
                    mpulp += conserve, 'init_distrib_{}_{}_{}'.format(start,finish,g_enc)



def compute_capability_bitmap(agents):
    '''
    Computes a bitmap encoding of agents' capabilities. Each capability is
    associated with a bit in a binary word of length equal to the number of capabilities.

    Args:
        agents (list): List containing agent information. The iith entry is a tuple (q, cap)
            corresponding to the iith agent, where q is the initial state of
            the agent and cap is the set of all capabilities that agent has.

            An example of a valid agents list::

                agents = [('q1', {'VIS'}), ('q3', {'LID'}), ('q7', {'LID','IR'})]

    Returns:
        (dict): Dictionary mapping capabilities to integers representing the
        binary words for the capabilities.
    '''
    capabilities = list(set.union(*[cap for _, cap in agents]))
    capabilities.sort()
    capabilities = {c: 1<<k for k, c in enumerate(capabilities)}
    logging.debug('Capabilities bitmap: %s', capabilities)

    return capabilities


def compute_agent_classes(agents, capabilities):
    '''
    Computes the set of agent types w.r.t. capabilities.

    Args:
        agents (list): List of agents, where agents are tuples (q, cap), q is the initial state of
            the agent, and cap is the set of capabilities. Agents' identifiers are their
            indices in the list.
        capabilities (dict): Dictionary of capability class encoding that maps capability classes to binary words
            represented as integers.

    Returns:
        (dict): Dictionary of agent capability classes that maps frozen (immutable) sets of
        capabilities to the binary words encoding the corresponding capability classes.
    '''
    return {frozenset(g): sum([capabilities[c] for c in g]) for _, g in agents}


def compute_initial_capability_distribution(ts, agents, capabilities):
    '''Computes the initial number of agents of each class at each state.
    Input
    -----
    - The transition system specifying the environment.
    - List of agents, where agents are tuples (q, cap), q is the initial state of
    the agent, and cap is the set of capabilities. Agents' identifiers are their
    indices in the list.
    - Dictionary of capability encoding that maps capabilities to binary words
    represented as integers.
    Output
    ------
    Dictionary from states to distribution of agents from each class. The
    distribution is a list of length equal to the number of capabilities, and
    each element is the number of agents of having those capabilities (a class).
    '''
    #Austin, I didnt get to fixing this but the problem is the way he is referencing
    #the indexes of the capabilities. I am not sure how to reference these correctly
    #without creating all of the capabilities variables.
    nc = len(capabilities)
    capability_distribution = {u: [0]*2**nc for u in ts.g}
    #print(capability_distribution)
    for state, g in agents:
        #print('cap: ',capabilities,'g: ',g)
        g_enc = sum([capabilities[c] for c in g])
        #print(g_enc, state)
        capability_distribution[state][g_enc] += 1
    '''
    nc = len(capabilities)
    capability_distribution = {u: [0]*(2**nc) for u in ts.g}
    for state, cap in agents:
        g = sum([capabilities[c] for c in cap])
        capability_distribution[state][g] += 1

    print(capability_distribution)
    '''
    return capability_distribution


def computeRobustnessUpperBound(ts,agents,formula):
    '''
    Computes a loose upper bound on the robustness value.

    The robustness value measures whether there are enough agents to satisfy the
    temporal logic specification in the given transition system. A positive
    robustness value indicates there are more than enough required agents to
    satisfy the formula. A negative robustness value indicates that there
    are not enough required agents to satisfy the formula.

    This function computes an upper bound on the robustness value for a given
    transition system (TS), set of agents, and CaTL formula. This upper bound can
    be used as a "sanity check" to test if the formula is infeasible for the
    given TS and set of agents. If the upper bound is negative, then the
    problem is infeasible. If the upper bound is positive, then it is *possible*
    that the problem is feasible.

    If the upper bound is negative, there is no point in solving the associated
    MILP. The problem is infeasible.

    **Note:** This function does *not* take into account the travel time between
    regions in the TS.

    Args:
        ts:                 The transition system
        agents (list):      List of agents in the system. Each agent has the form ``(initial_state, {capability1, capability2,...})``.
        formula:            Either a string or a catl.CATLFormula object containing the formula. 
                            If a string, it is converted into a catl.CATLFormula object by
                            calling ``formula = CATLFormula.from_formula(formula)``.

    Returns:
        (float):    Upper bound on the robustness value.
    '''

    # If formula is a string, convert to CATLFormula object
    if isinstance(formula, str):
        formula = CATLFormula.from_formula(formula)

    #Creates a dict that tells how many of each capability are available in the full team
    # TODO determine if formula.capabilities() is too expensive to call here (it's recursive)
    #      had issue where there might be a capability in the formula not represented by an agent
    capDict ={cap: sum([int(cap in agent[1]) for agent in agents]) for cap in formula.capabilities()}
    ub =  get_ub(formula,ts,capDict)
    return ub



def get_ub(formula,ts,capDict):
    '''Uses recursive relationships to compute the excess capacity of the formula with respect to the team'''
    if formula.op == Operation.BOOL:
        return 0
    elif formula.op == Operation.PRED:
        ub = max(capDict.values())
        prop = formula.proposition
        numRegions = sum([prop in ud['prop'] for u, ud in ts.g.nodes(data=True)])
        for cr in formula.capability_requests:
            ub = min(ub,capDict[cr[0]]-numRegions*cr[1])
        return ub
    elif formula.op == Operation.AND :
        return min([get_ub(ch,ts,capDict) for ch in formula.children])
    elif formula.op == Operation.UNTIL:
        return min(get_ub(formula.left,ts,capDict), get_ub(formula.right,ts,capDict))
    elif formula.op == Operation.OR:
        return max([get_ub(ch,ts,capDict) for ch in formula.children])
    elif formula.op in (Operation.ALWAYS, Operation.EVENT):
        return get_ub(formula.child,ts,capDict)


def constrain_grave_state_pulp(mpulp,ast,agent_classes,grave_region):
   
    for k in range(0,int(ast.bound())+1):
        for g in agent_classes.items():
            name = 'z_' + str(grave_region) + '_' + str(g[1]) + '_' + str(k)
            # name_val = m.getVarByName(name)
            # m.addConstr(name_val == 0, 'grave_reject_{}_{}_{}'.format(str(grave_region),g[1],k))
            name_val = mpulp.variablesDict()[name]
            mpulp += name_val == 0, 'grave_reject_{}_{}_{}'.format(str(grave_region),g[1],k)
    


def create_system_variables_pulp(mpulp, ts, agent_classes, bound, vtype=pulp.LpInteger,num_agents=1000,regularize=False,alpha=0.5):
    '''Creates the state and transition variables associated with the given
    transition system using the PuLP framework.

    The state variables are z_{state}_{cap}_k, where {state} is a node in the TS
    graph, {cap} is a capability class encoded as an integer, and k is the time
    step.

    The transition variables are z_{state1}_{state2}_{cap}_k, where {state1} and
    {state2} define the transition, {cap} is a capability class encoded as an
    integer, and k is the time step.

    Input
    -----
    - The PuLP model object.
    - The transition system specifying the environment.
    - The agent classes given as a dictionary from frozen sets of capabilities
    to bitmaps (integers).
    - Time bound.
    - Variable type (default: integer).

    Note
    ----
    Data structure holding the variables is a list of list of variables, e.g.,

        d['vars'][k][g] is the z_{q/e}_bitmap(g)_k

    where d is the dictionary of attributes for a node q or an edge e in the TS,
    g is an agent class (frozen set of capabilities), bitmap(g) is the binary
    encoding of g as an integer, and k is the time step.
    Also, d['vars'] is a list of length `bound+1', d['vars'][k] is a dictionary
    from frozen sets to gurobi variables.
    '''
    # node variables
    #print(ts.g.nodes(data=True))

    newObj = pulp.LpAffineExpression() #Create objective function for total travel time
    for u, d in ts.g.nodes(data=True):
        d['vars'] = [] # initialize node variables list
        for k in range(bound+1):
            name = 'z_' + str(u) + '_{}_' + str(k)

            # Use PuLP variables instead of Gurobi variables.
            d['vars'].append({g: pulp.LpVariable(name.format(enc), lowBound=0, upBound=num_agents, cat=vtype)
                           for g, enc in agent_classes.items()})

    # edge variables
    
    for u, v, d in ts.g.edges(data=True):
        d['vars'] = [] # initialize edge variables list
        for k in range(-1,bound+1):
            if k == -1:
                # Use 'n1' in the name instead of '-1'
                # This avoids naming issues in PuLP. PuLP simply converts `-` symbols
                # into underscores, which creates problems in the read_sol_data function.
                name = 'z_' + str(u) + '_' + str(v) + '_{}_n1'
            else:
                name = 'z_' + str(u) + '_' + str(v) + '_{}_' + str(k)
            edgeList = {g: pulp.LpVariable(name.format(enc), cat=vtype, lowBound=0, upBound=2*num_agents)
                            for g, enc in agent_classes.items()}
            d['vars'].append(edgeList)
            if str(u) != str(v):
                for edge in edgeList.values():
                    newObj += alpha/(bound*num_agents)*edge*d['weight'] #This is the weighted total travel time
    normTravelTime = pulp.LpVariable('normTravelTime', cat=pulp.LpContinuous, lowBound=0.0, upBound=1.0)
    mpulp += normTravelTime == newObj, "travelTimeEquality"

    if regularize:
        # mpulp.setObjectiveN(alpha*newObj,mpulp.NumObj)
        mpulp += alpha*newObj, "Objective Function"



def extract_propositions(ts, ast):
    '''
    Returns the set of propositions in the formula, and checks that it is
    included in the transitions system.

    Input
    -----
    - The transition system specifying the environment.
    - The AST of the CaTL specification formula.

    Output
    ------
    Set of propositions in the specification formula.
    '''
    #print [d['prop'] for _, d in ts.g.nodes(data=True)]
    ts_propositions = set.union(*[d['prop'] for _, d in ts.g.nodes(data=True)])
    formula_propositions = set(ast.propositions())
    assert formula_propositions <= ts_propositions, \
                                'There are unknown propositions in the formula!'
    return formula_propositions




def route_planning(inputs,ts, agents, formula, 
                    bound=None,
                    file_name=None,
                    replan_req=False,
                    robust=False,
                    regularize=False,
                    alpha=0.5,
                    upperBound=False,
                    load_previous=False,
                    solver=None,
                    compute_IIS=True,
                    verbose=True,
                    compress_files=True,
                    solver_time_limit=None,
                    solver_threads=None):

    '''
    Performs route planning for agents ``agents`` moving in a transition system
    ``ts`` such that the CaTL specification ``formula`` is satisfied.

    Args:
        inputs:
        ts:
        agents:
        formula (str):
        bound:
        file_name (str):
        replan_req (bool):
        robust (bool):
        regularize (bool):
        alpha (float):
        upperBound (bool):
        load_previous:
        solver_name (str):
        compute_IIS (bool):
        verbose:
        compress_files (bool):
        solver_time_limit (float): Time limit on the MILP solution process. The MILP will stop after
                                    this amount of time. If no time limit is specified, the MILP will 
                                    run to completion.

    Returns:
        (tuple): Tuple containing the following elements:

            - **mpulp**: PuLP LpProblem object containing the (solved) MILP corresponding to the planning problem
            - **replan_data**: Object containing the following fields:
                - TODO: [INSERT HERE]

    Input
    -----
    - The transition system specifying the environment.
    - List of agents, where agents are tuples (q, cap), q is the initial state
    of the agent, and cap is the set of capabilities. Agents' identifiers are
    their indices in the list.
    - The CaTL specification formula.
    - The time bound used in the encoding (default: computed from CaTL formula).

    Output
    ------
    TODO: TBD
    '''
    # print formula
    ast = CATLFormula.from_formula(formula)
    # print ast
    if bound is None:
        bound = int(ast.bound())
    # create MILP
    if inputs.load_old_files:
        # TODO: Create method to load files using PuLP     
        print("\nWARNING: Loading old files not implemented.\n")

    mpulp = pulp.LpProblem("iitchs_prob", pulp.LpMinimize)

    # create agent system variables
    capabilities = compute_capability_bitmap(agents)
    agent_classes = compute_agent_classes(agents, capabilities)
    agent_capability_map = []
    for ag in agents:
        agent_capability_map.append([agent_classes[k] for k in agent_classes.keys() if k==ag[1]][0])
    
    # create system variables
    # create_system_variables(m, ts, agent_classes, bound,num_agents=len(agents),regularize=regularize,alpha=alpha)
    create_system_variables_pulp(mpulp, ts, agent_classes, bound,num_agents=len(agents),regularize=regularize,alpha=alpha)
    

    # add system constraints
    capability_distribution = compute_initial_capability_distribution(ts,
                                                       agents, capabilities)


    # add_system_constraints(m, ts, agent_classes, capability_distribution, bound)
    add_system_constraints_pulp(mpulp, ts, agent_classes, capability_distribution, bound)


    # add CMTL formula constraints
    stl = catl2stl(ast)
    ranges = {variable: (0, len(agents)) for variable in stl.variables()}
    #print stl
    if upperBound:
        #Compute and add a loose upperbound to the robustness value
        ub = computeRobustnessUpperBound(ts,agents,ast)
        #print "Upper Bound ", ub
        ranges['rho'] = (0, ub)
    else:
        ranges['rho'] = (0, 20-1) # TODO: kind of a hack
    # stl_milp = stl2milp(stl, ranges=ranges, model=m, robust=robust)
    stl_milp = stl2milp_pulp(stl, ranges=ranges, model=mpulp, robust=robust, verbose=verbose)

    stl_milp.M = 20 # set big M
    stl_milp.translate()


    # add proposition constraints
    # add_proposition_constraints(m, stl_milp, ts, ast, capabilities,
    #                         agent_classes, bound)
    add_proposition_constraints_pulp(mpulp, stl_milp, ts, ast, capabilities, agent_classes, bound)

    # constrain_grave_state(m,ast,agent_classes,inputs.replan_grave)
    if hasattr(inputs, 'replan_grave'):
        constrain_grave_state_pulp(mpulp,ast,agent_classes,inputs.replan_grave)

    if inputs.load_old_files:
        print("\nWARNING: loading old files not implemented.\n")
        pass
        # m = gurobipy.read(reload_file+'.lp')
        # m.read(reload_file+'.sol')
        # if hasattr(inputs,'timeLimit'):
        #     old_m.params.timeLimit = inputs.timeLimit
        # old_m.optimize()
    else:
        # run optimizer
        if hasattr(inputs,'timeLimit') and solver_time_limit is None:
            # The variable solver_time_limit takes precedence over inputs.timeLimit
            solver_time_limit = inputs.timeLimit

        # TODO: Add multithreading capability to SCIP and CBC.
        #       Currently this only works for GUROBI.


        # # Set up the solver type and time limit
        if solver is None:
            # Use the default solver. Currently SCIP.
            solver_object = pulp.SCIP()
            if solver_time_limit is not None:
                solver_object.timeLimit = solver_time_limit
        elif solver.upper() == 'GUROBI':
            # GUROBI uses the Gurobipy interface.
            #
            # Options can be set by passing in keyword arguments equivalent to Gurobi parameter
            # names to the constructor.
            #
            # Example:
            #   solver = pulp.GUROBI(
            #       OutputFlag=0,
            #       TimeLimit=600,
            #       Threads=1
            #   )
            options = {}
            if solver_threads is not None:
                options['Threads'] = solver_threads
            
            if not verbose:
                options['OutputFlag'] = 0

            if solver_time_limit is not None: 
                options['TimeLimit'] = solver_time_limit

            solver_object = pulp.GUROBI(**options)
        elif solver.upper() == 'GUROBI_CMD':
            # GUROBI_CMD calls the Gurobi command line solver.
            # Options can be set for the solver via pulp by passing a list of tuples to the 
            # "options" keyword argument when creating the solver. For each tuple, the first
            # entry must be the string name of the option, and the second entry must be the
            # value that option should have. The option names are the same as those in the 
            # Gurobi documentation.
            #
            # Example:
            #   solver = pulp.GUROBI_CMD(options=[
            #       ("OutputFlag", 0),
            #       ("TimeLimit", 600)
            #   ])

            # Set verbosity
            if verbose:
                print("\nNOTE: Do not use GUROBI_CMD solver with the iitchs_base benchmarking code.\nIt is impossible to distinguish between infeasible and timed-out solutions.\n")
                output_flag = 1
            else:
                output_flag = 0

            # Set threads
            if solver_threads is None:
                solver_threads = 0 # Gurobi default value; uses as many as necessary

            # Set solver_time_limit
            if solver_time_limit is not None:
                solver_object = pulp.GUROBI_CMD(threads=solver_threads, options=[("TimeLimit",solver_time_limit), ("OutputFlag", output_flag)])
            else:
                solver_object = pulp.GUROBI_CMD(threads=solver_threads, options=[("OutputFlag", output_flag)])
        elif solver.upper() == 'SCIP' or solver.upper() == 'SCIP_CMD':
            solver_object = pulp.SCIP()
            if solver_time_limit is not None:
                solver_object.timeLimit = solver_time_limit
        elif solver.upper() == 'COIN_CMD':
            solver_object = pulp.COIN_CMD()
            if solver_time_limit is not None:
                solver_object.timeLimit = solver_time_limit
        elif solver.upper() == 'PULP_CBC_CMD':
            solver_object = pulp.PULP_CBC_CMD()
            if solver_time_limit is not None:
                solver_object.timeLimit = solver_time_limit
        else:
            logging.error("\n Error: Solver " + solver+ " is not currently supported.\n")

        # # Set the solving function
        if verbose:
            solve_function = mpulp.solve
        else:
            solve_function = suppress_output(mpulp.solve)

        # # Solve the problem.
        # # This is equivalent to mpulp.solve(solver=pulp.SOLVER_NAME_HERE())
        solve_function(solver=solver_object)


        if mpulp.status == pulp.LpStatusOptimal:
            if mpulp.objective is not None:
                logging.info('"Optimal objective LP": %f', pulp.value(mpulp.objective))
        elif mpulp.status == pulp.LpStatusNotSolved:
            logging.error('Model was not solved.')
        elif mpulp.status == pulp.LpStatusInfeasible:
            logging.error('Model is infeasible.')

            if mpulp.solver.name == 'GUROBI' and compute_IIS:
                # Compute the infeasible set, if using Gurobi
                mpulp.solverModel.computeIIS()
                mpulp.solverModel.write("infeasible_set.ilp")

        elif mpulp.status == pulp.LpStatusUnbounded:
            logging.error('Model is unbounded.')
        else:
            logging.error("Model is undefined.")
            
    if inputs.load_old_files == False:
        # TODO: This isn't currently doing anything...file_name is the name of a directory, and so file_name+".mps" is invalid.
        mpulp.writeMPS(file_name+".mps")
        mpulp.writeLP(file_name+".lp")
        if mpulp.solver.name == 'GUROBI':
            try:
                mpulp.solverModel.write(file_name+".sol")
            except:
                if verbose:
                    print("Error: unable to write the model.")
        else:
            write_sol(mpulp, file_name=file_name+".sol")

        # Compress the output files, which can be fairly large (~8-45 MB)
        if compress_files:
            lp_name = f"{file_name}.lp"
            mps_name = f"{file_name}.mps"
            sol_name = f"{file_name}.sol"

            with open(lp_name, 'rb') as lp_file, open(mps_name, 'rb') as mps_file, open(sol_name, 'rb') as sol_file:
                with gzip.open(f"{lp_name}.gz", 'wb') as lp_zip, gzip.open(f"{mps_name}.gz", 'wb') as mps_zip, gzip.open(f"{sol_name}.gz", 'wb') as sol_zip:
                    shutil.copyfileobj(lp_file, lp_zip)
                    shutil.copyfileobj(mps_file, mps_zip)
                    shutil.copyfileobj(sol_file, sol_zip)
            
            # Delete the original files
            if os.path.isfile(lp_name):
                os.remove(lp_name)
            else:
                print("\nERROR: Removing lp file did not work.\n")

            if os.path.isfile(mps_name):
                os.remove(mps_name)
            else:
                print("\nERROR: Removing mps file did not work.\n")

            if os.path.isfile(sol_name):
                os.remove(sol_name)
            else:
                print("\nERROR: Removing sol file did not work.\n")


    if replan_req:
        # TODO: Update this to PuLP
        #print("stuck here")
        replan_time = inputs.replan_time
        replan_region = inputs.replan_region
        replan_grave = inputs.replan_grave
        replan_num = inputs.replan_num
        replan_cap = inputs.replan_cap
        print("\nWARNING: replan_route not currently implemented.\n")
        # replan_route(m,agents,ts,ast,capabilities,agent_classes,stl_milp,replan_region,replan_cap,replan_time,replan_num,replan_grave)

    if mpulp.status == pulp.LpStatusOptimal:
        if mpulp.objective is not None:
            logging.info('"Optimal objective LP": %f', pulp.value(mpulp.objective))
    elif mpulp.status == pulp.LpStatusNotSolved:
        logging.error('Model was not solved.')
    elif mpulp.status == pulp.LpStatusInfeasible:
        logging.error('Model is infeasible.')
        if mpulp.solver.name == 'GUROBI' and compute_IIS:
            # Compute the infeasible set, if using Gurobi
            # TODO: Include GUROBI_CMD?
            mpulp.solverModel.computeIIS()
            mpulp.solverModel.write("infeasible_set.ilp")
    elif mpulp.status == pulp.LpStatusUnbounded:
        logging.error('Model is unbounded.')
    else:
        logging.error("Model is undefined.")
    if replan_req:
        mpulp.writeMPS(inputs.save_filename+".mps")
        mpulp.writeLP(inputs.save_filename+".lp")
        if mpulp.solver.name == 'GUROBI' or mpulp.solver.name == 'GUROBI_CMD':
            mpulp.solverModel.write(inputs.save_filename+".sol")
        else:
            write_sol(mpulp, file_name=inputs.save_filename+".sol")




    replan_data = empty_struct()
    replan_data.ast = ast
    replan_data.agent_classes = agent_classes
    replan_data.ts = ts
    replan_data.capabilities = capabilities
    replan_data.stl_milp = stl_milp
    replan_data.agents = agents
    replan_data.agent_capability_map = agent_capability_map

    return mpulp, replan_data






def generate_MILP_problems(ts, agents, formula, 
                    bound=None,
                    file_name=None,
                    robust=False,
                    regularize=False,
                    alpha=0.5,
                    upperBound=False,
                    replan_grave=None,
                    verbose=True,
                    compress_files=True):

    '''
    Similar to route_planning(), but simply generates the MILP .lp / .mps problem files
    without actually solving them.

    Args:
        inputs:
        ts:
        agents:
        formula (str):
        bound:
        file_name (str):
        replan_req (bool):
        robust (bool):
        regularize (bool):
        alpha (float):
        upperBound (bool):
        load_previous:
        solver_name (str):
        compute_IIS (bool):
        verbose:
        compress_files (bool):
        solver_time_limit (float): Time limit on the MILP solution process. The MILP will stop after
                                    this amount of time. If no time limit is specified, the MILP will 
                                    run to completion.

    Returns:
        (tuple): Tuple containing the following elements:

            - **mpulp**: PuLP LpProblem object containing the (solved) MILP corresponding to the planning problem
            - **replan_data**: Object containing the following fields:
                - TODO: [INSERT HERE]

    Input
    -----
    - The transition system specifying the environment.
    - List of agents, where agents are tuples (q, cap), q is the initial state
    of the agent, and cap is the set of capabilities. Agents' identifiers are
    their indices in the list.
    - The CaTL specification formula.
    - The time bound used in the encoding (default: computed from CaTL formula).

    Output
    ------
    TODO: TBD
    '''
    # print formula
    ast = CATLFormula.from_formula(formula)
    # print ast
    if bound is None:
        bound = int(ast.bound())

    # create MILP
    mpulp = pulp.LpProblem("iitchs_prob", pulp.LpMinimize)

    # create agent system variables
    capabilities = compute_capability_bitmap(agents)
    agent_classes = compute_agent_classes(agents, capabilities)
    agent_capability_map = []
    for ag in agents:
        agent_capability_map.append([agent_classes[k] for k in agent_classes.keys() if k==ag[1]][0])
    
    # create system variables
    # create_system_variables(m, ts, agent_classes, bound,num_agents=len(agents),regularize=regularize,alpha=alpha)
    create_system_variables_pulp(mpulp, ts, agent_classes, bound,num_agents=len(agents),regularize=regularize,alpha=alpha)
    

    # add system constraints
    capability_distribution = compute_initial_capability_distribution(ts,
                                                       agents, capabilities)


    # add_system_constraints(m, ts, agent_classes, capability_distribution, bound)
    add_system_constraints_pulp(mpulp, ts, agent_classes, capability_distribution, bound)


    # add CMTL formula constraints
    stl = catl2stl(ast)
    ranges = {variable: (0, len(agents)) for variable in stl.variables()}
    #print stl
    if upperBound:
        #Compute and add a loose upperbound to the robustness value
        ub = computeRobustnessUpperBound(ts,agents,ast)
        #print "Upper Bound ", ub
        ranges['rho'] = (0, ub)
    else:
        ranges['rho'] = (0, 20-1) # TODO: kind of a hack
    # stl_milp = stl2milp(stl, ranges=ranges, model=m, robust=robust)
    stl_milp = stl2milp_pulp(stl, ranges=ranges, model=mpulp, robust=robust, verbose=verbose)

    stl_milp.M = 20 # set big M
    stl_milp.translate()


    # add proposition constraints
    # add_proposition_constraints(m, stl_milp, ts, ast, capabilities,
    #                         agent_classes, bound)
    add_proposition_constraints_pulp(mpulp, stl_milp, ts, ast, capabilities, agent_classes, bound)

    # constrain_grave_state(m,ast,agent_classes,inputs.replan_grave)
    if replan_grave is not None:
        constrain_grave_state_pulp(mpulp,ast,agent_classes,replan_grave)


    # Save the problem files
    lp_name = f"{file_name}.lp"
    mps_name = f"{file_name}.mps"

    mpulp.writeLP(lp_name)
    mpulp.writeMPS(mps_name)

    # Compress the output files, which can be fairly large (~8-45 MB)
    if compress_files:
        with open(lp_name, 'rb') as lp_file, open(mps_name, 'rb') as mps_file:
            with gzip.open(f"{lp_name}.gz", 'wb') as lp_zip, gzip.open(f"{mps_name}.gz", 'wb') as mps_zip:
                shutil.copyfileobj(lp_file, lp_zip)
                shutil.copyfileobj(mps_file, mps_zip)
        
        # Delete the original files
        if os.path.isfile(lp_name):
            os.remove(lp_name)
        else:
            print("\nERROR: Removing lp file did not work.\n")

        if os.path.isfile(mps_name):
            os.remove(mps_name)
        else:
            print("\nERROR: Removing mps file did not work.\n")
        
        return f"{lp_name}.gz", f"{mps_name}.gz"
    
    return lp_name, mps_name
