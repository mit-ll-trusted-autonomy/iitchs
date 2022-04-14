'''
IITCHS main function

Entry point for the IITCHS code.
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

# SPDX-License-Identifier: BSD-3-Clause

import pickle
import argparse
from types import SimpleNamespace
import pandas as pd
import importlib
from os import path
from copy import Error, deepcopy

# For creating Output File Directories, if they don't exist.
from pathlib import Path

# For debugging purposes
import sys
if sys.flags.debug:
    import pdb


from catl_planning.route_planning import route_planning, compute_agent_classes, compute_capability_bitmap
from catl_planning.traj_planner import get_agent_capabilities, get_agent_map, read_sol_data, run_planner, run_planner_cpp
from catl_planning.visualization import show_environment
from catl_planning.pure_pursuit import sim_dynamics
from catl_planning.kml_gen import gen_kml_file
from catl_planning.kml_gen import gen_replan_kml_file
from catl_planning.kml_gen import txt_output as txt_file_gen
from catl_planning.load_sol import load_sol

try:
    from catl_planning.decomposition_functions import decompose
    CAN_DECOMPOSE = True
except IOError as ioe:
    # can happen if Z3 bindings not available
    CAN_DECOMPOSE = False

from catl_planning.reduced_ts import reduce_ts

from lomap.classes.ts import Ts

from yaml_parsing import get_state_dictionary

from pulp.constants import LpStatusNotSolved, LpSolutionInfeasible


class empty_struct:
    pass


def run_cs_run(m, args):
    """
    Runs an IITCHS routine which generates capability schedules, assigns agents to capability schedules, 
    then performs low-level trajectory planning for agents to satisfy their capability schedules. 
    A plot of the agents' final low-level trajectories is displayed.

    Args:
        m: The casefile module being run.
        args: An object returned from calling ``parse_args()`` on an argparse object. The attribute 
            ``args.solver`` can be set to ``SCIP``, ``GUROBI``, or ``GUROBI_CMD`` to use either the SCIP or GUROBI solver, respectively.
    """

    m.replan_req = False
    ts = Ts.load(m.ts_filename)
    for u, v in ts.g.edges():
        assert 'grave' in ts.g.node[v]['prop'] or ts.g.has_edge(v,u)


    if m.show_visuals:
        show_environment(ts)

    if args.time_limit is not None:
        time_limit = float(args.time_limit)
    else:
        time_limit = None

    #Run the optimization
    GurModel,replan_data = route_planning(m,ts, m.agents, m.formula, bound=None,file_name=m.save_filename, replan_req=m.replan_req,robust=m.robust,load_previous=False,solver=args.solver,solver_time_limit=time_limit)

    #Load the solution from route_planning
    data = load_sol(f'{m.save_filename}.sol')

    #Run the trajectory planner
    the_plan,caps,past_plans = run_planner(m,ts,data)
    print(len(the_plan[0]))
    pickle.dump(the_plan,open(m.save_filename+".pkl","wb"))

    if m.sim_dynamics:
        dynamics_plan = sim_dynamics(m,the_plan)
        pickle.dump(the_plan,open(m.save_filename+"dynamics_plan.pkl","wb"))
        if m.kml_output:
            gen_kml_file(m,dynamics_plan)
        if m.txt_out:
            txt_file_gen(m,dynamics_plan)
    else:
        pickle.dump(the_plan,open(m.save_filename+"_the_plan.pkl","wb"))
        if m.kml_output:
            gen_kml_file(m,the_plan)
        if m.txt_out:
            txt_file_gen(m,the_plan)







def run_cs_decomp(m, args):
    """
    Runs an IITCHS routine using decomposition. This preprocesses the problem
    decomposing the specification into independent specifications, each with
    dedicated agents.

    Each individual problem is solved with the regular IITCHS approach and
    the results are combined for a global solution, followed by low-level
    trajectory planning for agents to satisfy their capability schedules.

    Plots the agent's final low-level trajectories.

    Args:
        m: The casefile module to run
        args: An object returned from collaing ``parse_args()`` on an argparse
              object. The attribute ``args.solver`` can be set to ``SCIP``,
              ``GUROBI``, or ``GUROBI_CMD``.
    """

    if not CAN_DECOMPOSE:
        raise RuntimeError('Missing dependency for decomposition. '
                           'Check that z3 Python bindings are available')

    m.replan_req = False
    ts = Ts.load(m.ts_filename)

    if m.show_visuals:
        show_environment(ts)

    if args.time_limit is not None:
        time_limit = float(args.time_limit)
    else:
        time_limit = None

    formula_decomp, _ = decompose(m.formula, m.agents, ts)
    print(f'Decomposed into {len(formula_decomp)} subteams')

    # Call decomposition of spec into subteams
    sub_ts_list = [] # list of modified TSs if reduce_ts is enabled, 
                     # otherwise just a list of the original ts
    for i, x in enumerate(formula_decomp.keys()):
        #Run the optimization

        subteam = [m.agents[int(y)] for y in formula_decomp[x]['agents']]
        spec    = formula_decomp[x]['formula']
        print('subteam: ',x,subteam)
        print('subteam length: ',len(subteam))
        print('spec: ',x,spec)
        if args.reduce_ts:
            sub_ts = reduce_ts(ts, spec, subteam)
        else:
            sub_ts = ts
        sub_ts_list.append(sub_ts)
        PulpModel,replan_data = route_planning(m, sub_ts, subteam, spec,
                bound=None, file_name=m.save_filename+'_'+str(x),
                replan_req=False,robust=m.robust,load_previous=False,
                solver=args.solver, solver_time_limit=time_limit)

        if PulpModel.sol_status == LpSolutionInfeasible:
            sys.exit('Decomposition resulted in an infeasible subteam:'
                     ' giving up.')

        if time_limit is not None:
            # treating time limit as a global time limit, not per-subteam
            time_limit -= PulpModel.solutionTime
            if time_limit <= 0 or \
                PulpModel.status == LpStatusNotSolved:
                # if not solved, assume this was because of the time limit
                # Note: this status is distinct from it being infeasible
                # PuLP does not correctly set the solutionTime if there
                # is an interrupt due to a timeout, so we cannot rely
                # on updating the time_limit with solutionTime
                sys.exit(f'Stopping subteam solving (at subteam {i}): '
                          'time limit was reached.')

    # loop over the k-1 solutions
    assert len(formula_decomp) == len(sub_ts_list)
    for x, sub_ts in zip(formula_decomp.keys(), sub_ts_list):
        #Load the solution from route_planning
        data = load_sol(f'{m.save_filename}_{x}.sol')

        #Run the trajectory planner
        the_plan,_,_ = run_planner(m,sub_ts,data)

        pickle.dump(the_plan,open(m.save_filename+'_'+str(x)+".pkl","wb"))

        if m.sim_dynamics:
            dynamics_plan = sim_dynamics(m,the_plan)
            pickle.dump(the_plan,open(m.save_filename+'_'+str(x)+"dynamics_plan.pkl","wb"))
            if m.kml_output:
                gen_kml_file(m,dynamics_plan)
            if m.txt_out:
                txt_file_gen(m,dynamics_plan)
        else:
            if m.kml_output:
                gen_kml_file(m,the_plan)
            if m.txt_out:
                txt_file_gen(m,the_plan)


def run_cs_run_cpp(m, solver=None, iitchs_path=None, time_limit=None):
    """
    Runs an IITCHS routine which generates capability schedules and assigns agents to capability schedules.
    This function is designed to be easily called from C++.
    Differences from ``run_cs_run()`` include:

        - No low-level trajectory planning performed.
        - No plots are generated.

    Args:
        m: The casefile module being run.
        solver (string): (Optional). The name of the solver to use. May be one of 'SCIP', 'GUROBI', or 'GUROBI_CMD'.
        iitchs_path (string): (Optional). Full path to the iitchs_base folder.
    
    Returns:
        (tuple): Tuple containing:

            - **output_strings** *(list)*: List containing each agent's schedule in string form. The iith entry of the list
              contains the string for the iith agent's schedule. Format of the string 
              is ``STATE0-STATE1:t0-t1; STATE1-STATE2:t1-t2; ...``. If ``STATEii == STATEjj``, then the agent remains
              within ``STATEii`` during  the specified time interval. If ``STATEii != STATEjj``, then the agent traverses
              the edge from ``STATEii`` to ``STATEjj`` during the specified time interval.
              Examples include ``q1-q1:0-10`` or ``q7-q2:50-75``.
            - **PuLP_Model** *(PuLP LpProblem object)*: Object containing the PuLP MILP problem that was solved to obtain the
              capability schedule.
            - **replan_data** *(object)*: Python object containing the data for replanning.
            - **path_args** *(object)*: Python object containing the paths to the transition system file and output save path.
            - **agent_positions**: List containing the locations (states/edges) of capabilities at all timesteps. The iith
                entry corresponds to a list containing position information at time ii. The list in entry ii contains position
                information for all capability classes represented by the agents. **NOTE:** The order of entries in list ii
                does **not** correspond to the order of agents in the casefile. To find the entry that an agent jj maps to in
                the iith list, use the agent_index_dict output (described below).
            - **agent_tasks**: TODO
            - **agent_index_dict***: Dictionary mapping agent number to the corresponding column (i.e. second index) number in
                the ``agent_positions`` list.
    """

    # If an absolute path to the iitchs_MOOS folder is given,
    # change the relative directories in case1.py to use the
    # new path. Useful when running the MOOS app in a different
    # directory than the iitchs_MOOS folder.
    path_args = empty_struct
    if iitchs_path is not None:
        assert path.exists(iitchs_path), "Error: path " + iitchs_path + " does not exist. Change your iitchs_path input for run_cs_run_cpp."

        catl_path = iitchs_path + '/src'
        path_args.ts_filename = catl_path + m.ts_filename[1:] # Removes the leading '.'
        path_args.save_filename = catl_path + m.save_filename[1:]
        path_args.output_path = catl_path + m.output_path[1:]

        assert path.exists(path_args.ts_filename), "Error: path_args.ts_filename path " + path_args.ts_filename + " does not exist."

        # Create the output_path, if it doesn't already exist.
        Path(path_args.output_path).mkdir(parents=True, exist_ok=True)
        assert path.exists(path_args.output_path), "Error, path_args.output_path " + path_args.output_path + " does not exist."
    else:
        path_args.ts_filename = m.ts_filename
        path_args.save_filename = m.save_filename
        path_args.output_path = m.output_path


    m.replan_req = False
    ts = Ts.load(path_args.ts_filename)
    for u, v in ts.g.edges():
        assert 'grave' in ts.g.node[v]['prop'] or ts.g.has_edge(v,u)


    # if m.show_visuals:
        # show_environment(ts)

    #Run the optimization
    PuLP_Model,replan_data = route_planning(m,ts, m.agents, m.formula, bound=None,file_name=path_args.save_filename, replan_req=m.replan_req,robust=m.robust,load_previous=False,solver=solver,solver_time_limit=time_limit)

    # Load the solution from route_planning
    data = load_sol(f'{path_args.save_filename}.sol')

    #Run the trajectory planner
    agent_positions, output_strings, agent_tasks, agent_index_dict = run_planner_cpp(m,ts,data)


    return output_strings, PuLP_Model, replan_data, path_args, agent_positions, agent_tasks, agent_index_dict




def get_argparser():
    """
    A convenience function for building an argparse object. This should only be used internally.

    Returns:
        parser (argparse object): An argparse object used when calling ``main.py``.
    """
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    subparsers = parser.add_subparsers(dest='action')
    parser_run = subparsers.add_parser(
        'run', help='Run an example with no replanning or extras')
    parser_decomp = subparsers.add_parser(
        'decomp', help='Ran an example with specification decomposition')
    parser_run_cpp = subparsers.add_parser(
        'run_cpp', help='Run the "run" command using the function that MOOS C++ code calls.')
    
    # Argument to specify the MILP solver when using the 'run' option
    parser_run.add_argument('-s', '--solver', default='SCIP', help='Specify which solver PuLP should call. Current options are "SCIP", "GUROBI", "COIN_CMD", or "PULP_CBC_CMD".')
    parser_decomp.add_argument('-s', '--solver', default='SCIP', help='Specify which solver PuLP should call. Current options are "SCIP", "GUROBI", "COIN_CMD", or "PULP_CBC_CMD".')
    parser_run_cpp.add_argument('-s', '--solver', default='SCIP', help='Specify which solver PuLP should call. Current options are "SCIP" or "GUROBI".')


    # Argument to specify solver time limit
    parser_run.add_argument('-t', '--time_limit', default=None, help='Specify the solver time limit in seconds.')
    parser_decomp.add_argument('-t', '--time_limit', default=None, help='Specify the solver time limit in seconds.')
    parser_run_cpp.add_argument('-t', '--time_limit', default=None, help='Specify the solver time limit in seconds.')

    # decomp-specific options
    parser_decomp.add_argument('--reduce-ts', action='store_true', 
                               help='Remove unnecessary states from transition system '
                                    'in each decomposed subspec/subteam.')


    parser.add_argument('module', help='module containing the case study data')
    return parser


if __name__=="__main__":
    parser = get_argparser()
    args = parser.parse_args()
    module = importlib.import_module(args.module)

    if args.action == 'run':
        run_cs_run(module, args)

    elif args.action == 'decomp':
        run_cs_decomp(module, args)

    elif args.action == 'run_cpp':
        if args.time_limit is not None:
            time_limit = float(args.time_limit)
        else:
            time_limit = None

        output = run_cs_run_cpp(module, solver=args.solver, time_limit=time_limit)
        print("\n\nOutput:\n")
        print(output)

