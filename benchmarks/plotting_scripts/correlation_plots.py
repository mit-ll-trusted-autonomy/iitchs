'''
Plots a comparison of the mean / median solver times of Gurobi vs SCIP.

Also compares the difference between 'Robust' and 'Feasible' runtimes.

Authors: Hassaan Khan (hassaan.khan@ll.mit.edu), James Usevitch (james.usevitch@ll.mit.edu)
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
import sys
import pickle
import tarfile
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from argparse import ArgumentParser

if sys.flags.debug:
    import pdb


def robust_vs_feasible(data_fea, data_other, gurobi_name="GUROBI", scip_name="SCIP", show=True):
    gurobi_robust = data_fea.loc[(data_fea['MILP_Solver_Used'] == gurobi_name) & (data_fea['Robust'] == True)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == gurobi_name) & (data_other['Robust'] == True)]['Runtime_Seconds'].tolist()
    gurobi_feasible = data_fea.loc[(data_fea['MILP_Solver_Used'] == gurobi_name) & (data_fea['Robust'] == False)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == gurobi_name) & (data_other['Robust'] == False)]['Runtime_Seconds'].tolist()

    scip_robust = data_fea.loc[(data_fea['MILP_Solver_Used'] == scip_name) & (data_fea['Robust'] == True)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == 'SCIP') & (data_other['Robust'] == True)]['Runtime_Seconds'].tolist()
    scip_feasible = data_fea.loc[(data_fea['MILP_Solver_Used'] == scip_name) & (data_fea['Robust'] == False)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == 'SCIP') & (data_other['Robust'] == False)]['Runtime_Seconds'].tolist()
    
    box_data_robust = [gurobi_robust, scip_robust]
    box_data_feasible = [gurobi_feasible, scip_feasible]

    fig, ax = plt.subplots(1,2, sharey=True)
    plt.suptitle('IITCHS Planning - MILP Solve Times')

    labels = ['Gurobi', 'SCIP']
    x = np.arange(len(labels))

    ax[0].set_title('Robust Solution Solve Time')
    ax[0].set_ylabel('Solve Time (sec)')
    ax[0].boxplot(box_data_robust)
    ax[0].set_xticklabels(labels)


    ax[1].set_title('Feasible Solution Solve Time')
    ax[1].boxplot(box_data_feasible)
    ax[1].set_xticklabels(labels)

    if show:
        plt.show(block=False)


def num_states_vs_solve_time(data_fea, data_other, gurobi_name="GUROBI", scip_name="SCIP", show=True):
    num_states_list = [4*4, 4*5, 5*5, 5*6]

    box_data_scip_states = [
        data_fea.loc[(data_fea['MILP_Solver_Used'] == scip_name) & (data_fea['Grid_TS_Xdim'] == 4) & (data_fea['Grid_TS_Ydim'] == 4)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == scip_name) & (data_other['Grid_TS_Xdim'] == 4) & (data_other['Grid_TS_Ydim'] == 4)]['Runtime_Seconds'].tolist(),
        data_fea.loc[(data_fea['MILP_Solver_Used'] == scip_name) & (data_fea['Grid_TS_Xdim'] == 4) & (data_fea['Grid_TS_Ydim'] == 5)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == scip_name) & (data_other['Grid_TS_Xdim'] == 4) & (data_other['Grid_TS_Ydim'] == 5)]['Runtime_Seconds'].tolist(),
        data_fea.loc[(data_fea['MILP_Solver_Used'] == scip_name) & (data_fea['Grid_TS_Xdim'] == 5) & (data_fea['Grid_TS_Ydim'] == 5)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == scip_name) & (data_other['Grid_TS_Xdim'] == 5) & (data_other['Grid_TS_Ydim'] == 5)]['Runtime_Seconds'].tolist(),
        data_fea.loc[(data_fea['MILP_Solver_Used'] == scip_name) & (data_fea['Grid_TS_Xdim'] == 5) & (data_fea['Grid_TS_Ydim'] == 6)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == scip_name) & (data_other['Grid_TS_Xdim'] == 5) & (data_other['Grid_TS_Ydim'] == 6)]['Runtime_Seconds'].tolist(),
    ]

    box_data_gurobi_states = [
        data_fea.loc[(data_fea['MILP_Solver_Used'] == gurobi_name) & (data_fea['Grid_TS_Xdim'] == 4) & (data_fea['Grid_TS_Ydim'] == 4)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == gurobi_name) & (data_other['Grid_TS_Xdim'] == 4) & (data_other['Grid_TS_Ydim'] == 4)]['Runtime_Seconds'].tolist(),
        data_fea.loc[(data_fea['MILP_Solver_Used'] == gurobi_name) & (data_fea['Grid_TS_Xdim'] == 4) & (data_fea['Grid_TS_Ydim'] == 5)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == gurobi_name) & (data_other['Grid_TS_Xdim'] == 4) & (data_other['Grid_TS_Ydim'] == 5)]['Runtime_Seconds'].tolist(),
        data_fea.loc[(data_fea['MILP_Solver_Used'] == gurobi_name) & (data_fea['Grid_TS_Xdim'] == 5) & (data_fea['Grid_TS_Ydim'] == 5)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == gurobi_name) & (data_other['Grid_TS_Xdim'] == 5) & (data_other['Grid_TS_Ydim'] == 5)]['Runtime_Seconds'].tolist(),
        data_fea.loc[(data_fea['MILP_Solver_Used'] == gurobi_name) & (data_fea['Grid_TS_Xdim'] == 5) & (data_fea['Grid_TS_Ydim'] == 6)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == gurobi_name) & (data_other['Grid_TS_Xdim'] == 5) & (data_other['Grid_TS_Ydim'] == 6)]['Runtime_Seconds'].tolist(),
    ]
    # box_data_solver_states = {solver: [data_fea.loc[(data_fea['MILP_Solver_Used'] == gurobi_name) & (data_fea['Grid_TS_Xdim'] == 5) & (data_fea['Grid_TS_Ydim'] == 6)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == gurobi_name) & (data_other['Grid_TS_Xdim'] == 5) & (data_other['Grid_TS_Ydim'] == 6)]['Runtime_Seconds'].tolist() for form_len in lengths] for solver in solvers}

    fig, ax = plt.subplots(1,2, sharey=True)
    plt.suptitle('IITCHS Planning - Number of States vs. Solve Time')

    labels = [str(k) for k in num_states_list]

    ax[0].set_title('Gurobi')
    ax[0].set_ylabel('Solve Time (sec)')
    ax[0].set_xlabel('Number of States')
    ax[0].boxplot(box_data_gurobi_states)
    ax[0].set_xticklabels(labels)

    ax[1].set_title('SCIP')
    ax[1].boxplot(box_data_scip_states)
    ax[1].set_xticklabels(labels)
    ax[1].set_xlabel('Number of States')

    if show:
        plt.show(block=False)

def formula_len_vs_solve_time(data_fea, data_other, solvers, formula_name, show=True):
    lengths = sorted(data_fea[formula_name].unique())
    box_data_solver = {solver: [data_fea.loc[(data_fea['MILP_Solver_Used'] == solver) & (data_fea[formula_name] == form_len)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == solver) & (data_other[formula_name] == form_len)]['Runtime_Seconds'].tolist() for form_len in lengths] for solver in solvers}

    fig, ax = plt.subplots(1, len(solvers), sharey=True)
    plt.suptitle(f'IITCHS Planning - {formula_name} vs. Solve Time')
    labels = lengths

    ax[0].set_ylabel('Solve Time (sec)')
    for i, solver in enumerate(box_data_solver):
        ax[i].set_title(solver)
        ax[i].set_xlabel(f'{formula_name}')
        ax[i].boxplot(box_data_solver[solver])
        ax[i].set_xticklabels(labels)

    if show:
        plt.show(block=False)

def flatten(big_list):
    return [item for small_list in big_list for item in small_list]

def ast_bound_vs_solve_time(data_fea, data_other, solvers, show=True):
    # Set bin size to group AST values into
    # E.g. bin_size = 20 will group AST lengths 0-20 together, 20-40 together, etc
    bin_size = 20
    run_time = {solver: None for solver in solvers}
    unique_bounds = {solver: None for solver in solvers}
    ast_bounds = {solver: None for solver in solvers}

    for solver in solvers:
        unique_bounds[solver] = np.hstack((
            data_fea.loc[data_fea['MILP_Solver_Used'] == solver]['Formula_AST_Bound'].unique(),
            data_other.loc[data_other['MILP_Solver_Used'] == solver]['Formula_AST_Bound'].unique(),
            ))
        
        run_time[solver] = flatten([data_fea.loc[(data_fea['MILP_Solver_Used'] == solver) & (data_fea['Formula_AST_Bound'] == ast)]['Runtime_Seconds'].tolist() + data_other.loc[(data_fea['MILP_Solver_Used'] == solver) & (data_other['Formula_AST_Bound'] == ast)]['Runtime_Seconds'].tolist() for ast in unique_bounds[solver]])
        ast_bounds[solver] = flatten([data_fea.loc[(data_fea['MILP_Solver_Used'] == solver) & (data_fea['Formula_AST_Bound'] == ast)]['Formula_AST_Bound'].tolist() + data_other.loc[(data_fea['MILP_Solver_Used'] == solver) & (data_other['Formula_AST_Bound'] == ast)]['Formula_AST_Bound'].tolist() for ast in unique_bounds[solver]])
        
    # Max Bound
    max_bound = max(np.hstack([l for l in unique_bounds.values()]))

    # List of bins
    bins = np.arange(0, max_bound+bin_size, bin_size)
    binned_inds = {solver: np.digitize(ast_bounds[solver], bins) for solver in solvers}
    binned_data = {solver: sorted([data for data in zip(run_time[solver], binned_inds[solver])], key=lambda x: x[1]) for solver in solvers}
    binned_data = {solver: [[d[0] for d in binned_data[solver] if d[1] == ind] for ind in range(1,len(bins))] for solver in solvers}

    labels = [f"{bin-bin_size}-{bin}" for bin in bins[1::]]
    fig, ax = plt.subplots(1, len(solvers), sharey=True)
    plt.suptitle(f'IITCHS Planning - AST Bounds vs. Solve Time')
    ax[0].set_ylabel('Solve Time (sec)')
    for i, solver in enumerate(run_time):
        ax[i].set_title(solver)
        ax[i].set_xlabel('AST bounds')
        ax[i].boxplot(binned_data[solver])
        pdb.set_trace() if sys.flags.debug else None
        ax[i].set_xticklabels(labels)

    if show:
        plt.show(block=False)

def regularize_true_false(data_fea, data_other, solvers, show=True):

   data_solver = {solver: [data_fea.loc[(data_fea['MILP_Solver_Used'] == solver) & (data_fea["Regularize"] == option)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == solver) & (data_other["Regularize"] == option)]['Runtime_Seconds'].tolist() for option in [True, False]] for solver in solvers}

   labels = ["True", "False"]
   fig, ax = plt.subplots(1, len(solvers), sharey=True)
   plt.suptitle(f'IITCHS Planning - Regularize (Bool) vs. Solve Time')
   ax[0].set_ylabel('Solve Time (sec)')
   for i, solver in enumerate(data_solver):
        ax[i].set_title(solver)
        ax[i].set_xlabel('Regularize')
        ax[i].boxplot(data_solver[solver])
        ax[i].set_xticklabels(labels)

   if show:                                                                                                                                                                                                                                              
       plt.show(block=False)  

def alpha(data_fea, data_other, solvers, show=True): 
    data = {solver: data_fea.loc[(data_fea['MILP_Solver_Used'] == solver)][['Runtime_Seconds', 'Alpha']] for solver in solvers}
    data_other = {solver: data_other.loc[(data_other['MILP_Solver_Used'] == solver)][['Runtime_Seconds', 'Alpha']] for solver in solvers}
    data = {solver: pd.concat([data[solver], data_other[solver]], ignore_index=True) for solver in solvers}
    alpha = np.unique(data[solvers[0]]['Alpha'])[0]

    labels = [str(alpha)]
    fig, ax = plt.subplots(1, len(solvers), sharey=True)
    plt.suptitle(f'IITCHS Planning - Alpha {alpha} vs. Solve Time')
    ax[0].set_ylabel('Solve Time (sec)')
    for i, solver in enumerate(data):
        ax[i].set_title(solver)
        ax[i].set_xlabel('Alpha')
        ax[i].boxplot(data[solver]['Runtime_Seconds'])
        ax[i].set_xticklabels(labels)

    if show:                                                                                                                                                                                                                                              
        plt.show(block=False)  

def upper_bound_true_false(data_fea, data_other, solvers, show=True):
   data_solver = {solver: [data_fea.loc[(data_fea['MILP_Solver_Used'] == solver) & (data_fea["Upper_Bound"] == option)]['Runtime_Seconds'].tolist() + data_other.loc[(data_other['MILP_Solver_Used'] == solver) & (data_other["Upper_Bound"] == option)]['Runtime_Seconds'].tolist() for option in [True, False]] for solver in solvers}

   labels = ["True", "False"]
   fig, ax = plt.subplots(1, len(solvers), sharey=True)
   plt.suptitle(f'IITCHS Planning - Upper Bound (Bool) vs. Solve Time')
   ax[0].set_ylabel('Solve Time (sec)')
   for i, solver in enumerate(data_solver):
        ax[i].set_title(solver)
        ax[i].set_xlabel('Upper Bound')
        ax[i].boxplot(data_solver[solver])
        ax[i].set_xticklabels(labels)

   if show:                                                                                                                                                                                                                                              
       plt.show(block=False)  

def milp_constraints(data_fea, data_other, solvers, show=True):
    bin_size = 20000
    unique_bounds = {solver: None for solver in solvers}
    ast_bounds = {solver: None for solver in solvers}

    data = {solver: data_fea.loc[(data_fea['MILP_Solver_Used'] == solver)][['Runtime_Seconds', 'Number_of_MILP_Constraints']] for solver in solvers}
    data_other = {solver: data_other.loc[(data_other['MILP_Solver_Used'] == solver)][['Runtime_Seconds', 'Number_of_MILP_Constraints']] for solver in solvers}
    data = {solver: pd.concat([data[solver], data_other[solver]], ignore_index=True) for solver in solvers }

    # Max Bound
    max_bound = max([max(data[solver]['Number_of_MILP_Constraints']) for solver in solvers])

    # List of bins
    bins = np.arange(0, max_bound+bin_size, bin_size)
    binned_inds = {solver: np.digitize(data[solver]['Number_of_MILP_Constraints'], bins) for solver in solvers}
    binned_data = {solver: sorted([data for data in zip(data[solver]['Runtime_Seconds'], binned_inds[solver])], key=lambda x: x[1]) for solver in solvers}
    binned_data = {solver: [[d[0] for d in binned_data[solver] if d[1] == ind] for ind in range(1,len(bins))] for solver in solvers}
    
    labels = [f"{(bin-bin_size)/1000}k-{bin/1000}k" for bin in bins[1::]]
    fig, ax = plt.subplots(1, len(solvers), sharey=True)
    plt.suptitle(f'IITCHS Planning - MILP Constraints vs. Solve Time')
    ax[0].set_ylabel('Solve Time (sec)')
    for i, solver in enumerate(binned_data):
        ax[i].set_title(solver)
        ax[i].set_xlabel('MILP Constraints')
        ax[i].boxplot(binned_data[solver])
        ax[i].set_xticklabels(labels)

    if show:
        plt.show(block=False)

def milp_variables(data_fea, data_other, solvers, show=True):
    bin_size = 50000
    unique_bounds = {solver: None for solver in solvers}
    ast_bounds = {solver: None for solver in solvers}

    data = {solver: data_fea.loc[(data_fea['MILP_Solver_Used'] == solver)][['Runtime_Seconds', 'Number_of_MILP_Variables']] for solver in solvers}
    data_other = {solver: data_other.loc[(data_other['MILP_Solver_Used'] == solver)][['Runtime_Seconds', 'Number_of_MILP_Variables']] for solver in solvers}
    data = {solver: pd.concat([data[solver], data_other[solver]], ignore_index=True) for solver in solvers }

    # Max Bound
    max_bound = max([max(data[solver]['Number_of_MILP_Variables']) for solver in solvers])

    # List of bins
    bins = np.arange(0, max_bound+bin_size, bin_size)
    binned_inds = {solver: np.digitize(data[solver]['Number_of_MILP_Variables'], bins) for solver in solvers}
    binned_data = {solver: sorted([data for data in zip(data[solver]['Runtime_Seconds'], binned_inds[solver])], key=lambda x: x[1]) for solver in solvers}
    binned_data = {solver: [[d[0] for d in binned_data[solver] if d[1] == ind] for ind in range(1, len(bins))] for solver in solvers}
    
    labels = [f"{(bin-bin_size)/1000}k-{bin/1000}k" for bin in bins[1::]]
    fig, ax = plt.subplots(1, len(solvers), sharey=True)
    plt.suptitle(f'IITCHS Planning - MILP Variables vs. Solve Time')
    ax[0].set_ylabel('Solve Time (sec)')
    for i, solver in enumerate(binned_data):
        ax[i].set_title(solver)
        ax[i].set_xlabel('MILP Variables')
        ax[i].boxplot(binned_data[solver])
        ax[i].set_xticklabels(labels)

    if show:
        plt.show(block=False)


if __name__ == '__main__':
    # Arg parse
    parser = ArgumentParser(description="Plotting benchmarks")
    parser.add_argument("-p", dest="datadir", required=True,
                        help="directory where pkl files are stored")
    args = parser.parse_args()
    datadir = args.datadir
    
    # Grab data from pkl files
    with open(os.path.join(datadir, 'DataFrame.pkl'), 'rb') as file1, open(os.path.join(datadir, 'DataFrame_Feasible.pkl'), 'rb') as file2, open(os.path.join(datadir, 'DataFrame_Infeasible.pkl'), 'rb') as file3, open(os.path.join(datadir, 'DataFrame_Other.pkl'), 'rb') as file4:
        data = pickle.load(file1)
        data_fea = pickle.load(file2)
        data_inf = pickle.load(file3)
        data_other = pickle.load(file4) # TODO: Assuming this represents all the timeouts, but double check later

    solvers = data_fea["MILP_Solver_Used"].unique()

    # Prints out the number of feasible, timed-out, and total MILPs to the terminal.
    print(f"\nTotal Feasible MILPs: {len(data_fea)+len(data_other)}\n# of fully-solved MILPs: {len(data_fea)}\n# of timed-out MILPs: {len(data_other)}")
    for solver in solvers:
        timeouts = len(data_other.loc[data_other["MILP_Solver_Used"] == solver])
        print(f"\t{timeouts} timeouts for {solver}")
    print("\n")

    # Plots
    robust_vs_feasible(data_fea, data_other)
    
    # # Number of states vs. Solve Time
    num_states_vs_solve_time(data_fea, data_other)

    # # Formula length vs solve time
    formula_len_vs_solve_time(data_fea, data_other, solvers, formula_name="Formula_Num_Tasks")
    formula_len_vs_solve_time(data_fea, data_other, solvers, formula_name="Formula_Num_Bool_Operators")

    # # AST bound vs solve time
    ast_bound_vs_solve_time(data_fea, data_other, solvers)

    # Regularize True/False
    regularize_true_false(data_fea, data_other, solvers, show=True)

    # Alpha
    alpha(data_fea, data_other, solvers, show=True)

    # Upper bound True/False
    upper_bound_true_false(data_fea, data_other, solvers, show=True)

    # MILP Constraints
    milp_constraints(data_fea, data_other, solvers, show=True)

    # MILP Variables
    milp_variables(data_fea, data_other, solvers, show=True)

    # Final blocking command that plots everything
    plt.show()
