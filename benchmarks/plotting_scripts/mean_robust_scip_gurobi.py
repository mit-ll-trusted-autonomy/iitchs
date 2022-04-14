'''
Plots a comparison of the mean / median solver times of Gurobi vs SCIP.

Also compares the difference between 'Robust' and 'Feasible' runtimes.

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
import pandas as pd
import pickle
import matplotlib.pyplot as plt
import numpy as np


if __name__ == '__main__':

	datadir = '/home/ja29747/code/iitchs_base/benchmarks/output/last_run/'

	with open(datadir+'DataFrame.pkl', 'rb') as file1, open(datadir+'DataFrame_Feasible.pkl', 'rb') as file2, open(datadir+'DataFrame_Infeasible.pkl', 'rb') as file3:
		data = pickle.load(file1)
		data_fea = pickle.load(file2)
		data_inf = pickle.load(file3)



	gurobi_robust = data_fea.loc[(data_fea['MILP_Solver_Used'] == 'GUROBI') & (data_fea['Robust'] == True)]['Runtime_Seconds']
	gurobi_feasible = data_fea.loc[(data_fea['MILP_Solver_Used'] == 'GUROBI') & (data_fea['Robust'] == False)]['Runtime_Seconds']

	scip_robust = data_fea.loc[(data_fea['MILP_Solver_Used'] == 'SCIP') & (data_fea['Robust'] == True)]['Runtime_Seconds']
	scip_feasible = data_fea.loc[(data_fea['MILP_Solver_Used'] == 'SCIP') & (data_fea['Robust'] == False)]['Runtime_Seconds']


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

	plt.show()




	# OLD BELOW ----------------------------------------------------------------------------------------------------------------------------------

	# # # Process the data for feasible instances

	# gurobi_robust_mean = data_fea.loc[(data_fea['MILP_Solver_Used'] == 'GUROBI') & (data_fea['Robust'] == True)]['Runtime_Seconds'].mean()
	# gurobi_feasible_mean = data_fea.loc[(data_fea['MILP_Solver_Used'] == 'GUROBI') & (data_fea['Robust'] == False)]['Runtime_Seconds'].mean()

	# gurobi_robust_median = data_fea.loc[(data_fea['MILP_Solver_Used'] == 'GUROBI') & (data_fea['Robust'] == True)]['Runtime_Seconds'].median()
	# gurobi_feasible_median = data_fea.loc[(data_fea['MILP_Solver_Used'] == 'GUROBI') & (data_fea['Robust'] == False)]['Runtime_Seconds'].median()

	# scip_robust_mean = data_fea.loc[(data_fea['MILP_Solver_Used'] == 'SCIP') & (data_fea['Robust'] == True)]['Runtime_Seconds'].mean()
	# scip_feasible_mean = data_fea.loc[(data_fea['MILP_Solver_Used'] == 'SCIP') & (data_fea['Robust'] == False)]['Runtime_Seconds'].mean() 

	# scip_robust_median = data_fea.loc[(data_fea['MILP_Solver_Used'] == 'SCIP') & (data_fea['Robust'] == True)]['Runtime_Seconds'].median()
	# scip_feasible_median = data_fea.loc[(data_fea['MILP_Solver_Used'] == 'SCIP') & (data_fea['Robust'] == False)]['Runtime_Seconds'].median() 

	
	# # # Plot the average and median times across solvers


	# labels = ['Robust Solution', 'Feasible Solution']
	# gur_means = [gurobi_robust_mean, gurobi_feasible_mean]
	# gur_medians = [gurobi_robust_median, gurobi_feasible_median]

	# scip_means = [scip_robust_mean, scip_feasible_mean]
	# scip_medians = [scip_robust_median, scip_feasible_median]


	# x = np.arange(len(labels))
	# width = 0.35

	# # Plot of Mean

	# fig, ax = plt.subplots(1,2, sharey=True)
	# plt.suptitle('Gurobi vs. SCIP, Robust vs Feasible')

	# # First plot: Means
	# rects1 = ax[0].bar(x - width/2, gur_means, width, label='Gurobi')
	# rects2 = ax[0].bar(x + width/2, scip_means, width, label='SCIP')

	# ax[0].set_ylabel('Solve Time (sec)')
	# ax[0].set_title('Mean Solving Times')
	# ax[0].set_xticks(x)
	# ax[0].set_xticklabels(labels)
	# ax[0].legend()

	# ax[0].bar_label(rects1, labels=[round(num,1) for num in gur_means], padding=3)
	# ax[0].bar_label(rects2, labels=[round(num,1) for num in scip_means], padding=3)

	# # Second plot: Medians
	# rects3 = ax[1].bar(x - width/2, gur_medians, width, label='Gurobi')
	# rects4 = ax[1].bar(x + width/2, scip_medians, width, label='SCIP')

	# # ax[0,1].set_ylabel('Solve Time (sec)')
	# ax[1].set_title('Median Solving Times')
	# ax[1].set_xticks(x)
	# ax[1].set_xticklabels(labels)
	# ax[1].legend()

	# ax[1].bar_label(rects3, labels=[round(num,1) for num in gur_medians], padding=3)
	# ax[1].bar_label(rects4, labels=[round(num,1) for num in scip_medians], padding=3)

	# # fig.tight_layout()

	# plt.show()

