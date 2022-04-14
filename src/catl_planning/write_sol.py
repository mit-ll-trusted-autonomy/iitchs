'''
PuLP SOL File Generator

Generates .sol files when using PuLP as an optimization interface.
Using Gurobi is not required.

**Note:** .sol files are specific to the Gurobi solver. The
specification can be found at: 
https://www.gurobi.com/documentation/9.1/refman/sol_format.html
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

import pulp
import sys
if sys.flags.debug:
	import pdb

def write_sol(pulp_problem, file_name=None):
	"""
    Generates a .sol file from a PuLP LpProblem object.

    Args:
        pulp_problem: PuLP LpProblem object to create .sol
            file from. The LpProblem object must be solved before 
            passing it into this function.

        file_name (str): (Optional). Name (and path) of the output
            file. By default the output file is saved to `./pulp.sol`.
	"""

	if file_name is None:
		file_name = "./pulp.sol"

	f = open(file_name, "w")

	f.write("# Solution for PuLP model " + pulp_problem.name + "\n")
	if pulp_problem.objective is not None:
		f.write("# Objective value = " + str(pulp_problem.objective.value()) + "\n") # Can also use pulp.value(prob.objective)
	else:
		f.write("No Objective Value (model.objective == None)")

	# In PuLP, calling `.variables()` seems to produce a list of variables
	# in alphabetical order. Gurobi instead prints all variables to the .sol file
	# in order of variable creation.
	for var in pulp_problem.variables():
		f.write(var.name + " " + str(var.value()) + "\n")

	f.close()
