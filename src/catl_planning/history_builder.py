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

import numpy as np
import random # THIS IMPORT IS ONLY FOR TESTING PURPOSES

def regCapHist(hist,regions,caps,agents):
	# reformat history array to be indexed by region, capability, and time
	# Input:
	# - hist : a list of np arrays that map agents to regions
	# - regions: a list of regions
	# - caps: a list of capabilities
	# - agents: a list of agents
	#
	# Output:
	# - reg_cap_hist: a 3-d np array, whose indices are region, capability, and time
	#		and whose values are the count of that capability in that region at that time.
	#		Indices are their respective indices in the input lists.

	# first, we need to pad the end of the history and convert it to a numpy array
	hist_pad = padHistory(hist)

	# initialize new history array
	reg_cap_hist = np.zeros((len(regions),len(caps),hist_pad.shape[1]),dtype=int)

	# for each agent
	for idx, row in enumerate(hist_pad):
	
		#encode agent history as a 1 for in each region at each time step
		one_hot_hist = np.zeros((len(regions),row.size),dtype=int)
		one_hot_hist[row,np.arange(row.size)] = 1

		# for each cap the agent has
		for cap in agents[idx][1]:
			reg_cap_hist[:,caps.index(cap),:] = np.add(reg_cap_hist[:,caps.index(cap),:],one_hot_hist)

	return reg_cap_hist

def padHistory(hist):
	# Take in a list of history arrays
	# Pad the last value so they are all of equal length

	max_len = max([x.size for x in hist]) # length things need to be padded to
	hist_pad = np.zeros((len(agents),max_len),dtype=int) # initialize new array
	for idx, x in enumerate(hist):
		if x.size<max_len: # needs to be padded
			pad_len = max_len - x.size
			pad_val = x[-1]
			x = np.concatenate((x,pad_val*np.ones(pad_len,dtype=int)),axis=None)
			hist_pad[idx] = x
		else:
			hist_pad[idx] = x

	return hist_pad


def main():
	######################################
	#
	# HERE IS WHERE WE TEST THESE THINGS!
	#
	######################################

	# seed for debugging purposes
	np.random.seed(0)
	random.seed(0)

	# dummy data for testing
	agents = [('q7', {'A1'}), ('q7', {'A1'}), ('q7', {'A2'}),  ('q7', {'A1','A2'}), ('q7',{'A1','A2'}),
				('q7',  {'A1'}), ('q7',  {'A2'}), ('q7',  {'A1'}), ('q7', {'A2'}), ('q7',{'A1'}),('q7',{'A2'})]
	regions = ['q0','q1','q2','q3','q4','q5','q6','q7','q8','q9','grave']
	max_time = 100
	caps = ['A1','A2']

	# create individual agent histories of length between 18 and 20
	time_vec = [random.randint(17,20) for i in range(len(agents))]
	hist = []
	for idx, x in enumerate(agents):
		hist.append(np.random.randint(len(regions),size=(time_vec[idx]))) 

	# Call our function
	reg_cap_hist = regCapHist(hist,regions,caps,agents)

	# to make sure things look good, let's check the first entry in our resulting matrix
	print (reg_cap_hist[0][0]) # region 0, capability 0

	hist_pad = padHistory(hist)

	ag_0 = [idx for idx, a in enumerate(agents) if 'A1' in a[1]] # agents with capability 0

	idx_list = np.zeros(max_time,dtype=int)
	for x in ag_0:
		for y in np.where(hist_pad[x]==0):
			idx_list[y] += 1
	print (idx_list)

	# idx_list and reg_cap_hist[0][0] should match

def main_2():

	cap_array = [1,2,3,1,1,1]
	moos_msg = "  AGENT_1 : q3 : 5 ; AGENT_2 : q3 : 5 "
	replan_moos_str_2_msg(moos_msg,cap_array)
	

if __name__ == "__main__":
    main_2()
