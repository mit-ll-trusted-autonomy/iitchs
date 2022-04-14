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

from z3 import *
import networkx as nx
from catl_planning.utils import getLeaves_nx
from itertools import *
import sys
import time

def encode_assignment_z3(slv,vrs,constraints,G,ts):
    
    # ensure all agents are assigned to at least one task!
    for jj in range(len(vrs['agents'])):
        constraint_list = [constraints['lambda_m_Agent_j'][(m,jj)] for m in vrs['leaves']]
        slv.add(Or(constraint_list))

        # declare Agent_j_Capability_i  
        for ii in range(len(vrs['caps'])):
            if vrs['caps'][ii] in vrs['agents'][jj][1]:
                slv.add(constraints['Agent_j_Capability_i'][(jj,ii)])
            else:
                slv.add(Not(constraints['Agent_j_Capability_i'][(jj,ii)]))

    t = time.time()
    # lambda_m_Eligible
    for m in vrs['leaves']:
        eligible_list = list()
        ce_list = list()
        for x in zip(G.node[m]['label']['caps'],G.node[m]['label']['counts']):
            # get label
            label = G.node[m]['label']['label']
            # find multiplicity of that label in ts
            lab_mult = len([y for y in ts.g.nodes() if label in ts.g.node[y]['prop']])
            # list of things that need to sum to at least x[1]
            mji_list = [constraints['lambda_m_Agent_j_Capability_i'][(m,jj,vrs['caps'].index(x[0]))] for jj in range(len(vrs['agents']))]
            num_req = x[1]*lab_mult
            eligible_list.append((Sum(mji_list)>=num_req))
            slv.add(constraints['cap_m_g_Excess'][(m,vrs['caps'].index(x[0]))]==Sum(mji_list)-num_req)
            ce_list.append(constraints['cap_m_g_Excess'][(m,vrs['caps'].index(x[0]))])
        slv.add(constraints['lambda_m_Eligible'][m]==And(eligible_list))
        slv.add(If(constraints['lambda_m_Eligible'][m],constraints['lambda_m_Excess'][m]==find_min(ce_list),constraints['lambda_m_Excess'][m]==1000))
    
        # make sure no agents are ineligibly assigned!
        for jj in range(len(vrs['agents'])):
            # if leaf is ineligible:
            slv.add(Implies(Not(constraints['lambda_m_Eligible'][m]),Not(constraints['lambda_m_Agent_j'][(m,jj)])))
            # agents are not assigned to it
            
            # encode lambda_m_Agent_j_Capability_i
            for ii in range(len(vrs['caps'])):
                slv.add(If(And(constraints['Agent_j_Capability_i'][(jj,ii)],constraints['lambda_m_Agent_j'][(m,jj)]),constraints['lambda_m_Agent_j_Capability_i'][(m,jj,ii)]==1,constraints['lambda_m_Agent_j_Capability_i'][(m,jj,ii)]==0))

        slv.add(constraints['lambda_m_Independent'][m])

    slv.add(constraints['lambda_Excess']==find_min(list(constraints['lambda_m_Excess'].values())))

    # make sure tasks joined by conjunction are satisfiable
    toCheck = [leaf for leaf in vrs['leaves']]
    while toCheck:
        leaf = toCheck.pop()
        pred = G.predecessors(leaf)[0]
        # TODO: what if this is ANDed with another subtree? -- then assume we are ok I think
        # If task is anded with another thing
        if G.node[pred]['label']['label']=='AND':
            # get other successor of this node
            succ = [x for x in G.successors(pred) if x!=leaf][0]
            # get temporal operators at or below succ
            to_right = [x for x in nx.dfs_postorder_nodes(G,succ) if G.node[x]['label']['label'][0] in ['F','G']]
            leaves_left = [x for x in nx.dfs_postorder_nodes(G,leaf) if x in vrs['leaves']]
            leaves_right = [x for x in nx.dfs_postorder_nodes(G,succ) if x in vrs['leaves']]
            labels_left = [G.node[x]['label']['label'] for x in leaves_left]
            labels_right = [G.node[x]['label']['label'] for x in leaves_right]

            # if the states that have that label are disjoint, their assignments need to be disjoint as well
            # For now, lets' assume different labels mean different states
            # TODO: update to check labels in TS?
            if succ in toCheck: # this means it's another task
                toCheck.remove(succ)
                # if nodes aren't on the same labeled state
                if G.node[leaf]['label']['label'] != G.node[succ]['label']['label']:
                    # force the assignments to be disjoint
                    slv.add(constraints['v_m_Independent'][pred])
                    # if to_right is empty, force independence if labels are disjoint
            elif len(to_right)==0:
                # all labels in right should be in labels_left
                if not all(label in labels_left for label in labels_right):
                    slv.add(constraints['v_m_Independent'][pred])

    # # Find nodes labeled until, and force their children to be independent?
    # untils = [node for node in G.nodes() if G.node[node]['label']['label'][0]=='U']
    # # If downstream on right hand side has temporal operators, we are good
    # # Or if labels on tasks are the same as labels on left hand side
    # for unt in untils:
    #     succs = G.successors(unt)
    #     # sort numerically (hence, convert to int then back to string)
    #     succs = [str(s) for s in sorted([int(x) for x in succs])]

    #     # get leaves and labels to left and right of until
    #     leaves_left = [x for x in nx.dfs_postorder_nodes(G,succs[0]) if x in vrs['leaves']]
    #     leaves_right = [x for x in nx.dfs_postorder_nodes(G,succs[1]) if x in vrs['leaves']]
    #     labels_left = [G.node[x]['label']['label'] for x in leaves_left]
    #     labels_right = [G.node[x]['label']['label'] for x in leaves_right]

    #     # get temporal operators to the right
    #     to_right = [x for x in nx.dfs_postorder_nodes(G,succs[1]) if G.node[x]['label']['label'][0] in ['F','G']]

    #     # if to_right is empty, force independence if labels are disjoint
    #     if len(to_right)==0:
    #         # all labels in right should be in labels_left
    #         if not all(label in labels_left for label in labels_right):
    #             slv.add(constraints['v_m_Independent'][unt])

    return slv


def operator_constraints_z3(slv,G,vrs,constraints,ts):

    # list of first letter of operators that we need to consider
    cu_set = ['A', 'U'] # first letter of labels 'AND' and 'UNTIL'
    d_set = ['O'] # first letter of label 'OR'
    ae_set = ['F', 'G'] # first letter of labels 'F' (eventually) and 'G' (always)

    t = time.time()
    slv = encode_assignment_z3(slv,vrs,constraints,G,ts)
    t2 = time.time()-t
    print("Encoding time: ",t2)

    # loop over nodes
    for x in vrs['nodes']:
    # check type of operator:
        # get first letter of operator and check what kind it is
        op = G.node[x]['label']['label'][0] 
        # call <operator>_constraints function
        if x=='0':
            slv = root_conjunction_constraints_z3(slv,G,vrs,constraints,x)
        elif op in cu_set:
            slv = conjunction_until_constraints_z3(slv,G,vrs,constraints,x)
        elif op in d_set:
            slv = disjunction_constraints_z3(slv,G,vrs,constraints,x)
        elif op in ae_set:
            slv = always_eventually_constraints_z3(slv,G,vrs,constraints,x)

    return slv

def root_conjunction_constraints_z3(slv,G,vrs,constraints,idx):

    # index of child nodes
    child_idxs = G.successors(idx)

    # v_m_Eligible
    child_eligible = list()
    for x in child_idxs:
        if x in vrs['leaves']:
            child_eligible.append(constraints['lambda_m_Eligible'][x])
        else:
            child_eligible.append(constraints['v_m_Eligible'][x])

    slv.add(constraints['v_m_Eligible'][idx]==And(child_eligible))

    for j in range(len(vrs['agents'])):
        ch_agent_list = []
        for x in child_idxs:
            if x in vrs['leaves']:
                ch_agent_list.append(constraints['lambda_m_Agent_j'][(x,j)])
            else:
                ch_agent_list.append(constraints['v_m_Agent_j'][(x,j)])  
        slv.add(constraints['v_m_Agent_j'][(idx,j)]==Or(ch_agent_list))

    for x in combinations(child_idxs,2):
        v_list = list()
        for j in range(len(vrs['agents'])):
            v_agent_list = list()
            if x[0] in vrs['leaves']:
                v_agent_list.append(constraints['lambda_m_Agent_j'][(x[0],j)])
            else:
                v_agent_list.append(constraints['v_m_Agent_j'][(x[0],j)])
            if x[1] in vrs['leaves']:
                v_agent_list.append(constraints['lambda_m_Agent_j'][(x[1],j)])
            else:
                v_agent_list.append(constraints['v_m_Agent_j'][(x[1],j)])
            v_list.append(And(v_agent_list))
        if x in constraints['root_v_m_v_mu_Independent'].keys():
            slv.add(constraints['root_v_m_v_mu_Independent'][x]==Not(Or(v_list)))
        else:
            slv.add(constraints['root_v_m_v_mu_Independent'][(x[1],x[0])]==Not(Or(v_list)))

    # add root independent for overall cost ?
    slv.add(If(constraints['v_m_Independent'][idx],constraints['v_Independent_cost'][idx]==1,constraints['v_Independent_cost'][idx]==0))
    slv.add(constraints['root_Independent']==And(list(constraints['root_v_m_v_mu_Independent'].values())))
    return slv

def conjunction_until_constraints_z3(slv,G,vrs,constraints,idx):
    # get constraints from node idx in G
    # node is for conjunction or until

    # index of child nodes
    child_idxs = G.successors(idx)

    # v_m_Eligible
    child_eligible = list()
    for x in child_idxs:
        if x in vrs['leaves']:
            child_eligible.append(constraints['lambda_m_Eligible'][x])
        else:
            child_eligible.append(constraints['v_m_Eligible'][x])
    
    for j in range(len(vrs['agents'])):
        ch_agent_list = []
        for x in child_idxs:
            if x in vrs['leaves']:
                ch_agent_list.append(constraints['lambda_m_Agent_j'][(x,j)])
            else:
                ch_agent_list.append(constraints['v_m_Agent_j'][(x,j)])  
        slv.add(constraints['v_m_Agent_j'][(idx,j)]==Or(ch_agent_list))

    # v_m_Eligible is an And() operation over a list of Child_Eligible
    slv.add(constraints['v_m_Eligible'][idx]==And(child_eligible))

    # since we are not at the root, we only have binary relationships
    # put a warning here just in case
    if len(child_idxs)>2:
        print("WARNING: Expected binary operator at UNTIL and CONJUNCTION")
        sys.exit()
    v_list = list()
    for j in range(len(vrs['agents'])):
        v_agent_list = list()
        for x in child_idxs:
            if x in vrs['leaves']:
                v_agent_list.append(constraints['lambda_m_Agent_j'][(x,j)])
            else:
                v_agent_list.append(constraints['v_m_Agent_j'][(x,j)])
        v_list.append(And(v_agent_list))
    slv.add(constraints['v_m_Independent'][idx]==Not(Or(v_list)))
    slv.add(If(constraints['v_m_Independent'][idx],constraints['v_Independent_cost'][idx]==1,constraints['v_Independent_cost'][idx]==0))

    return slv

def always_eventually_constraints_z3(slv,G,vrs,constraints,idx):
    # get constraints from node idx in G
    # node is for always or eventually operator

    # index of child node
    child_idx = G.successors(idx)[0]

    if child_idx in vrs['leaves']: # child is a task
        for y in range(len(vrs['agents'])):
            slv.add(constraints['v_m_Agent_j'][(idx,y)]==constraints['lambda_m_Agent_j'][(child_idx,y)])
        slv.add(constraints['v_m_Eligible'][idx]==constraints['lambda_m_Eligible'][child_idx])
        slv.add(constraints['v_m_Independent'][idx]==constraints['lambda_m_Independent'][child_idx])
        slv.add(If(constraints['v_m_Independent'][idx],constraints['v_Independent_cost'][idx]==1,constraints['v_Independent_cost'][idx]==0))

    else: # child is an operator
        for y in range(len(vrs['agents'])):
            slv.add(constraints['v_m_Agent_j'][(idx,y)]==constraints['v_m_Agent_j'][(child_idx,y)])
        # v_m_Eligible
        slv.add(constraints['v_m_Eligible'][idx]==constraints['v_m_Eligible'][child_idx])
        slv.add(constraints['v_m_Independent'][idx]==constraints['v_m_Independent'][child_idx])
        slv.add(If(constraints['v_m_Independent'][idx],constraints['v_Independent_cost'][idx]==1,constraints['v_Independent_cost'][idx]==0))

    return slv

def disjunction_constraints_z3(slv,G,vrs,constraints,idx):

    # get indices of child nodes
    child_idxs = G.successors(idx)

    # add xor between the eligibility of the children
    child_xor = list()
    for x in child_idxs:
        if x in vrs['leaves']:
            # append leaf eligibility
            child_xor.append(constraints['lambda_m_Eligible'][x])
        else:
            # append node eligibility
            child_xor.append(constraints['v_m_Eligible'][x])
            # Ensure ALL downstream nodes to have an empty assignment!
            succ_leaves = [node for node in nx.dfs_postorder_nodes(G,x) if node in vrs['leaves']]
            assign_list = []
            for leaf in succ_leaves:
                # Assign list should have all agents and leaves in it
                for agent in range(len(vrs['agents'])):
                    assign_list.append(constraints['lambda_m_Agent_j'][(leaf,agent)])
            slv.add(Implies(Not(constraints['v_m_Eligible'][x]),
                            Not(Or(assign_list))))
    slv.add(listXor(child_xor))

    child_consts = list()
    for x in child_idxs:
        child_list = list()
        if x in vrs['leaves']:
            child_list.append(constraints['v_m_Eligible'][idx]==constraints['lambda_m_Eligible'][x])
            child_list.append(constraints['v_m_Independent'][idx]==constraints['lambda_m_Independent'][x])
            child_list.append(If(constraints['v_m_Independent'][idx],constraints['v_Independent_cost'][idx]==1,constraints['v_Independent_cost'][idx]==0))
            for y in range(len(vrs['agents'])):
                child_list.append(constraints['v_m_Agent_j'][(idx,y)]==constraints['lambda_m_Agent_j'][(x, y)])
        else:
            child_list.append(constraints['v_m_Eligible'][idx]==constraints['v_m_Eligible'][x])
            child_list.append(constraints['v_m_Independent'][idx]==constraints['v_m_Independent'][x])
            child_list.append(If(constraints['v_m_Independent'][idx],constraints['v_Independent_cost'][idx]==1,constraints['v_Independent_cost'][idx]==0))
            for y in range(len(vrs['agents'])):
                child_list.append(constraints['v_m_Agent_j'][(idx,y)]==constraints['v_m_Agent_j'][(x,y)])
        child_consts.append(And(child_list))

    slv.add(listXor(child_consts))

    return slv

def mymin(x,y):
    # function to compute min of two integers
    return If(x >= y, y, x)

def find_min(mylist):
    # function to compute min from list of integers
    if len(mylist)>1:
        min_exp = mymin(mylist[0],mylist[1])
        for i in range(2,len(mylist)):
            min_exp = mymin(min_exp,mylist[i])
    else:
        min_exp = mylist[0]
    return min_exp

def listXor(in_list):
    # helper function to nest Xor for more than 2 arguments

    xor_out = Xor(in_list[0],in_list[1])
    if len(in_list)>2:
        for x in range(2,len(in_list)):
            xor_out = Xor(xor_out,in_list[x])

    return xor_out

def declare_vrs(vrs,G):
    # make dictionary of dictionaries of constraints to be passed to other functions in z3_functions

    constraints = dict()

    # agents and capabilities
    constraints['Agent_j_Capability_i'] = {(x,y): Bool('Agent_%s_Capability_%s' % (x,y)) for x in range(len(vrs['agents'])) for y in range(len(vrs['caps']))}

    # task constraints
    constraints['lambda_m_Agent_j'] = {(x,y): Bool('lambda_%s_Agent_%s' % (x, y)) for x in vrs['leaves'] for y in range(len(vrs['agents']))}
    constraints['lambda_m_Eligible'] = {x: Bool('lambda_%s_Eligible' % x) for x in vrs['leaves']}
    constraints['lambda_m_Agent_j_Capability_i'] = {(x,y,z): Int('lambda_%s_Agent_%s_Capability_%s' % (x, y, z)) for x in vrs['leaves'] for y in range(len(vrs['agents'])) for z in range(len(vrs['caps']))}
    constraints['lambda_m_Independent'] = {x: Bool('lambda_%s_Independent' % x) for x in vrs['leaves']}
    constraints['lambda_m_Excess'] = {x: Int('lambda_%s_Excess' % x) for x in vrs['leaves']}
    constraints['lambda_Excess'] = Int('lambda_Excess')
    
    # node constraints
    constraints['v_m_Eligible'] = {x: Bool('v_%s_Eligible' % x) for x in vrs['nodes']}
    constraints['v_m_Agent_j'] = {(x,y): Bool('v_%s_Agent_%s' % (x,y)) for x in vrs['nodes'] for y in range(len(vrs['agents']))}
    constraints['v_m_Independent'] = {x: Bool('v_%s_Independent' % x) for x in vrs['nodes']}
    constraints['v_Independent_cost'] = {x: Int('v_%s_Independent_cost' % x) for x in vrs['nodes']}

    children = G.successors('0')
    constraints['root_v_m_v_mu_Independent'] = {(y[0],y[1]): Bool('root_v_%s_v_%s_Independent' % (y[0],y[1])) for y in combinations(children,2)}
    constraints['root_Independent_cost'] = {(y[0],y[1]): Int('root_v_%s_v_%s_Independent_cost' % (y[0],y[1])) for y in combinations(children,2)}
    constraints['root_Independent'] = Bool('root_Independent')
    
    constraints['cap_m_g_Excess'] = {(x,y): Int('cap_%s_%s_Excess' % (x,y)) for x in vrs['leaves'] for y in range(len(vrs['caps']))}

    return constraints


def init_vrs(agents,G):

    # list of capabilities
    caps = list(set.union(*[cap for _, cap in agents]))

    # get set of unique classes
    classes = [x for _,x in agents]
    all_class = list()
    for x in range(len(classes)-1,-1,-1):
        if classes.index(classes[x])==x:
            all_class.append(classes[x])

    # maximum number of each class
    max_f = [ len([x for x in range(len(classes)) if classes[x]==y]) for y in all_class]

    leaves = getLeaves_nx(G)
    nodes = list(set(G.nodes())-set(leaves))

    vrs = dict()

    vrs['caps']        = caps
    vrs['all_class']   = all_class
    vrs['nodes']       = nodes
    vrs['leaves']      = leaves
    vrs['max_f']       = max_f
    vrs['agents']      = agents

    return vrs
