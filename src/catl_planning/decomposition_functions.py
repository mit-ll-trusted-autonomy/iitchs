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

import networkx as nx
import pickle
import itertools
import time
import copy

from catl import CATLFormula

from catl_planning.route_planning import computeRobustnessUpperBound
from catl_planning.utils import plotAST_nx
from catl_planning.z3_functions import *
import z3

# TODO: Modify splitting of graph to only happen if resulting agent sets are disjoint? But, we could conceivably split a group A,B, and C from ABC to AB ABC and then further into AB AB C and then recombine AB+AB and leave C separate!

def decompose(spec, agents, ts,root_wt=-100,v_ind_wt=-10,lambda_wt=-1):
        # main function for decomposing a spec and teams
        # make spec into network x graph
        G = plotAST_nx(spec)

        # dictionary to hold problem variables
        vrs = init_vrs(agents,G)

        constraints = declare_vrs(vrs,G)

        # print("CREATE OPT")
        # initialize z3 optimizer
        # opt = Optimize()
        slv = Solver()


        # set timout to 60 seconds (60,000 milliseconds)
        slv.set('timeout',60000)

        # add constraints to optimizer
        t = time.time()
        slv = operator_constraints_z3(slv,G,vrs,constraints,ts)
        constraint_time = time.time()-t
        print("Constraint time: ",constraint_time)

        # add requirement that root node '0' should have Op_m_Eligible==True
        slv.add(constraints['v_m_Eligible']['0'])
        slv.add(constraints['root_Independent'])

        t1 = time.time()
        # check if model is satisfiable
        if slv.check() == z3.unsat:
                # IF UNSAT, RETURN ORIGINAL PROBLEM
                print("MODEL UNSAT! Returning original formula and agents")
                formula_out = dict()
                formula_out[0] = dict()
                formula_out[0]['agents'] = range(len(agents))
                formula_out[0]['formula'] = spec
                return formula_out, "unsat"

        try:
                mlen = len(slv.model())
        except:
                # some error
                mlen = 0

        if mlen==0:
                # model timed out, presumablt
                print("Z3 TIMED OUT! Returning original formula and agents")
                formula_out = dict()
                formula_out[0] = dict()
                formula_out[0]['agents'] = range(len(agents))
                formula_out[0]['formula'] = spec
                return formula_out, "timeout"

        t2 = time.time()-t1
        print("Solving time: ",t2)

        # UNCOMMENT THE FOLLOWING TO PRINT THE SATISFYING MODEL TO TERMINAL
        # # print satisfying model
        # print(slv.model())

        # UNCOMMENT THE FOLLOWING TO SAVE THE MODEL CONSTRAINTS TO A PICKLE FILE
        # # save model constraints
        # pickle.dump(slv.sexpr(), open("save_slv_assertions.p","wb"))
        # with open("slv.txt", "w") as text_file:
        #     text_file.write(slv.sexpr())


        # UNCOMMENT THE FOLLOWING TO SAVE THE MODEL TO PICKLE FILES OR TEXT
        # # save model
        # pickle.dump(slv.model().sexpr(), open("save_slv_model.p","wb"))
        # # or
        # with open("slv_model.txt", "w") as text_file:
        #     text_file.write(slv.model().sexpr())

        # UNCOMMENT THE FOLLOWING TO LOAD FROM PICKLE FILES
        # # load file
        # with open('save_slv_assertions.p', 'rb') as f:
        #      x = pickle.load(f)

        # print('ROOT COST')
        # for x in constraints['root_Independent_cost']:
        #       print(x,slv.model()[x])

        # print('V_IND')
        # for x in constraints['v_Independent_cost']:
        #       print(x,slv.model()[x])

        model = slv.model()
        # get initial task assignments
        tasks = taskAssignments(constraints, model)

        # cache capability to index mapping
        cap2idx = {c:i for i,c in enumerate(vrs['caps'])}
        cap_excesses = []
        for task in tasks:
            cap_excesses.append(
                    find_min([constraints['cap_m_g_Excess'][(task, cap2idx[c])]
                     for c in G.node[task]['label']['caps']
                    ]))


        t3 = time.time()
        model = refineAssignments(constraints, slv, tasks, cap_excesses)
        refinement_time = time.time()-t3
        print(f'Refinement time: {refinement_time}')

        # reassign tasks
        tasks = taskAssignments(constraints, model)

        # print('TASKS')
        # for k in tasks.keys():
                # print(k,tasks[k])

        # COMBINE TASKS THAT HAVE OVERLAPPING AGENTS
        # loop over keys
        changed = True
        while changed:
                changed = False
                for k1, k2 in itertools.combinations(tasks.keys(),2):
                        # if entries are not identical
                        if set(tasks[k1])!=set(tasks[k2]):
                                if len(set(tasks[k1]).intersection(set(tasks[k2])))!=0:
                                        changed = True
                                        tasks[k1] = list(set(tasks[k1]).union(set(tasks[k2])))
                                        tasks[k2] = list(set(tasks[k1]).union(set(tasks[k2])))

        # make a copy for final assignments
        G2 = copy.deepcopy(G)

        label_assignment(G,tasks)
        subtask_graphs_prelim = getSubtasks_assign(G,tasks,vrs,ts,agents)

        # now with the final assignment, we need to double check the splitting of the original graph
        tasks2 = dict()
        for graph in subtask_graphs_prelim:
                get_final_assignment(graph)
                leaves = [node for node in graph.nodes() if len(graph.successors(node))==0]
                for leaf in leaves:
                        tasks2[leaf] = [ag for ag in graph.node[leaf]['agents']]
        label_assignment(G2,tasks2)
        subtask_graphs = getSubtasks_assign(G2,tasks2,vrs,ts,agents)

        formulas = [formulaFromGraph(x) for x in subtask_graphs]

        formula_out = dict()
        for x in range(len(subtask_graphs)):
                formula_out[x] = dict()
                formula_out[x]['graph'] = subtask_graphs[x]
                root = nx.topological_sort(subtask_graphs[x])[0]
                formula_out[x]['root'] = root
                formula_out[x]['agents'] = subtask_graphs[x].node[root]['agents']
                formula_out[x]['formula'] = subtask_graphs[x].node[root]['formula']

        return formula_out, slv

########
# The next set of functions (starting with the word "find") return a list of nodes from a graph that correspond to a specific operator
# For example, findUnt locates nodes that correspond with Until in the formula from which the graph was constructed
########

def findUnt(nx_graph):
        unt_idx = findOper(nx_graph,'U')
        return unt_idx

def findAlw(nx_graph):
        alw_idx = findOper(nx_graph,'G')
        return alw_idx

def findEvent(nx_graph):
        event_idx = findOper(nx_graph,'F')
        return event_idx

def findOper(nx_graph,str):
        # find indices of operator starting with str
        str_idx = [x for x in nx_graph.nodes() if 'label' in nx_graph.node[x]['label'] and nx_graph.node[x]['label']['label'][0]==str]
        return str_idx

def findOr(nx_graph):
        # find indices of nodes labeled 'OR'
        # check all nodes
        or_idx = findStr(nx_graph,'OR')
        return or_idx

def findAnd(nx_graph):
        # find indices of nodes labeled 'OR'
        # check all nodes
        and_idx = findStr(nx_graph,'AND')
        return and_idx

def findStr(nx_graph,str):
        # find indices of nodes that match string
        # check all nodes
        str_idx = [x for x in nx_graph.nodes() if 'label' in nx_graph.node[x]['label'] and nx_graph.node[x]['label']['label']==str]
        return str_idx

########
# The following functions are used to locate constraints in 'slv' concerning whether a given node is Eligible or Substitutable
########

def findEligible(slv, node):
        # find constraint index associated with the eligibility of 'node' (whether a node or a leaf)
        const_idx = [i for i,x in enumerate(range(len(slv.model().decls()))) if str(slv.model().decls()[x]) in ['lambda_'+node+'_Eligible','v_'+node+'_Eligible']]
        return const_idx

def findSubstitutable(slv, node):
        # find constraint index associated with the substitutability of 'node' (whether a node or a leaf)
        const_idx = [i for i,x in enumerate(range(len(slv.model().decls()))) if str(slv.model().decls()[x]) in ['lambda_'+node+'_Substitutable','v_'+node+'_Substitutable']]
        return const_idx

########
# The following set of functions are used to decompose a graph based on the formula from which the graph was constructed
# The main function is getSubtasks
########

def pruneOr(nx_graph, slv):
        # for all nodes labeled 'OR', remove branches that have not been selected
        # get list of all nodes that are labeled 'OR'
        or_list = findOr(nx_graph)
        # for each node, find its successors and eliminate ineligible downstream nodes
        for node in or_list:
                preds = nx_graph.predecessors(node)
                next_nodes = nx_graph.successors(node)
                # check each for eligibility
                for nxt in next_nodes:
                        # find constraint in z3 solver
                        const_idx = findEligible(slv,nxt)
                        # prune from graph if not eligible
                        if not slv.model()[slv.model().decls()[const_idx[0]]]:
                                nx_graph.remove_nodes_from([x for x in nx.dfs_postorder_nodes(nx_graph,nxt)])
                        else: # nxt is eligible, need an edge from preds to it
                                for pred in preds:
                                        nx_graph.add_edge(pred,nxt)
                nx_graph.remove_node(node) # remove or
        return nx_graph

def pruneOr_assign(nx_graph, vrs, ts, agents):
        # for all nodes labeled 'OR', remove agents from branches that have not been selected
        # get list of all nodes that are labeled 'OR'

        # need ts, agents

        or_list = findOr(nx_graph)
        # for each node, find its successors and eliminate ineligible downstream nodes
        for node in or_list:
                print("PRUNING OR FOR: ", node)
                next_nodes = nx_graph.successors(node)
                # check each for eligibility
                for nxt in next_nodes:
                        # find constraint in z3 solver
                        if nxt in vrs['leaves']:
                                print("LOOKING AT SUCCESSOR: ",nxt)
                                # prune from graph if not eligible
                                taskFormula(nx_graph,nxt)
                                ast = CATLFormula.from_formula(nx_graph.node[nxt]['formula'])
                                ce = computeRobustnessUpperBound(ts,[agents[int(x)] for x in nx_graph.node[node]['agents']],ast)
                                if ce < 0:
                                        print("SUCCESSOR IS NOT ELIGIBLE")
                                        # get predecessors
                                        preds = nx_graph.predecessors(node)
                                        # remove successors of nxt
                                        nx_graph.remove_nodes_from([x for x in nx.dfs_postorder_nodes(nx_graph,nxt)])

                                        # add edge to other node
                                        next_nodes.remove(nxt)
                                        nx_graph.add_edge(preds[0],next_nodes[0])

                                        # remove OR
                                        nx_graph.remove_node(node) # remove or

                        else:
                                # TODO: update for current version of pruning!!!
                                #raise NotImplementedError
                                # nxt is a node
                                print("LOOKING AT SUCCESSOR: ",nxt)
                                formulaFromGraph(nx_graph)
                                ast = CATLFormula.from_formula(nx_graph.node[nxt]['formula'])
                                ce = computeRobustnessUpperBound(ts,[agents[int(x)] for x in nx_graph.node[node]['agents']],ast)
                                if ce < 0:
                                        print("SUCCESSOR IS NOT ELIGIBLE")
                                        print("CURRENT AGENT SET IS: ",nx_graph.node[nxt]['agents'])
                                        nx_graph.node[nxt]['agents']=set()
                                        succ_list = [x for y in nx.dfs_successors(nx_graph,nxt).values() for x in y]
                                        for succ in succ_list:
                                                print(succ, " IS SUCCESSOR OF ",nxt,". SETTING TO EMPTY AGENT SET.")
                                                print("CURRENT AGENT SET IS: ",nx_graph.node[succ]['agents'])
                                                nx_graph.node[succ]['agents']=set()
        return nx_graph


def opLimits(nx_graph,node):
        # return limits of temporal operator at node in nx_graph
        # entire label
        node_str = nx_graph.node[node]['label']['label']
        # now split the string into 1st and 2nd limits
        lim1 = node_str.split('[')[1].split(',')[0]
        lim2 = node_str.split('[')[1].split(',')[1].split(']')[0]
        return lim1, lim2

def subUntil(nx_graph):
        # find all untils
        unt_list = findUnt(nx_graph)
        # index of highest node
        maxNode = max(map(int,nx_graph.nodes()))
        # for each node make appropriate substitutions
        for node in unt_list:
                # get time bounds on until
                lim1, lim2 = opLimits(nx_graph,node)
                # get predecessors and successors
                preds = nx_graph.predecessors(node)
                succs = nx_graph.successors(node)
                succs2 = [int(x) for x in succs]
                succs2.sort()
                succs = [str(x) for x in succs2]
                # succs.sort() # this will keep left and right successors in the correct order
                # add new node, labeled with and
                maxNode += 1
                nx_graph.add_node(str(maxNode))
                for pred in preds:
                        nx_graph.add_edge(pred,str(maxNode))
                nx_graph.node[str(maxNode)]['label'] = {'label':'AND'}
                # NOTE: we are assuming successor 1 is 'left' and successor 2 is 'right'
                # this assumption matches our code for constructing G. Changes to that code
                # could mean that we need to modify this code.
                # add always node to connect to successor 1
                maxNode +=1
                nx_graph.add_node(str(maxNode))
                nx_graph.node[str(maxNode)]['label'] = {'label':'G [0.0, '+lim2+']'}
                nx_graph.add_edge(str(maxNode-1),str(maxNode))
                nx_graph.add_edge(str(maxNode),succs[0])
                # add eventually node to connect to successor 2
                maxNode +=1
                nx_graph.add_node(str(maxNode))
                nx_graph.node[str(maxNode)]['label'] = {'label':'F ['+lim1+', '+lim2+']'}
                nx_graph.add_edge(str(maxNode-2),str(maxNode))
                nx_graph.add_edge(str(maxNode),succs[1])
                # remove until node
                nx_graph.remove_node(str(node))

def subUntil_assign(nx_graph, tasks):
        # find all untils
        unt_list = findUnt(nx_graph)
        # index of highest node
        maxNode = max(map(int,nx_graph.nodes()))
        # for each node make appropriate substitutions
        for node in unt_list:
                # get time bounds on until
                lim1, lim2 = opLimits(nx_graph,node)
                # get predecessors and successors
                preds = nx_graph.predecessors(node)
                succs = nx_graph.successors(node)
                succs2 = [int(x) for x in succs]
                succs2.sort()
                succs = [str(x) for x in succs2]
                # add new node, labeled with and

                succ_leaves = list()
                for y in succs:
                        if nx_graph.out_degree(y)==0:
                                # successor is a leaf, all its assigned agents are on the node itself
                                if y in tasks:
                                        # if it doesn't have a key, we don't need to worry about it
                                        succ_leaves.append(set(tasks[y]))
                        else:
                                # successor has subsequent leaves
                                succ_leaves.append(set(itertools.chain.from_iterable([tasks[x] for x in nx.dfs_postorder_nodes(nx_graph,y) if nx_graph.out_degree(x)==0 and x in tasks])))

                # if intersection of succ_leaves is not empty, we don't need to substitute
                if len(succ_leaves)>1  and not succ_leaves[0].intersection(succ_leaves[1]):
                        maxNode += 1
                        nx_graph.add_node(str(maxNode))
                        for pred in preds:
                                nx_graph.add_edge(pred,str(maxNode))
                        nx_graph.node[str(maxNode)]['label'] = {'label':'AND'}
                        # NOTE: we are assuming successor 1 is 'left' and successor 2 is 'right'
                        # this assumption matches our code for constructing G. Changes to that code
                        # could mean that we need to modify this code.
                        # add always node to connect to successor 1
                        maxNode +=1
                        nx_graph.add_node(str(maxNode))
                        nx_graph.node[str(maxNode)]['label'] = {'label':'G [0.0, '+lim2+']'}
                        nx_graph.add_edge(str(maxNode-1),str(maxNode))
                        nx_graph.add_edge(str(maxNode),succs[0])
                        # add eventually node to connect to successor 2
                        maxNode +=1
                        nx_graph.add_node(str(maxNode))
                        nx_graph.node[str(maxNode)]['label'] = {'label':'F ['+lim1+', '+lim2+']'}
                        nx_graph.add_edge(str(maxNode-2),str(maxNode))
                        nx_graph.add_edge(str(maxNode),succs[1])
                        # remove until node

                        # label agent sets in graph
                        combine_agent_sets(nx_graph,str(maxNode))
                        combine_agent_sets(nx_graph,str(maxNode-1))
                        combine_agent_sets(nx_graph,str(maxNode-2))

                        nx_graph.remove_node(str(node))

def subAndLoop(nx_graph):
        # run subAnd until the graph stops changing size
        # this is necessary if we have nested temporal operators with an and (e.g. G F (t1 && t2))
        doneLoop = False
        while not doneLoop:
                size_g = len(nx_graph.nodes())
                subAnd(nx_graph)
                if len(nx_graph.nodes())==size_g:
                        doneLoop = True

def subAndLoop_assign(nx_graph):
        # run subAnd until the graph stops changing size
        # this is necessary if we have nested temporal operators with an and (e.g. G F (t1 && t2))
        doneLoop = False
        while not doneLoop:
                size_g = len(nx_graph.nodes())
                subAnd_assign(nx_graph)
                if len(nx_graph.nodes())==size_g:
                        doneLoop = True

def subAnd(nx_graph):
        # substitute and with upstream conjunction
        alw_idx = findAlw(nx_graph)
        event_idx = findEvent(nx_graph)
        op_idx = alw_idx + event_idx # all nodes with temporal operator
        # for each node, check if any of its successors are 'AND'
        for node in op_idx:
                for succ in nx_graph.successors(node):
                        if nx_graph.node[succ]['label']['label'] == 'AND':
                                # need to substitute node with AND with downstream ALWAYS
                                lim1, lim2 = opLimits(nx_graph,node) # get time bounds
                                maxNode = max(map(int,nx_graph.nodes()))+1 # index of highest node plus one
                                nx_graph.add_node(str(maxNode))
                                nx_graph.node[str(maxNode)]['label'] = {'label':'AND'}
                                preds =  nx_graph.predecessors(node)
                                for pred in preds:
                                        nx_graph.add_edge(pred,str(maxNode))
                                succs = nx_graph.successors(succ)
                                # for each successor of the original 'AND'
                                # add an always node and connect it to that successor and to the new 'AND' node
                                # add always node to connect to successor 1
                                maxNode +=1
                                nx_graph.add_node(str(maxNode))
                                nx_graph.node[str(maxNode)]['label'] = {'label':'G ['+lim1+', '+lim2+']'}
                                nx_graph.add_edge(str(maxNode-1),str(maxNode)) # connect to new 'AND'
                                nx_graph.add_edge(str(maxNode),succs[0]) # connect to successor
                                # add eventually node to connect to successor 2
                                maxNode +=1
                                nx_graph.add_node(str(maxNode))
                                nx_graph.node[str(maxNode)]['label'] = {'label':'G ['+lim1+', '+lim2+']'}
                                nx_graph.add_edge(str(maxNode-2),str(maxNode)) # connect to new 'AND'
                                nx_graph.add_edge(str(maxNode),succs[1]) # connect to successor
                                # since there's only one predecessor for any node, and because all operators are binary,
                                # we can remove the original operator and 'AND' in this if statement -- it will only be entered once
                                # for each node in op_idx
                                nx_graph.remove_node(node) # remove original temporal operator node
                                nx_graph.remove_node(succ) # remove original 'AND'

def subAnd_assign(nx_graph):
        # substitute and with upstream conjunction
        alw_idx = findAlw(nx_graph)
        event_idx = findEvent(nx_graph)
        op_idx = alw_idx + event_idx # all nodes with temporal operator
        # for each node, check if any of its successors are 'AND'
        for node in op_idx:
                for succ in nx_graph.successors(node):
                        if nx_graph.node[succ]['label']['label'] == 'AND':
                                # need to substitute node with AND with downstream ALWAYS
                                # successors of the AND node
                                succs = nx_graph.successors(succ)

                                # if successors have disjoint assignments, then we DO want to split
                                if nx_graph.node[succs[0]]['agents'].intersection(nx_graph.node[succs[1]]['agents']):
                                        # successors share an assignemtn, so we DON'T want to split
                                        break
                                else:
                                        # OK SO WE PROBABLY WANT TO DO THIS BASED ON TEAMS, NOT ASSIGNMENTS?
                                        # if successors have disjoint assignments, then we DO want to split
                                        lim1, lim2 = opLimits(nx_graph,node) # get time bounds
                                        maxNode = max(map(int,nx_graph.nodes()))+1 # index of highest node plus one
                                        nx_graph.add_node(str(maxNode))
                                        nx_graph.node[str(maxNode)]['label'] = {'label':'AND'}
                                        preds =  nx_graph.predecessors(node)
                                        for pred in preds:
                                                nx_graph.add_edge(pred,str(maxNode))
                                        # for each successor of the original 'AND'
                                        # add an always node and connect it to that successor and to the new 'AND' node
                                        # add always node to connect to successor 1
                                        maxNode +=1
                                        nx_graph.add_node(str(maxNode))
                                        nx_graph.node[str(maxNode)]['label'] = {'label':'G ['+lim1+', '+lim2+']'}
                                        nx_graph.add_edge(str(maxNode-1),str(maxNode)) # connect to new 'AND'
                                        nx_graph.add_edge(str(maxNode),succs[0]) # connect to successor
                                        # add eventually node to connect to successor 2
                                        maxNode +=1
                                        nx_graph.add_node(str(maxNode))
                                        nx_graph.node[str(maxNode)]['label'] = {'label':'G ['+lim1+', '+lim2+']'}
                                        nx_graph.add_edge(str(maxNode-2),str(maxNode)) # connect to new 'AND'
                                        nx_graph.add_edge(str(maxNode),succs[1]) # connect to successor
                                        # since there's only one predecessor for any node, and because all operators are binary,
                                        # we can remove the original operator and 'AND' in this if statement -- it will only be entered once
                                        # for each node in op_idx
                                        nx_graph.remove_node(node) # remove original temporal operator node
                                        nx_graph.remove_node(succ) # remove original 'AND'

                                        # label agent sets in graph
                                        combine_agent_sets(nx_graph,str(maxNode))
                                        combine_agent_sets(nx_graph,str(maxNode-1))
                                        combine_agent_sets(nx_graph,str(maxNode-2))

def splitRootAnd(nx_graph):
        # split graph at root if root is and, return two subgraphs
        root = nx.topological_sort(nx_graph)[0]
        if nx_graph.node[root]['label']['label'] == 'AND':
                nx_graph.remove_node(root)
        graphs = list(nx.weakly_connected_component_subgraphs(nx_graph))
        return graphs

def splitRootAnd_assign(nx_graph):
        # split graph at root if root is and, return two subgraphs

        root = nx.topological_sort(nx_graph)[0]
        # only split if the root node is 'AND'
        if nx_graph.node[root]['label']['label'] == 'AND':
                # successors
                succs = nx_graph.successors(root)
                # get rid of cascading AND nodes
                checkSuccs = True
                while checkSuccs:
                        sizeG = len(nx_graph.nodes())
                        for x in succs:
                                if nx_graph.node[x]['label']['label'] == 'AND':
                                        succs_2 = nx_graph.successors(x)
                                        for y in succs_2:
                                                nx_graph.add_edge(root,y)
                                        nx_graph.remove_node(x)
                        # need to refresh the list of successors
                        succs = nx_graph.successors(root)

                        # if the size of the graph hasn't changed, we're done
                        if sizeG==len(nx_graph.nodes()):
                                checkSuccs = False

                subteams = dict()
                key = 1
                while succs:
                        x = succs.pop(0)
                        x_agents = nx_graph.node[x]['agents']
                        x_nodes = [x]
                        loop_done = False
                        while not loop_done:
                                intersects = [y for y in succs if x_agents.intersection(nx_graph.node[y]['agents'])]
                                for y in intersects:
                                        x_agents = x_agents.union(nx_graph.node[y]['agents'])
                                        x_nodes.append(y)
                                        succs.remove(y)
                                if len([y for y in succs if x_agents.intersection(nx_graph.node[y]['agents'])])==0:
                                                subteams[key]   = dict()
                                                subteams[key]['agents'] = x_agents
                                                subteams[key]['nodes'] = x_nodes
                                                key += 1
                                                loop_done = True
                                        # now need to check for intersections again!
                                        # if no more intersections
                                                # add to dictionary
                                                # increment key
                                                # loop_done=True


                # if more than one subteam
                if len(subteams.keys())>1:
                        # remove root AND and replace it
                        nx_graph.remove_node(root)
                        for key in subteams.keys():
                                # for each key, add a new AND node, and connect that key's nodes to it
                                nodes = subteams[key]['nodes']
                                if len(nodes) > 1:
                                        new_node = str(max([int(x) for x in nx_graph.nodes()])+1)
                                        nx_graph.add_node(new_node)
                                        nx_graph.node[new_node]['label'] = dict()
                                        nx_graph.node[new_node]['label']['label'] = 'AND'
                                        nx_graph.node[new_node]['agents'] = set()
                                        for x in subteams[key]['nodes']:
                                                nx_graph.add_edge(new_node,x)
                                                nx_graph.node[new_node]['agents'] = nx_graph.node[new_node]['agents'].union(nx_graph.node[x]['agents'])

        # split up into subgraphs
        graphs = list(nx.weakly_connected_component_subgraphs(nx_graph))
        return graphs

def parallelize(nx_graph):
        # split graph at root 'AND' recursively until original graph is parallelized
        # first call to parallelize
        graphs = splitRootAnd(nx_graph)
        if len(graphs) > 1:
                new_graphs = []
                for g in graphs:
                        subg = parallelize(g)
                        new_graphs += subg
                graphs = new_graphs
        return graphs

def parallelize_assign(nx_graph):
        # split graph at root 'AND' recursively until original graph is parallelized
        # first call to parallelize

        #TODO: ADD A STEP HERE COMBINING 'AND' NODES AT THE ROOT!

        graphs = splitRootAnd_assign(nx_graph)
        if len(graphs) > 1:
                new_graphs = []
                for g in graphs:
                        subg = parallelize_assign(g)
                        new_graphs += subg
                graphs = new_graphs
        return graphs

def getSubtasks(nx_graph,slv):
        # analyze graph of specification and get complete split
        pruneOr(nx_graph,slv)
        subUntil(nx_graph)
        subAndLoop(nx_graph)
        graphs = parallelize(nx_graph)
        return graphs

def getSubtasks_assign(nx_graph,tasks,vrs,ts,agents):
        # analyze graph of specification and get complete split
        # TODO: make an option to prune OR or not!
        pruneOr_assign(nx_graph,vrs,ts,agents)
        subUntil_assign(nx_graph,tasks)
        subAndLoop_assign(nx_graph)
        graphs = parallelize_assign(nx_graph)
        return graphs

########
# The following set of function are used for generating formulas using the labels in a graph
# The main function is formulaFromGraph
########

def taskFormula(nx_graph,node):
        # capability and counts paired into a big string
        c_str = str(''.join(['('+x+','+str(y)+'),' for x,y in zip(nx_graph.node[node]['label']['caps'],nx_graph.node[node]['label']['counts'])])[0:-1])
        # string encoding the whole task
        t_str = 'T('+nx_graph.node[node]['label']['duration']+','+nx_graph.node[node]['label']['label']+',{'+c_str+'})'
        nx_graph.node[node]['formula'] = t_str

def alwEventFormula(nx_graph,node):
        # add formula to node with temporal operator G or F
        # there is only one successor node, so we only need to append G or F to the successor formula
        succ = nx_graph.successors(node)[0]
        nx_graph.node[node]['formula'] = nx_graph.node[node]['label']['label'] + '( ' + nx_graph.node[succ]['formula'] +' )'

def andOrFormula(nx_graph,node):
        # add formula to node with AND or OR
        label = nx_graph.node[node]['label']['label']
        if label == 'AND':
                form_op = ' && '
        else:
                form_op = ' || '
        # if we allow AND to be more than binary (which we do), we need to loop over the successors and them to the formula
        succs = nx_graph.successors(node)
        succs2 = [int(x) for x in succs]
        succs2.sort()
        succs = [str(x) for x in succs2]
        formula = nx_graph.node[succs[0]]['formula']
        for x in range(len(succs)-1):
                formula = formula+form_op+nx_graph.node[succs[x+1]]['formula']
        # since this is a binary operator, there should be exactly two successor nodes
        nx_graph.node[node]['formula'] = '( '+formula+' )'


def untilFormula(nx_graph,node):
        # add formula to node with Until
        label = nx_graph.node[node]['label']['label']
        succs = nx_graph.successors(node)
        succs2 = [int(x) for x in succs]
        succs2.sort()
        succs = [str(x) for x in succs2]
        # because we don't change nodes labeled until, the node is succs[0] Until succs[1]
        nx_graph.node[node]['formula'] = '( '+nx_graph.node[succs[0]]['formula']+' '+label+' '+nx_graph.node[succs[1]]['formula']+' )'


def taskAssignments(constraints, model):
    """
    Collects agent assignments to tasks from a Z3 model

    Args:
        constraints: a dictionary of Z3 variables (symbolic constants)
        model: a Z3 model object

    Returns:
        dict: a dictionary from tasks to agents
    """

    tasks = dict()
    # check all agent assignments
    for x in constraints['lambda_m_Agent_j'].values():
            # if agent assignment is true
            if model[x]:
                    # get agentnum and tasknum
                    const_string = str(x).split('_')
                    if const_string[1] in tasks:
                            tasks[const_string[1]].append(const_string[3])
                    else:
                            tasks[const_string[1]] = [const_string[3]]
    return tasks


def refineAssignments(constraints, slv, tasks, cap_excesses):
    """
    Refines a task assignment with a sequence of queries.
    Each query requires increasing one of the minimum capability excesses,
    without any of the other capability excess dropping below the minimum.

    This may increase individual capability excesses, even if the overall
    minimum cannot be increased.

    Note: this commits to the original task assignments, and does not
          try to decompose differently. It only adjusts the agent
          assignments.

    Args:
        constraints: a dictionary of Z3 variables (symbolic constants)
        slv: a Z3 Solver object
        tasks: the original task assignment
        cap_excesses: the capability excesses for each task

    Returns:
        z3 model: an updated z3 model for the last satisfiable check

    Preconditions:
        slv is in a sat state (has a model)

    Postconditions:
        slv is in an unsat state
          - constraints were added until unsat
    """

    print('Refining model to increase capability excesses.')
    model = slv.model()
    res = z3.sat
    ce_model = [model.eval(ce).as_long() for ce in cap_excesses]
    original_min_ce = min(ce_model)
    # guaranteed to be unsat eventually
    # could consider having an option to limit iterations
    num_refinements = 0
    while res != z3.unsat:
        # check model for smallest capability excess
        ce_model = [model.eval(ce).as_long() for ce in cap_excesses]
        min_ce = min(ce_model)
        indices = set([i for i, c in enumerate(ce_model) if c == min_ce])
        # require at least one of the smallest capability excesses to increase
        # without any of the others dropping below the minimum
        # stronger condition would only use ANDs
        # but then might fail even if we can increase some
        or_list = []
        and_list = []
        for i, ce in enumerate(cap_excesses):
            if i in indices:
                # want it to increase
                or_list.append(ce > min_ce)
                # but don't allow decrease
                and_list.append(ce >= min_ce)
            else:
                and_list.append(ce > min_ce)
        slv.add(And(Or(or_list), And(and_list)))
        res = slv.check()
        if res == z3.sat:
            model = slv.model()
            num_refinements += 1
        else:
            print(f'Updated {num_refinements} capability excesses. '
                  f'Minimum increased by {min_ce - original_min_ce}.')
    return model


def formulaFromGraph(nx_graph):
        # Input: network x graph with operators and tasks in labels
        # Output: a formula corresponding to the labels in the graph
        # Function adds a property "formula" to each node in the graph
        # The overall formula is given at the root node

        # list of nodes to translate
        to_translate = [x for x in nx_graph.nodes() if nx_graph.out_degree(x) != 0]
        # get list of leaves
        leaves = [x for x in nx_graph.nodes() if nx_graph.out_degree(x) == 0]
        # add formula to leaves
        for x in leaves:
                taskFormula(nx_graph,x)
        # now work on nodes in to_translate
        # loop as long as to_translate is not empty
        while to_translate:
                x = to_translate.pop(0)
                # check if successors are left in to_translate:
                if list(set(nx_graph.successors(x)) & set(to_translate)):
                        # successors still need to be translated, push x back onto the stack
                        to_translate.append(x)
                else:
                        # parse always or eventually
                        if nx_graph.node[x]['label']['label'][0] in ['F','G']:
                                alwEventFormula(nx_graph,x)
                        # binary 'AND' or 'OR'
                        elif nx_graph.node[x]['label']['label'][0] in ['A','O']:
                                andOrFormula(nx_graph,x)
                        # 'Until'
                        elif nx_graph.node[x]['label']['label'][0] in ['U']:
                                untilFormula(nx_graph,x)
                        # there shouldn't be anything here
                        else:
                                print("Error: unexpected operator in nx_graph")
                                pass

        # identify root node -- since we are working with trees, this shold be unique
        root = [x for x in nx_graph.nodes() if nx_graph.in_degree(x)==0]

        # get formula from root node
        formula = nx_graph.node[root[0]]['formula']

        return formula

def combine_graphs(graph_list):
        # take the items in graph_list, and combine them with conjunction

        out_graph = nx.DiGraph()
        out_graph.add_node('root')
        out_graph.node['root']['label'] = {'label':'AND'}

        for x in graph_list:
                # check if out_graph already exists
                out_graph = nx.disjoint_union(out_graph,x)

                # find nodes with in_degree == 0
                indegs = [y for y in out_graph.nodes() if out_graph.in_degree(y)==0 ]

                # if there are more than one:
                if len(indegs)>1:
                        # add edge from new node to roots of old graphs
                        for y in indegs:
                                if y!=0:
                                        out_graph.add_edge(0,y)

        return out_graph

def label_assignment(nx_graph,tasks):
        # function to label nodes of a graph according to the task assignments
        # assignments are propagated from the leaves up to the root

        # label leaves with agent assignments
        leaves = [x for x in nx_graph.nodes() if nx_graph.out_degree(x)==0]
        for x in leaves:
                if x in tasks:
                        nx_graph.node[x]['agents'] = set(tasks[x])
                else:
                        # empty set if no assignment (happens because of 'OR')
                        nx_graph.node[x]['agents'] = set()

        # now add list of agents to all nodes
        nodes = [x for x in nx_graph.nodes() if nx_graph.out_degree(x)!=0]
        while len(nodes)>0:
                x = nodes.pop(0)
                # check if any successors are still in the list of nodes
                res = all(y not in nodes for y in nx_graph.successors(x))
                if res:
                        combine_agent_sets(nx_graph,x)
                else:
                        nodes.append(x)
        return nx_graph

def combine_agent_sets(nx_graph,node):
        # for a node in a graph, label it with the task assignment from its successors

        # all successors have been labeled
        nx_graph.node[node]['agents'] = set()
        for y in nx_graph.successors(node):
                nx_graph.node[node]['agents'] = nx_graph.node[node]['agents'].union(nx_graph.node[y]['agents'])

def get_final_assignment(nx_graph):

        # get root node of nx_graph
        root = [node for node in nx_graph.nodes() if len(nx_graph.predecessors(node))==0][0]
        agents = nx_graph.node[root]['agents']

        # label all leaves with the root agent set
        leaves = [node for node in nx_graph.nodes() if len(nx_graph.successors(node))==0]
        for leaf in leaves:
                nx_graph.node[leaf]['agents'] = agents
