
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

from __future__ import print_function
import io
from catl_planning.route_planning import route_planning
import yaml
import networkx as nx
import numpy as np
import lomap as lm
import csv
import time
import copy

def generateRandomTS_seed(tsname,numStates,edgeProb,labelNames,labelProbs,edgeWeightRange,seed):
    #open yaml file
    np.random.seed(seed)
    stream = file(tsname+'.yaml','w')
    randomGraph = nx.DiGraph(nx.gnp_random_graph(numStates,edgeProb))
    solitary=[ n for n,d in randomGraph.degree_iter() if d==0 ]
    # print 'solitary: ', solitary
    randomGraph.remove_nodes_from(solitary)
    node_nums = randomGraph.nodes()
    # print 'remaining: ', node_nums
    #Generate random labelings
    labelSet = np.random.choice(labelNames,size=len(node_nums),p=labelProbs)

    #Apply node lables
    tsnodes = dict()
    for j in range(len(node_nums)):
        randomGraph.node[node_nums[j]]['prop'] = set([labelSet[j]])
    #generate Edge weights
    tsedges = dict()
    nxedges = randomGraph.edges()
    edgeWeightSet = np.random.random_integers(edgeWeightRange[0],edgeWeightRange[1],len(nxedges))
    for k in range(len(nxedges)):
        randomGraph[nxedges[k][0]][nxedges[k][1]]['weight'] = edgeWeightSet[k]
    #Add in self-loops
    randomGraph.add_weighted_edges_from([(node,node,1) for node in randomGraph.nodes()])


    tsModel = lm.Ts(tsname,directed=True,multi=False);
    tsModel.g = randomGraph
    tsModel.init=[tsModel.g.nodes()[0]]
    tsModel.current=tsModel.init
    tsModel.final=tsModel.init
    tsModel.save(tsname)
    return tsModel

def generateRandomTS(tsname,numStates,edgeProb,labelNames,labelProbs,edgeWeightRange):
    #open yaml file
    stream = file(tsname+'.yaml','w')
    randomGraph = nx.DiGraph(nx.gnp_random_graph(numStates,edgeProb))
    solitary=[ n for n,d in randomGraph.degree_iter() if d==0 ]
    # print 'solitary: ', solitary
    randomGraph.remove_nodes_from(solitary)
    node_nums = randomGraph.nodes()
    # print 'remaining: ', node_nums
    #Generate random labelings
    labelSet = np.random.choice(labelNames,size=len(node_nums),p=labelProbs)

    #Apply node lables
    tsnodes = dict()
    for j in range(len(node_nums)):
        randomGraph.node[node_nums[j]]['prop'] = set([labelSet[j]])
    #generate Edge weights
    tsedges = dict()
    nxedges = randomGraph.edges()
    edgeWeightSet = np.random.random_integers(edgeWeightRange[0],edgeWeightRange[1],len(nxedges))
    for k in range(len(nxedges)):
        randomGraph[nxedges[k][0]][nxedges[k][1]]['weight'] = edgeWeightSet[k]
    #Add in self-loops
    randomGraph.add_weighted_edges_from([(node,node,1) for node in randomGraph.nodes()])


    tsModel = lm.Ts(tsname,directed=True,multi=False);
    tsModel.g = randomGraph
    tsModel.init=[tsModel.g.nodes()[0]]
    tsModel.current=tsModel.init
    tsModel.final=tsModel.init
    tsModel.save(tsname)
    return tsModel

def generateRandomGridTS_seed(tsname,dimTuple,labelNames,edgeWeightRange,mult,seed):

    np.random.seed(seed)
    randomGraph = nx.DiGraph(nx.grid_graph(dimTuple))
    node_nums = randomGraph.nodes()
    #Generate random labelings
    #Make sure at least one of each label is present
    labelSet = [labelNames[-1]]*len(node_nums)
    randLabel = np.random.choice(len(node_nums),mult*(len(labelNames)-1),replace=False)

    for x in range(mult*(len(labelNames)-1)):
        labelSet[randLabel[x]] = labelNames[x%(len(labelNames)-1)]



    #Apply node lables
    for j in range(len(node_nums)):
        randomGraph.node[node_nums[j]]['prop'] = set([labelSet[j]])
    #generate Edge weights
    nxedges = randomGraph.edges()
    edgeWeightSet = np.random.random_integers(edgeWeightRange[0],edgeWeightRange[1],len(nxedges))
    for k in range(len(nxedges)):
        randomGraph[nxedges[k][0]][nxedges[k][1]]['weight'] = edgeWeightSet[k]
    #Add in self-loops
    randomGraph.add_weighted_edges_from([(node,node,1) for node in randomGraph.nodes()])

    # TODO: Add grave state
    # Add grave state
    randomGraph.add_node('grave')
    randomGraph.node['grave']['prop'] = 'grave'
    # Add edges to grave state
    for x in node_nums:
        randomGraph.add_edge(x,'grave',weight = 1)
        randomGraph.add_edge('grave',x,weight = 1000)


    tsModel = lm.Ts(tsname,directed=True,multi=False);
    tsModel.g = randomGraph
    tsModel.init=[tsModel.g.nodes()[0]]
    tsModel.current=tsModel.init
    tsModel.final=tsModel.init
    tsModel.save(tsname)
    return tsModel

def generateRandomGridTS(tsname,dimTuple,edgeProb,labelNames,labelProbs,edgeWeightRange):
    #open yaml file
    # stream = file(tsname+'.yaml','w')
    randomGraph = nx.DiGraph(nx.grid_graph(dimTuple))
    # solitary=[ n for n,d in randomGraph.degree_iter() if d==0 ]
    # # print 'solitary: ', solitary
    # randomGraph.remove_nodes_from(solitary)
    node_nums = randomGraph.nodes()
    # print 'remaining: ', node_nums
    #Generate random labelings
    #Make sure at least one of each label is present
    # print labelNames
    # print type(labelNames)
    labelSet = np.random.permutation(list(labelNames) + list(np.random.choice(labelNames,size=(len(node_nums)-len(labelNames)),p=labelProbs)))

    #Apply node lables
    tsnodes = dict()
    for j in range(len(node_nums)):
        randomGraph.node[node_nums[j]]['prop'] = set([labelSet[j]])
    #generate Edge weights
    tsedges = dict()
    nxedges = randomGraph.edges()
    edgeWeightSet = np.random.random_integers(edgeWeightRange[0],edgeWeightRange[1],len(nxedges))
    for k in range(len(nxedges)):
        randomGraph[nxedges[k][0]][nxedges[k][1]]['weight'] = edgeWeightSet[k]
    #Add in self-loops
    randomGraph.add_weighted_edges_from([(node,node,1) for node in randomGraph.nodes()])


    tsModel = lm.Ts(tsname,directed=True,multi=False);
    tsModel.g = randomGraph
    # print tsModel.g.nodes()
    tsModel.init=[tsModel.g.nodes()[0]]
    tsModel.current=tsModel.init
    tsModel.final=tsModel.init
    tsModel.save(tsname)
    return tsModel


def generateRandomAgents(stateList,capabilityList,numCapabilitiesPerAgent,numAgentsPerClass, numClasses):
    agents = []
    # print
    for j  in range(numClasses):
        modCapList =  copy.copy(capabilityList)
        #Ensure capability coverage
        modCapList.pop(j%len(capabilityList))
        randClass = set({capabilityList[j%len(capabilityList)]}) | set(np.random.choice(modCapList,size=numCapabilitiesPerAgent-1,replace=False))
        agents += [(stateList[np.random.choice(range(len(stateList)))], randClass) for k in range(numAgentsPerClass)]
    return agents

def generateRandomAgents_seed(stateList,capabilityList,numCapabilitiesPerAgent,numAgentsPerClass, numClasses, seed):
    np.random.seed(seed)
    agents = []
    # print
    for j  in range(numClasses):
        modCapList =  copy.copy(capabilityList)
        #Ensure capability coverage
        modCapList.pop(j%len(capabilityList))
        randClass = set({capabilityList[j%len(capabilityList)]}) | set(np.random.choice(modCapList,size=numCapabilitiesPerAgent-1,replace=False))
        agents += [(stateList[np.random.choice(range(len(stateList)))], randClass) for k in range(numAgentsPerClass)]
    return agents

def generateRandomAgentsClass_seed(state,classList,numAgents,seed):
    np.random.seed(seed)
    numClasses = len(classList)
    classVec = np.random.randint(0,numClasses,numAgents)
    agents = [(state,classList[x]) for x in classVec]
    return agents

def monteCarloTests(experimentName,specification,numTrials,tsname,numStates,edgeProb,labelNames,
                labelProbs,edgeWeightRange,capabilityList,numCapabilitiesPerAgent,
                numAgentsPerClass, numClasses,solFileName,robust=True,MIPFocus=0):
    trialTimes = []
    trialCons = []
    trialVars = []
    trialVals = []
    trialIdx = 0
    csvName = experimentName+'.csv'
    writer = csv.writer(open(csvName,'w'))
    writer.writerow(['Run Time (s)','Linear Constraints','Variables','Objective Value'])
    while trialIdx < numTrials:
        print ("Trial: ",trialIdx+1)
        print ("StateTuple: ", numStates)
        tsnameTrial = tsname + '_' + str(trialIdx)+ '.yaml'
        solnameTrial = solFileName + '_' + str(trialIdx)
        print ("building TS")
        if len(numStates)==1:
            tsTrial = generateRandomTS(tsnameTrial,numStates,edgeProb,labelNames,labelProbs,edgeWeightRange)
        else:
            tsTrial = generateRandomGridTS(tsnameTrial,copy.copy(numStates),edgeProb,labelNames,labelProbs,edgeWeightRange)
        print ("building agents")
        agentsTrial = generateRandomAgents(tsTrial.g.nodes(),capabilityList,numCapabilitiesPerAgent,numAgentsPerClass, numClasses)
        print ("starting Optimization")
        t = time.time()
        # try:
        gurModel = route_planning(tsTrial, agentsTrial, specification, filetosave=solnameTrial,show_output=True,robust=robust,MIPFocus=MIPFocus)
        runTime = time.time()-t
        print ("Optimization Complete")
        if hasattr(gurModel, "ObjVal"):
            trialTimes.append(runTime)
            trialCons.append(gurModel.NumConstrs)
            trialVars.append(gurModel.NumVars)
            trialVals.append(gurModel.ObjVal)
            writer.writerow([runTime,gurModel.NumConstrs,gurModel.NumVars,gurModel.ObjVal])
            trialIdx += 1
            print (runTime)
            print (gurModel.NumConstrs)
            print (gurModel.NumVars)
            print (gurModel.ObjVal)
        # except:
            # continue
