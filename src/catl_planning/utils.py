# -*- coding: utf-8 -*-

"""
Collection of basic utility functions.

Primary Contributor(s): 
  Makai Mann (makai.mann@ll.mit.edu)
"""

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

from catl import CATLFormula
from catl import Operation as CATLOperation


# Functions for converting between integer and string state representations
def int2state(state:int)->str:
    '''
    Takes a state represented as an integer and returns the string version.
    IITCHS convention uses q<integer> for state names.

    Args:
        state: an integer state

    Returns:
        str: the string representation
    '''
    return f'q{state}'

def state2int(state:str)->int:
    '''
    Takes a state represented as a string and returns the integer version.
    IITCHS convention uses q<integer> for state names.

    Args:
        state: a state string

    Returns:
        int: the integer representation
    '''
    assert state[0] == 'q'
    return int(state[1:])


# CATL formula manipulation utilities

def plotAST_nx(specification):
    ast = CATLFormula.from_formula(specification)
    dot = nx.DiGraph()
    idx = 0
    cmdStr = 'ast'
    dot.add_node(str(idx))
    dot.node[str(idx)]['label'] = propMap(eval(cmdStr+'.op'),cmdStr,ast)
    successor, sString = getProps(ast)
    numChildren = len(successor)
    maxIdx = idx

    for chkIdx in range(numChildren):
        # TODO handle this special case more elegantly
        children = eval(f'{cmdStr}.{sString}')
        if isinstance(children, list):
            n = len(children)
        else:
            assert sString == 'child'
            n = 1
        if n>1:
            newCmd = cmdStr+'.'+sString+'['+str(chkIdx)+']'
        else:
            newCmd = cmdStr+'.'+sString
        dot, maxIdx = addNode_nx(dot, newCmd,0,maxIdx,ast)

    return dot

def getProps(obj):
    if "op" in dir(obj):
        if obj.op in (CATLOperation.NOT, CATLOperation.EVENT, CATLOperation.ALWAYS):
            successor = [obj.child]
            sString = 'child'
        elif obj.op in (CATLOperation.AND, CATLOperation.OR):
            successor = obj.children
            sString = 'children'
        elif obj.op in (CATLOperation.IMPLIES, CATLOperation.UNTIL):
            successor = [obj.left, obj.right]
            sString = ['left', 'right']
        else:
            successor = []
            sString = ''
        return successor, sString

def addNode_nx(digraph,cmdStr,pIdx,maxIdx,ast):

    idx = maxIdx+1
    digraph.add_node(str(idx))
    digraph.node[str(idx)]['label'] = propMap(eval(cmdStr+'.op'),cmdStr,ast)
    digraph.add_edge(str(pIdx),str(idx))
    maxIdx = idx

    successor, sString = getProps(eval(cmdStr))
    if isinstance(successor,list):
        numChildren = len(successor)
        for chkIdx in range(numChildren):
            if 'left' in sString:
                newCmd =cmdStr+'.'+sString[chkIdx]
            elif sString=='children':
                newCmd = cmdStr+'.'+sString+'['+str(chkIdx)+']'
            else:
                newCmd = cmdStr+'.'+sString
            digraph, maxIdx = addNode_nx(digraph, newCmd,idx,maxIdx,ast)
    else:
        newCmd = cmdStr+'.'+sString
        digraph, maxIdx = addNode_nx(digraph, newCmd,idx,maxIdx,ast)

    return digraph, maxIdx

def getLeaves_nx(G):
    # requires string pointing to dot file with graphviz graph object

    # get leaves of graph
    leaves = [x for x in G.nodes() if G.out_degree(x)==0 and G.in_degree(x)==1]

    return leaves


def propMap(prop_num,cmdStr,ast):

    if prop_num == 0: # NOP
        propString = {'label': 'NOP'}
    elif prop_num == 1: # NOT
        propString = {'label': '!'}
    elif prop_num == 2: # OR
        propString = {'label': 'OR'}
    elif prop_num == 3: # AND
        propString = {'label': 'AND'}
    elif prop_num == 4: # IMPLIES
        propString = {'label': '-->'}
    elif prop_num == 5: # UNTIL
        propString = {'label': 'U ['+str(eval(cmdStr+'.low'))+', '+str(eval(cmdStr+'.high'))+']'}
    elif prop_num == 6: # EVENT
        propString = {'label': 'F ['+str(eval(cmdStr+'.low'))+', '+str(eval(cmdStr+'.high'))+']'}
    elif prop_num == 7: # ALWAYS
        propString = {'label': 'G ['+str(eval(cmdStr+'.low'))+', '+str(eval(cmdStr+'.high'))+']'}
    elif prop_num == 8: # PRED
        countList = [i.count for i in eval(cmdStr+'.capability_requests')]
        capList = [i.capability for i in eval(cmdStr+'.capability_requests')]
        propString = {'label': str(eval(cmdStr+'.proposition')), 'duration': str(eval(cmdStr+'.duration')),'caps':capList,'counts':countList}
    elif prop_num == 9: # BOOL
        propString = {'label': 'BOOL'}
    return propString
