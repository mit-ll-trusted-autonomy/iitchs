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

from copy import deepcopy
import itertools
import networkx as nx
from typing import List, Set, Tuple

from lomap.classes.ts import Ts

from catl_planning.utils import getLeaves_nx, plotAST_nx

class ReducedTs:
    """
    This class represents a TransitionSystem that has been modified by
    removing unnecessary states. This is the output of reduce_ts. It is 
    intended for use with the decomp option, where a subteam might not 
    need to know about every state.

    It stores a pointer to the original transition system at self.orig_ts, 
    and an updated networkx graph at self.g.
    """

    def __init__(self, orig_ts:Ts):
        self.g = deepcopy(orig_ts.g)
        self.orig_ts = orig_ts


def reduce_ts(ts:Ts, spec:str, agents:List[Tuple[str, Set[str]]])->ReducedTs:
        """
        Reduces the size of a Ts by removing states. Any states that are not relevant
        to the spec are removed, and edge weights are adjusted accordingly.

        For example, consider the following path:
          q1 -weight:2-> q2 -weight:3-> q3
        where we want to remove q2, the updated path would be
          q1 -weight:5-> q3

        Args:
                ts: the transition system to modify
                spec: the specification
                agents: the agents to be used for this specification
                        a list of tuples (state, {capabilities})

        Returns:
                ReducedTs: a new transition system
        """

        new_ts = ReducedTs(ts)
        G = plotAST_nx(spec)
        tasks = getLeaves_nx(G)
        all_labels = {G.node[t]['label']['label'] for t in tasks}
        all_starting_states = {a[0] for a in agents} # used to ensure we don't remove a starting state
        print(f'Removing all states without one of the following labels: {all_labels}')
        keep_states = set()
        # a subset of keep_states, these states are only kept so they can
        # be treated as obstacles
        obstacle_states = set()
        remove_states = set()
        for state in new_ts.g.nodes():
                if 'grave' in new_ts.g.node[state]['prop'] or \
                   state in all_starting_states or \
                   len(new_ts.g.node[state]['prop'] & all_labels) != 0:
                        # keep the following kinds of states:
                        # * grave state
                        # * states with labels appearing in the spec
                        keep_states.add(state)
                elif len(new_ts.g.in_edges(state)) == 0 and \
                     len(new_ts.g.out_edges(state)) == 0:
                        # also keep:
                        # * obstacle states (need to be kept so they can be routed around)
                        keep_states.add(state)
                        obstacle_states.add(state)
                else:
                        remove_states.add(state)

        # find shortest path between each pair of states and add new edges
        # that bypass nodes marked for removal
        for src, dst in itertools.combinations(keep_states, 2):
                if src in obstacle_states or dst in obstacle_states:
                        # by definition, there are no paths to/from obstacle states
                        continue
                try:
                        path = nx.shortest_path(new_ts.g, src, dst, weight='weight')
                except nx.exception.NetworkXNoPath as e:
                        # If this comes up in practice, can try to handle by
                        #  * re-decomposing; or
                        #  * considering reachability/distance when assigning agents
                        raise RuntimeError(f'No path between {src} and {dst}. Decomposition infeasible.')

                to_keep_indices = [i for i in range(len(path)) if path[i] in keep_states]

                # add edges between keep states that goes around all states to be removed
                for idx0, idx1 in zip(to_keep_indices, to_keep_indices[1:]):
                        # collect all edges
                        collapsed_edges = []
                        new_weight = 0
                        for i in range(idx0, idx1):
                                edata = new_ts.g.get_edge_data(path[i], path[i+1])
                                new_weight += edata['weight']
                                if 'collapsed_edges' in edata:
                                        # this is an already collapsed edge
                                        assert not ts.g.has_edge(path[i], path[i+1])
                                        collapsed_edges.extend(edata['collapsed_edges'])
                                else:
                                        # this is an original edge
                                        assert ts.g.has_edge(path[i], path[i+1])
                                        collapsed_edges.append((path[i], path[i+1], edata))

                        if not ts.g.has_edge(path[idx0], path[idx1]):
                                # create a new edge
                                new_ts.g.add_edge(path[idx0], path[idx1],
                                                weight=new_weight,
                                                collapsed_edges=collapsed_edges)

                                # add edge in other direction
                                reverse_collapsed_edges = []
                                for edge in reversed(collapsed_edges):
                                        assert ts.g.has_edge(edge[1], edge[0])
                                        redge = (edge[1], edge[0], edge[2])
                                        reverse_collapsed_edges.append(redge)
                                new_ts.g.add_edge(path[idx1], path[idx0],
                                                weight=new_weight,
                                                collapsed_edges=reverse_collapsed_edges)

        # now remove unnecessary states
        for state in remove_states:
                new_ts.g.remove_node(state)

        print(f'Removed the following states: {remove_states}')

        return new_ts
