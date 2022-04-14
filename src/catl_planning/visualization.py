# SPDX-License-Identifier: BSD-3-Clause
#
# Contributors:
#   Cristian Ioan Vasile (cvasile@bu.edu) (cvasile@lehigh.edu)
#   James Usevitch (james.usevitch@ll.mit.edu)
#   Makai Mann (makai.mann@ll.mit.edu)

import itertools as it

import numpy as np
import shapely.geometry as geom
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib.collections import LineCollection



def drawPoint(viewport, pointx,pointy, color, style=None):
    '''Draws a point in the planar environment.'''
    if style:
        viewport.plot(pointx, pointy, color=color, **style)
    else:
        viewport.scatter(pointx, pointy, color=color)

def drawRegion(viewport, shape, style=None, text=None, textStyle=None):
    '''Draws a polygonal region in a planar environment.'''
    polygon = geom.Polygon(shape)
    x, y = list(zip(*polygon.exterior.coords))
    if style:
        style['facecolor'] = style.get('facecolor', 'white')
        style['edgecolor'] = style.get('edgecolor', 'black')
    else:
        style = {'facecolor': 'white', 'edgecolor':'black'}
    viewport.fill(x, y, alpha=0.75, **style)
    if text:
        x, y = polygon.centroid.coords[0]
        if textStyle:
            textStyle['horizontalalignment'] = \
                                  textStyle.get('horizontalalignment', 'center')
            textStyle['verticalalignment'] = \
                                    textStyle.get('verticalalignment', 'center')
            textStyle['fontsize'] = textStyle.get('fontsize', 12)
        else:
            textStyle = {'horizontalalignment' : 'center',
                         'verticalalignment' : 'center', 'fontsize' : 12}
        viewport.text(x, y, text, **textStyle)
    return polygon.centroid.coords[0]

def drawGraph(viewport, g, node_color='blue', edge_color='black', zorder=2):
    '''Plots the given graph in the viewport.'''
    x, y = list(zip(*[d['position'] for _, d in g.nodes(data=True)]))
    viewport.scatter(x, y, c=node_color, zorder=zorder+1)

    lines = [(g.node[u]['position'], g.node[v]['position'])
                                                for u, v in g.edges() if u != v]
    artist = LineCollection(lines, colors=edge_color, zorder=zorder)
    viewport.add_collection(artist)

def drawPolicy(viewport, solution, color='black', alpha_min=1.0, zorder=2):
    '''Draws the solution path with a fading effect.'''
    if alpha_min == 1.0:
        transparency = it.repeat(1.0)
    else:
        transparency = np.linspace(alpha_min, 1.0, len(solution)-1)

    for u, v, a in it.izip(solution, solution[1:], transparency):
        dx, dy = v.x - u.x, v.y - u.y
        plt.arrow(u.x, u.y, dx, dy, hold=True, color=color, alpha=a,
                  length_includes_head=True, head_width=0.08, zorder=zorder)

def show_environment(ts, save=None, figsize=None):
    '''Draws the environment and optionally save it to a file.'''
    fig = plt.figure(figsize=figsize)
    viewport = fig.add_subplot(111, aspect='equal')

    for u, d in ts.g.nodes(data=True):
        center = drawRegion(viewport, shape=d['shape'],
                            style={'facecolor': d['color']}, text=u)
        if 'position' not in d:
            d['position'] = (center[0], center[1] - 0.5)

    drawGraph(viewport, ts.g)

    if save is not None:
        plt.subplots_adjust(left=0.05, bottom=0.05, right=0.98, top=0.98,
                            wspace=0, hspace=0)
        plt.savefig(save, dpi=fig.dpi)

    plt.show()

def show_world(ts,fig):
    '''Draws a polygonal regions in a planar environment with labels.'''
    viewport = fig.add_subplot(111, aspect='equal')

    for u, d in ts.g.nodes(data=True):
        center = drawRegion(viewport, shape=d['shape'],
                            style={'facecolor': d['color']})# text=u)
        if 'position' not in d:
            d['position'] = (center[0], center[1] - 0.5)
    return fig,viewport

def show_environment_agents(ts,agents,fig,viewport, save=None, figsize=None):
    '''Draws agents in the environment.'''
    sys = ts.g.nodes('q'+str(agents[0][0]))
    sys = np.asarray(sys)
    cent_x = 0.0
    cent_y = 0.0
    x_vals = []
    y_vals = []
    for i in range(0,len(agents)):
        indiv_sys = sys[sys[:,0] == 'q'+str(agents[i][0])]
        old_x = cent_x
        old_y = cent_y
        cent_x = indiv_sys.item(1)['position'][0]
        cent_y = indiv_sys.item(1)['position'][1]

        cap = agents[i][2]
        print (cap)
        num = agents[i][3]
        if i ==1:
            num -=1


        if cap == 3:
            color = 'r'
            marker = 'o'
            r = 0.9
        elif cap == 6:
            color = 'k'
            marker = 's'
            r = 0.9
        elif cap == 5:
            color = 'w'
            marker = '*'
            r = 0.9
        elif cap == 12:
            color = 'm'
            marker = 'v'
            r = 0.9
        elif cap == 10:
            color = 'c'
            marker = 'p'
            r = 0.9
        else:

            color = 'k'
            marker = '.'
            r = 0.4

        ang_div = 2*np.divide(np.pi,num)
        randx = np.random.rand()
        randy = np.random.rand()
        for a in range(0,num):
             randx = np.random.rand()
             randy = np.random.rand()
             pointx = cent_x+ r*(0.9*randx + 0.1*np.sign(randx))
             pointy = cent_y + r*(0.9*randy + 0.1*np.sign(randy))
            #print(pointx,pointy)
             viewport.scatter(pointx,pointy,s=120,color=color,marker=marker, edgecolor='k',linewidth=2.0,zorder=2)
    #drawPoint(viewport, point, color, style=None)
    markerList = [mlines.Line2D([], [], color='r', marker='o', linestyle='None',
                        markersize=15, label='{Vis, IR}'),
                 mlines.Line2D([], [], color='k', marker='s', linestyle='None',
                        markersize=15, label='{Vis, UV}'),
                 mlines.Line2D([], [], color='w', marker='*', linestyle='None',
                        markersize=15, label='{Vis, Mo}'),
                 mlines.Line2D([], [], color='m', marker='v', linestyle='None',
                        markersize=15, label='{UV, Mo}'),
                mlines.Line2D([], [], color='c', marker='p', linestyle='None',
                        markersize=15, label='{UV, IR}')]



    #plt.legend(handles=markerList)
    if save is not None:
        plt.subplots_adjust(left=0.05, bottom=0.05, right=0.98, top=0.98,
                            wspace=0, hspace=0)
        plt.savefig(save, dpi=fig.dpi)

    #plt.close()
    return fig


def show_transition_agents(ts,agents,plt, save=None, figsize=None):
    '''Draws edges in the environment.'''
    sys = ts.g.nodes('q'+str(agents[0][0]))
    sys = np.asarray(sys)
    cent_x1 = 0.0
    cent_y1 = 0.0
    cent_x2 = 0.0
    cent_y2 = 0.0

    for i in range(0,len(agents)):
        indiv_sys1 = sys[sys[:,0] == 'q'+str(agents[i][0])]
        indiv_sys2 = sys[sys[:,0] == 'q'+str(agents[i][1])]
        old_x1 = cent_x1
        old_y1 = cent_y1
        old_x2 = cent_x2# for a in range(0,num):
        old_y2 = cent_y2
        cent_x1 = indiv_sys1.item(1)['position'][0]
        cent_y1 = indiv_sys1.item(1)['position'][1]
        cent_x2 = indiv_sys2.item(1)['position'][0]
        cent_y2 = indiv_sys2.item(1)['position'][1]

        x_vals = []
        y_vals = []

        cap = agents[i][3]
        num = agents[i][4]
        if cap == 3:
            color = 'r'
            marker = 'o'
            r = 1
        elif cap == 9:
            color = 'k'
            marker = 's'
            r = 1
        elif cap == 5:
            color = 'w'
            marker = '*'
            r = 1
        elif cap == 12:
            color = 'm'
            marker = 'v'
            r = 1
        elif cap == 10:
            color = 'c'
            marker = 'p'
            r = 1# for a in range(0,num):
        else:

            color = 'k'
            marker = '.'
            r = 1

        offset = np.divide(8,num)
        delx = cent_x2 - cent_x1
        dely = cent_y2 - cent_y1
        norm = np.sqrt((delx*delx)+(dely*dely))
        cx = np.divide(delx,norm)*10
        cy = np.divide(dely,norm)*10
        r = 10
        pos = 0
        if delx < 0.01 and dely< 0.01:
            #Self Loop
            x_vals = []
            y_vals = []
            for a in range(0,num):
                randx = np.random.rand()
                randy = np.random.rand()
                pointx = cent_x1 + r*randx
                pointy = cent_y1 + r*randy# for a in range(0,num):
                #print(pointx,pointy)
                x_vals.append(pointx)
                y_vals.append(pointy)
            plt.scatter(x_vals,y_vals,color=color,marker=marker,linewidth=5.0,zorder=2)

        else:
            for a in range(0,num):
                randx = np.random.rand()
                randy = np.random.rand()
                delx = (cent_x2+(randx*0.5)+num) - (cent_x1+(randx*0.5)+num)
                dely = (cent_y2+(randy*0.5)+num) - (cent_y1+(randy*0.5)+num)
                plt.arrow(cent_x1+(randx*0.5), cent_y1+(randy*0.5), (delx*.8), (dely*.8),head_width=0.5, head_length=0.5, fc=color, ec=color)

    return plt
