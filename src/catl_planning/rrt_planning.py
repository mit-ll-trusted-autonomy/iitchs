# -*- coding: utf-8 -*-

"""
Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)

Author: Zachary Serlin (zserlin@bu.edu)
"""

#--------------------------------------------------------------------------------

#LL Copyright
#This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.
#© 2019 Massachusetts Institute of Technology.
#The software firmware is provided to you on an As-Is basis
#Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above. Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
#LL Copyright

#--------------------------------------------------------------------------------

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

#--------------------------------------------------------------------------------

import matplotlib.pyplot as plt
import random
import math
import copy
import numpy as np
import time

show_animation = True


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea, expandDis=1.0, goalSampleRate=5, maxIter=500,num_agents=1,past_paths=None,agentnum=0,escape_time=2,box_bounds_obstacleList=None):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0],start[1],t=0)
        self.end = Node(goal[0],goal[1])
        self.bounds = randArea
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.box_bounds_obstacleList = box_bounds_obstacleList 
        self.num_agents = num_agents
        self.past_paths = past_paths
        self.sigma = 50
        self.agentnum=agentnum
        self.escape_time = escape_time

    def Planning(self,safe_size,agent_size,animation=True):
        """
        Pathplanning
        animation: flag for animation on or off
        """
        self.start_clock = time.time()
        self.nodeList = [self.start]
        saved_start = self
        count = 0
        restarted = 0
        while True:
            #If we get stuck - restart
            if count > 0:
                self.sigma = self.sigma + 30
                if count > 10:
                    self = saved_start
                    count = 0
                    restarted = 1
            else:
                self.sigma = 2
                restarted = 0
            '''
            if count > 5:
                saved_start.sigma += self.sigma
                #print('no solution')
                self = saved_start
                restarted = 1
            '''

            rand_num = random.randint(0, 100)
            # Random Sampling
            if rand_num > self.goalSampleRate:
                print_out = 0
                if restarted == 0:
                    if rand_num < 200:
                        rnd = [np.random.normal(self.end.x, self.sigma), np.random.normal(
                            self.end.y, self.sigma)]
                    else:
                        rnd = [random.uniform(self.bounds[0], self.bounds[1]), random.uniform(
                            self.bounds[2], self.bounds[3])]
                if restarted == 1:
                    rnd = [random.uniform(self.bounds[0], self.bounds[1]), random.uniform(
                        self.bounds[2], self.bounds[3])]
            else:
                rnd = [self.end.x, self.end.y]
                if time.time()-self.start_clock >self.escape_time:
                    path = []
                    '''
                    plot_bounds = [0, 140, 0, 60]
                    plt.plot(self.start.x, self.start.y, "ok")
                    plt.plot(self.end.x, self.end.y, "xk")
                    PlotCircle(self.start.x, self.start.y,agent_size,plot_bounds)
                    PlotCircle(self.end.x, self.end.y,agent_size,plot_bounds)
                    plt.axis(plot_bounds)
                    plt.grid(True)
                    for (x, y, size) in self.obstacleList:
                        PlotCircle(x, y, size,plot_bounds)
                    if self.past_paths != None:
                        for i in range(0,len(self.past_paths)):
                            for j in range(0,len(self.past_paths[i])):
                                if j == 0:
                                    PlotCircle(self.past_paths[i][j][0],self.past_paths[i][j][1],agent_size,plot_bounds)
                                    plt.plot(self.past_paths[i][j][0],self.past_paths[i][j][1],"sr")
                                else:
                                    plt.plot(self.past_paths[i][j][0],self.past_paths[i][j][1],".r")
                    plt.pause(0.01)
                    plt.grid(True)
                    plt.pause(0.01)  # Need for Mac
                    plt.show(block=False)
                    plt.pause(0.1)
                    plt.waitforbuttonpress()
                    '''
                    print('returned empty')
                    return path
                print_out = 0
                #print(self.obstacleList,self.end.x,self.end.y,self.start.x,self.start.y)

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind
            newNode.t += 1
            if print_out == 1:
                print(self.__CollisionCheck(newNode, self.obstacleList,agent_size),self.__AgentCollisionCheck(newNode, safe_size,agent_size, self.past_paths),self.__BoundsCollisionCheck(newNode, self.bounds))
            
            if self.box_bounds_obstacleList == None:
                if not self.__CollisionCheck(newNode, self.obstacleList,agent_size):
                    count += 1
                    continue
            else:
                if not self.__BoxCollisionCheck(newNode, self.box_bounds_obstacleList,agent_size) and not self.__CollisionCheck(newNode, self.obstacleList,agent_size):
                    count += 1
                    continue

            if not self.__AgentCollisionCheck(newNode, safe_size,agent_size, self.past_paths):
                count +=1
                continue

            if not self.__BoundsCollisionCheck(newNode, self.bounds):
                count +=1
                continue

            count = 0

            self.nodeList.append(newNode)

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                #print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def DrawGraph(self,bounds, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")
        for (x, y, size) in self.obstacleList:
            self.PlotCircle(x, y, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis(bounds)
        plt.grid(True)
        plt.pause(0.01)

    def PlotCircle(self, x, y, size):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(math.radians(d)) for d in deg]
        yl = [y + size * math.sin(math.radians(d)) for d in deg]
        plt.plot(xl, yl, "-k")

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList,agent_size):

        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= (size+agent_size):
                return False  # collision
        return True  # safe
    
    def __BoxCollisionCheck(self,node,box_bounds_obstacleList,agent_size):
        
        for (xmin,xmax,ymin,ymax) in box_bounds_obstacleList:
            
            above_bottom = node.y > ymin
            below_top = node.y < ymax 
            right_of_left = node.x > xmin
            left_of_right = node.x < xmax
            if above_bottom and below_top and right_of_left and left_of_right:
                return False  # collision


        return True  # safe
    def __AgentCollisionCheck(self, node, safe_size,agent_size,past_paths=None):

        if past_paths != None:
            for i in range(0,len(past_paths)):
                last_pos = len(past_paths[i])
                if node.t < last_pos:
                    dx = past_paths[i][node.t][0] - node.x
                    dy = past_paths[i][node.t][1] - node.y
                    d = math.sqrt(dx * dx + dy * dy)
                    if d <= (safe_size[i]+agent_size):
                        return False  # collision
                else:
                    dx = past_paths[i][last_pos-1][0] - node.x
                    dy = past_paths[i][last_pos-1][1] - node.y
                    d = math.sqrt(dx * dx + dy * dy)
                    if d <= (safe_size[i]+agent_size):
                        return False  # collision
        return True  # safe

    def __BoundsCollisionCheck(self, node, bounds):

        if node.x < bounds[0] or node.x > bounds[1] or node.y < bounds[2] or node.y > bounds[3]:
            return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y, t=0):
        self.x = x
        self.y = y
        self.parent = None
        self.t = t


def GetPathLength(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d

    return le


def GetTargetPoint(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    partRatio = (le - targetL) / lastPairLen
    #  print(partRatio)
    #  print((ti,len(path),path[ti],path[ti+1]))

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio
    #  print((x,y))

    return [x, y, ti]


def LineCollisionCheck(first, second, obstacleList):
    # Line bounds = [0, 140, 0, 60]Equation

    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    try:
        a = y2 - y1
        b = -(x2 - x1)
        c = y2 * (x2 - x1) - x2 * (y2 - y1)
    except ZeroDivisionError:
        return False

    for (ox, oy, size) in obstacleList:
        d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
        if d <= (size):
            return False

    #  print("OK")

    return True  # OK

def obstacle_collision_check(point,obstacleList):
    for ox,oy,size in obstacleList:
        d = np.power((np.power(ox-point[0],2)+np.power(oy-point[1],2)),0.5)
        if d <= size+2:
            return False
    return True


def PathSmoothing(path, maxIter, obstacleList):
    #  print("PathSmoothing")

    le = GetPathLength(path)
    print('Initial Path: ',len(path))

    for i in range(maxIter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        #  print(pickPoints)
        first = GetTargetPoint(path, pickPoints[0])
        #  print(first)
        second = GetTargetPoint(path, pickPoints[1])
        #  print(second)

        if first[2] <= 0 or second[2] <= 0:
            continue

        if (second[2] + 1) > len(path):
            continue


        if second[2] == first[2]:
            continue

        # collision check
        if not LineCollisionCheck(first, second, obstacleList):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[2] + 1])
        newPath.append([first[0], first[1]])
        newPath.append([second[0], second[1]])
        newbounds = Path.extend(path[second[2] + 1:])
        path = newPath
        le = GetPathLength(path)
        #print('Final Path: ',len(path))

    return path


def RRT_Get_Path(regions_to_avoid=None,start=None,goal=None,past_paths=None,start_region=None,goal_region=None,agentnum=None,agent_radius=None,expandDis=1,bounds=[0,100,0,100],max_rrt_time=2,box_bounds_obs_regions=None):
    # ====Search Path with RRT====
    # Parameter

#Need to setup timed backcheck - if time at last node is too long, either restart whole thing or do RRT*

    t = time.time()

#Constant Obstacles
# [x,y,size]
    obstacleList = [[0,0,0],[0,0,0]]

    if regions_to_avoid is not None and len(regions_to_avoid):
        obstacleList = np.concatenate((obstacleList,regions_to_avoid))


    start = [(start[0],start[1])]


    goal = [(goal[0],goal[1])]

    #To run function, we give it a set of regions no to visit as obstacles,
    #the start point for the agent in question, and all prior trajectories we have

    #I think this might be slower than I hoped - there are a ton of obstacles.
    #past_paths = []
    #start=[(5, 5),(30,20),(80,50),(100,50),(120,50),(140,50),(80,10),(20,30),(25,55),(100,5)]
    #goal=[(125,30),(165,30),(145,35),(60,5),(5,5),(5,25),(5,55),(155,55),(120,5),(140,50)]
    num_agents = len(start)
    plot_bounds = bounds
    #if agent_radius == None:
    #    agent_radius = np.ones(num_agents)*3
    '''
    plt.plot(start[0][0], start[0][1], "ok")
    plt.plot(goal[0][0], goal[0][1], "xk")
    plt.axis(plot_bounds)
    plt.grid(True)
    for (x, y, size) in obstacleList:
        PlotCircle(x, y, size,plot_bounds)
    if past_paths != None:
        for i in range(0,len(past_paths)):
            for j in range(0,len(past_paths[i])):
                plt.plot(past_paths[i][j][0],past_paths[i][j][1],"sr")
    plt.pause(0.01)
    plt.gridbounds = [0, 140, 0, 60](True)
    plt.pause(0.01)  # Need for Mac
    plt.show(block=False)
    plt.pause(0.1)
    plt.waitforbuttonpress()

    plt.clf()
    '''
    #Start Planning with RRT

    i = 0
    rrt = RRT(start=start[i], goal=goal[i],
              randArea=bounds,expandDis=expandDis, obstacleList=obstacleList,past_paths=past_paths,agentnum=agentnum,escape_time=max_rrt_time,box_bounds_obstacleList=box_bounds_obs_regions)
    #rrt = RRT(start=[(0, 0),(10,0)], goal=[(5, 10),(0,10)],
    #          randArea=[-2, 15], obstacleList=obstacleList)
    if np.size(agent_radius)>1:
        path = rrt.Planning(safe_size=agent_radius,agent_size=agent_radius[agentnum],animation=False)
    else:
        path = rrt.Planning(safe_size=agent_radius,agent_size=agent_radius,animation=False)
    path = path[::-1]
    # Path smoothing
    #maxIter = 1000
    #smoothedPath = PathSmoothing(path, maxIter, obstacleList)

    '''
    plt.plot(start[0][0], start[0][1], "or")
    plt.plot(goal[0][0], goal[0][1], "xr")
    plt.axis(plot_bounds)
    plt.grid(True)
    for (x, y, size) in obstacleList:
        Plotbounds = [0, 140, 0, 60]Circle(x, y, size,plot_bounds)
    if past_paths != None:
        for i in range(0,len(past_paths)):
            for j in range(0,len(past_paths[i])):
                PlotCircle(past_paths[i][j][0],past_paths[i][j][1],3,plot_bounds)
    plt.pause(0.01)
    plt.grid(True)
    plt.pause(0.01)  # Need for Mac
    plt.show(block=False)
    plt.pause(0.1)
    plt.waitforbuttonpress()

    plt.clf()
    '''
    #print('All Found')
    '''
    print('Time:',(time.time()-t))
    # Draw final path
    if show_animation:
        #rrt.DrawGraph(plot_bounds)
        for i in range(0,num_agents):
            plt.plot([x for (x, y) in past_paths[i]], [y for (x, y) in past_paths[i]], '-r')
            #plt.plot([x for (x, y) in smoothedPath], [
            #    y for (x, y) in smoothedPath], '-b')
            plt.plot(start[i][0], start[i][1], "or")
            plt.plot(goal[i][0], goal[i][1], "xr")
        #plt.xlim(plot_bounds[0],plot_bounds[1])
        #plt.ylim(plot_bounds[2],plot_bounds[3])
        plt.axis(plot_bounds)
        plt.grid(True)
        for (x, y, size) in obstacleList:
            PlotCircle(x, y, size,plot_bounds)
        plt.pause(0.01)
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()


    #Play Trajectories as video
    max_time = 0
    for i in range(0,num_agents):
        if len(past_paths[i]) > max_time:
            max_time = len(past_paths[i])
    for t in range(0,max_time):
        DrawGraph(plot_bounds,past_paths,num_agents,start,goal,agent_radius,obstacleList,t)
        plt.pause(.01)
        time.sleep(.01)
        '''
    return(path)

def DrawGraph(bounds,past_paths,num_agents,start,goal,agent_radius,obstacleList,t):
    plt.clf()
    for j in range(0,num_agents):
        max_time = len(past_paths[j])
        if max_time > t:
            plt.plot(start[j][0], start[j][1], ".r")
            plt.plot(goal[j][0], goal[j][1], "xr")
            plt.plot(past_paths[j][t][0],past_paths[j][t][1],"sb",markersize=12)
        else:
            plt.plot(start[j][0], start[j][1], ".r")
            plt.plot(goal[j][0], goal[j][1], "xr")
            plt.plot(goal[j][0], goal[j][1],"sb",markersize=12)

    for (x, y, size) in obstacleList:
        PlotCircle(x, y, size,bounds)
    #plt.xlim(bounds[0],bounds[1])
    #plt.ylim(bounds[2],bounds[3])
    plt.axis(bounds)
    plt.grid(True)

def PlotCircle(x, y, size,bounds):
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(math.radians(d)) for d in deg]
    yl = [y + size * math.sin(math.radians(d)) for d in deg]
    #plt.xlim(bounds[0],bounds[1])
    #plt.ylim(bounds[2],bounds[3])
    plt.axis(bounds)
    plt.grid(True)
    plt.plot(xl, yl, "-k")

if __name__ == '__main__':
    main()
