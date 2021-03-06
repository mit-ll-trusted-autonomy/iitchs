"""

Path tracking simulation with pure pursuit steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
Modified: Zachary Serlin

"""

#--------------------------------------------------------------------------------

#  The MIT License (MIT)

# Copyright (c) 2016 - 2021 Atsushi Sakai

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

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

#--------------------------------------------------------------------------------

import numpy as np
import math
import matplotlib.pyplot as plt
import pickle as pickle

'''
k = 0.5  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 2.3  # speed proportional gain
dt = 0.8  # [s]
L = 20.0  # [m] wheel base of vehicle


old_nearest_point_index = None
show_animation = False
'''

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((L / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((L / 2) * math.sin(self.yaw))


def update(state, a, delta):

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt
    state.rear_x = state.x - ((L / 2) * math.cos(state.yaw))
    state.rear_y = state.y - ((L / 2) * math.sin(state.yaw))

    return state


def PIDControl(target, current):
    a = Kp * (target - current)

    return a


def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index_time(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def calc_distance(state, point_x, point_y):

    dx = state.rear_x - point_x
    dy = state.rear_y - point_y
    return math.sqrt(dx ** 2 + dy ** 2)


def calc_target_index_time(state, cx, cy):
    
    
    global old_nearest_point_index
    ind = old_nearest_point_index + int(Lfc)
    old_nearest_point_index = ind
    
    '''
    distance_this_index = calc_distance(state, cx[ind], cy[ind])
    L = 0.0
    Lf = k * state.v + Lfc
    #distance_next_index = calc_distance(state, cx[ind+1], cy[ind+1])
    #if distance_this_index < distance_next_index:
    #    ind += 1
    #    old_nearest_point_index = ind
    if distance_this_index < Lf*2:
        while Lf > L and (ind + 1) < len(cx):
            dx = cx[ind] - state.rear_x
            dy = cy[ind] - state.rear_y
            L = math.sqrt(dx ** 2 + dy ** 2)
            ind += 1
    

    # search look ahead target point index
    
    '''
    '''
    global old_nearest_point_index

    if old_nearest_point_index is None:
        # search nearest point index
        dx = [state.rear_x - icx for icx in cx]
        dy = [state.rear_y - icy for icy in cy]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        old_nearest_point_index = ind
    else:
        ind = old_nearest_point_index
        distance_this_index = calc_distance(state, cx[ind], cy[ind])
        while True:
            ind = ind + 1 if (ind + 1) < len(cx) else ind
            distance_next_index = calc_distance(state, cx[ind], cy[ind])
            if distance_this_index < distance_next_index:
                break
            distance_this_index = distance_next_index
        old_nearest_point_index = ind

    L = 0.0

    Lf = k * state.v + Lfc

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind] - state.rear_x
        dy = cy[ind] - state.rear_y
        L = math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
    '''
    return ind


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)






def sim_dynamics(m,the_plan):

    global k
    global Lfc
    global Kp
    global dt
    global L
    global old_nearest_point_index
    global show_animation

    k = m.k  # look forward gain
    Lfc = m.Lfc  # look-ahead distance
    Kp = m.Kp  # speed proportional gain
    dt = m.dt  # [s]
    L = m.L  # [m] wheel base of vehicle

    old_nearest_point_index = m.old_nearest_point_index
    show_animation = False


    for a in range(0,the_plan.shape[0]):
        print(a)
        cx = []
        cy = []
        #print(the_plan[a])
        for t in range(0,the_plan.shape[1]):
            #if not the_plan[a][t][0] == []:
            cx = list(cx)
            cy = list(cy)
            cx.append(the_plan[a][t][0])
            cy.append(the_plan[a][t][1])

	    #
        test_cx = [i for i in cx if i>=0]
        cx = test_cx
        test_cy = [i for i in cy if i>=0]
        cy = test_cy
        target_speed = 1  # [m/s]
        
        traj_len = len(cx)
        print(traj_len)

        noise = np.random.normal(0,0.01,traj_len)
        cx = cx+noise
        cy = cy+noise

        T = traj_len#750.0  # max simulation time

        end_valx = 0
        end_valy = 0
        first_end = 0
        a_time = [T]
        a_time = np.asarray(a_time)
        #for g in range(0,T):
        #    if cx[g] < -1 and first_end == 0:
        #        end_valx = cx[g-1]
        #        end_valy = cy[g-1]
         #       first_end = 1
        #        a_time[0] = g
            
        #T = a_time
        # initial state
        state = State(x=cx[0], y=cy[0], yaw=2.0, v=0.0)
        #state = State(x=-0.0, y=-3.0, yaw=0.0, v=0.0)

        lastIndex = T - 1
        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        old_nearest_point_index = 0
        target_ind = calc_target_index_time(state, cx, cy)

        while T >= time and lastIndex > target_ind:
            #print(time,target_ind,lastIndex)
            ai = PIDControl(target_speed, state.v)

            di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
            state = update(state, ai, di)

            time = time + dt

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)

            if show_animation:  # pragma: no cover
                plt.cla()
                plot_arrow(state.x, state.y, state.yaw)
                plt.plot(cx, cy, "-r", label="course")
                plt.plot(x, y, "-b", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)

        # Test
        assert lastIndex >= target_ind, "Cannot goal"

        if True:  # pragma: no cover
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            #plt.subplots(1)
            #plt.plot(t, [iv * 3.6 for iv in v], "-r")
            #plt.xlabel("Time[s]")
            ##plt.ylabel("Speed[km/h]")
            #plt.grid(True)
            plt.show()
        np.savetxt(str(a)+'_des_x.txt',cx)
        np.savetxt(str(a)+'_des_y.txt',cy)
        np.savetxt(str(a)+'_atrit_time',a_time)
        np.savetxt(str(a)+'_traj_x.txt',x)
        np.savetxt(str(a)+'_traj_y.txt',y)
        


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
