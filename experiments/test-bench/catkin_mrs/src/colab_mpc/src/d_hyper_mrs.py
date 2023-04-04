#!/usr/bin/env python27

import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time
import math
from math import isclose

sys.path.append(sys.path[0]+'/DistributedControllerObject')
sys.path.append(sys.path[0]+'/Utilities')
sys.path.append(sys.path[0]+'/Utilities')

from PathFollowingLPVMPC_distri import PathFollowingLPV_MPC, _buildMatEqConst
from trackInitialization import Map, wrap
from plot_vehicle import *
from compute_plane import *

plot = True
it = True

def plot_hyperplanes(planes, points):
    plt.ion()

    for row in range(0,planes.shape[0]):
        ego = points[0,:,:,row]
        neighbour = points[1,:,:,row]

        # Define hyperplane equation
        a = planes[row,0]
        b = planes[row,1]
        c = -planes[row,2]

        # Generate random data points
        y = np.random.rand(50)

        # Calculate values of x and y based on hyperplane equation

        if isclose(b,0):
            y_hyperplane = (-b * y -c) / a
            plt.plot(y_hyperplane,y , color='red')
        else:
            y_hyperplane = (-a * y - c) / b
            plt.plot(y,y_hyperplane , color='red')
        # Plot data points and hyperplane
        # plt.scatter(x, y)

        plt.scatter(ego[:,0], ego[:,1], color='blue',marker="o")
        plt.scatter(neighbour[:,0], neighbour[:,1], color='green',marker="o")
        plt.show()
        plt.pause(0.001)


def generate_bounding_boxes(agents,dx,dy):
    print("placeholder")
    # (agent, points, x / y, horizon)
    # (horizon, agent, x / y, )

    agents_expanded = np.empty((2,4,2,10))

    for i in range(0,2):
        agents_expanded[i,0,0,:] = agents[:,i,0] + dx
        agents_expanded[i,0,1,:] = agents[:,i,1] + dy

        agents_expanded[i,1,0,:] = agents[:,i,0] - dx
        agents_expanded[i,1,1,:] = agents[:,i,1] + dy

        agents_expanded[i,2,0,:] = agents[:,i,0] + dx
        agents_expanded[i,2,1,:] = agents[:,i,1] - dy

        agents_expanded[i,3,0,:] = agents[:,i,0] - dx
        agents_expanded[i,3,1,:] = agents[:,i,1] - dy

    return agents_expanded

def initialise_agents(data,Hp,dt,map, accel_rate=0):
    agents = np.zeros((Hp,len(data),2))
    for id, el in enumerate(data):

        agents[:,id,:] = predicted_vectors_generation_V2(Hp, el, dt, map[id], accel_rate)[0][:,-2:]

    return agents

def predicted_vectors_generation_V2(Hp, x0, dt, map, accel_rate = 0):
    # We need a prediction of the states for the start-up proces of the controller (To instantiate the LPV variables)
    # [vx vy psidot y_e thetae theta s x y ]

    Vx      = np.zeros((Hp, 1))
    Vx[0]   = x0[0]
    S       = np.zeros((Hp, 1))
    S[0]    = 0
    Vy      = np.zeros((Hp, 1))
    Vy[0]   = x0[1]
    W       = np.zeros((Hp, 1))
    W[0]    = x0[2]
    Ey      = np.zeros((Hp, 1))
    Ey[0]   = x0[3]
    Epsi    = np.zeros((Hp, 1))
    Epsi[0] = x0[4]

    aux = map.getGlobalPosition(S[0], Ey[0])
    Theta = np.zeros((Hp, 1))
    Theta[0] = aux[2]
    X = np.zeros((Hp, 1))
    X[0] = aux[0]
    Y = np.zeros((Hp, 1))
    Y[0] = aux[1]

    Accel   = 1.0

    for i in range(0, Hp-1):
        Vy[i+1]      = x0[1]
        W[i+1]       = x0[2]
        Ey[i+1]      = x0[3]
        Epsi[i+1]    = x0[4]


    Accel   = Accel + np.array([ (accel_rate * i) for i in range(0, Hp)])

    for i in range(0, Hp-1):
        Vx[i+1]    = Vx[i] + Accel[i] * dt
        S[i+1]      = S[i] + Vx[i] * dt
        X[i+1], Y[i+1], Theta[i+1] = map.getGlobalPosition(S[i], Ey[i])

    xx  = np.hstack([ Vx, Vy, W,Ey, Epsi, Theta ,S ,X,Y]) # [vx vy psidot y_e thetae theta s x y]
    uu = np.zeros(( Hp, 2 ))
    return xx, uu

def main():

#########################################################
#########################################################
    # set constants


    N = 10
    dt = 0.01

    x0_0 = [1.3, -0.16, 0.00, 0.55, 0, 0.0, 0, 0.0, 1.55]  # [vx vy psidot y_e thetae theta s x y]
    x0_1 = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]
    x0_2 = [1.3, -0.16, 0.00, -0.55, 0, 0.0, 0, 0.0, 0.45]  # [vx vy psidot y_e thetae theta s x y]

    maps = [Map(),Map(),Map()]
    agents = initialise_agents([x0_0,x0_1,x0_2],N,dt,maps)

    agents_bb = generate_bounding_boxes(agents,0.2,0.2)

    planes = hyperplane_separator(2, 10)
    resu = planes.compute_hyperplane(agents)[:,:,0]
    plot_hyperplanes(resu,agents_bb)

if __name__ == "__main__":

    main()




