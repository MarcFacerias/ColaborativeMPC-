#!/usr/bin/env python27

import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time
import math

sys.path.append(sys.path[0]+'/DistributedPlanner')
sys.path.append(sys.path[0]+'/Utilities')
sys.path.append(sys.path[0]+'/Utilities')

from PathFollowingLPVMPC_distri import PathFollowingLPV_MPC, _buildMatEqConst
from trackInitialization import Map, wrap
from plot_vehicle import *

plot = True
it = True
class agent():

    #TODO: define Q and R
    def __init__(self, N, Map, dt, x0, Q=None,R=None):
        self.map = Map
        self.N = N
        self.dt = dt
        self.Q  = np.diag([120.0, 1.0, 1.0, 70.0, 0.0, 1500.0])   #[vx ; vy ; psiDot ; e_psi ; s ; e_y]
        self.R  = 0.01* np.diag([1, 1])                         #[delta ; a]
        self.Controller = PathFollowingLPV_MPC(self.Q, self.R, N, dt, Map, "OSQP")
        self.x0 = x0

    def one_step(self, lambdas, agents, uPred = None, xPred = None):

        if (xPred is None):
            xPred, uPred = predicted_vectors_generation_V2(self.N, np.array(self.x0), self.dt, self.map)

        feas, uPred, xPred = self._solve(self.x0, agents, lambdas,xPred, uPred)

        return feas,uPred, xPred


    def _solve(self, x0, agents,lambdas, Xpred, uPred):

        feas, Solution = self.Controller.solve(x0, Xpred, uPred, False, lambdas, agents)

        return feas, self.Controller.uPred, self.Controller.xPred


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

def eval_constraint(x1, x2, D):

    cost = -(math.sqrt((x1[0] - x2[0]) ** 2 + (x1[1] - x2[1]) ** 2) - D)

    if (cost<0):
        cost = 0

    return cost

def stop_criteria(cost, th):

    if np.all(cost <= th):
        return True

    return False

def main():

#########################################################
#########################################################
    # set constants


    N = 10
    dt = 0.01
    alpha = 5
    lambdas = np.zeros((3,3,N))
    max_it = 100
    finished = False
    lambdas_hist = [lambdas]

    # define neighbours
    n_0 = [0,2]
    n_1 = [1,2]
    n_2 = [0,1]

    x0_0 = [1.3, -0.16, 0.00, 0.55, 0, 0.0, 0, 0.0, 1.55]  # [vx vy psidot y_e thetae theta s x y]
    x0_1 = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]
    x0_2 = [1.3, -0.16, 0.00, -0.55, 0, 0.0, 0, 0.0, 0.45]  # [vx vy psidot y_e thetae theta s x y]

    maps = [Map(),Map(),Map()]
    agents = initialise_agents([x0_0,x0_1,x0_2],N,dt,maps)
    states_hist = [agents]

    if plot:
        disp = plotter(maps[0],3)

    r0 = agent(N, maps[0], dt, x0_0)
    r1 = agent(N, maps[1], dt, x0_1)
    r2 = agent(N, maps[2], dt, x0_2)

    x_old0 = None
    x_old1 = None
    x_old2 = None
    u_old0 = None
    u_old1 = None
    u_old2 = None

    while(it):

        lambdas = np.zeros((3, 3, N))
        it_OCD = 0
        while(not finished or it_OCD == max_it):

            it_OCD = + 1
            # TODO acces the subset of lambdas of our problem

            f0, uPred0, xPred0 = r0.one_step(lambdas[0,n_0,:], agents[:,n_0,:], u_old0, x_old0 )
            print("end 1")
            f1, uPred1, xPred1 = r1.one_step(lambdas[1,n_1,:], agents[:,n_1,:], u_old1, x_old1)
            print("end 2")
            f2, uPred2, xPred2 = r2.one_step(lambdas[2,n_2,:], agents[:,n_2,:], u_old2, x_old2)
            print("end 3")

            if not( f0 and f1 and f2):
                print(f0,f1,f2)
                print("one of the optimisation problems was not feasible exiting ...")
                break

            cost = np.zeros((3,3,N))

            agents[:,0,:] = xPred0[:,-2:]
            agents[:,1,:] = xPred1[:,-2:]
            agents[:,2,:] = xPred2[:,-2:]

            for k in range(0,N):
                for i in range(0,3):
                    for j in range(0, 3):

                        if (i != j):
                            cost[i,j,k]= eval_constraint(agents[k,i,:],agents[k,j,:],0.5)

            # update lambdas
            lambdas += alpha*cost
            lambdas[lambdas<0] = 0
            lambdas_hist.append(lambdas)
            states_hist.append(agents)
            finished = stop_criteria(cost,0.1)

            if not finished:
                print("breakpoint placeholder")


        r0.x0 = xPred0[1,:]
        r1.x0 = xPred1[1,:]
        r2.x0 = xPred2[1,:]
        x_old0 = xPred0
        x_old1 = xPred1
        x_old2 = xPred2
        u_old0 = uPred0
        u_old1 = uPred1
        u_old2 = uPred2
        finished = False

        # print("-------------------------------------------------")
        # print("Pose")
        # print("-------------------------------------------------")
        # print(xPred0)
        #
        # print("-------------------------------------------------")
        # print("Control")
        # print("-------------------------------------------------")
        # print(uPred0)

        if plot :
            disp.plot_step(xPred0[1, 7], xPred0[1, 8], xPred0[1, 5], 0)
            disp.plot_step(xPred1[1, 7], xPred1[1, 8], xPred1[1, 5], 1)
            disp.plot_step(xPred2[1, 7], xPred2[1, 8], xPred2[1, 5], 2)


if __name__ == "__main__":

    main()




