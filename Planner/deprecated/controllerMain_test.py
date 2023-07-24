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

from PathFollowingLPVMPC_simple import PathFollowingLPV_MPC, _buildMatEqConst
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
        self.R  = np.diag([0.01, 0.0])                         #[delta ; a]
        self.Controller = PathFollowingLPV_MPC(self.Q, self.R, N, dt, Map, "OSQP")
        self.x0 = x0

    def one_step(self, uPred = None, xPred = None):

        if (xPred is None):
            xPred, uPred = predicted_vectors_generation_V2(self.N, np.array(self.x0), self.dt, self.map)

        feas, uPred, xPred = self._solve(self.x0, xPred, uPred)

        return feas,uPred, xPred


    def _solve(self, x0, Xpred, uPred):

        feas, Solution = self.Controller.solve(x0, Xpred, uPred, False, "A_L", "B_L", "C_L", 4)

        return feas, self.Controller.uPred, self.Controller.xPred


def initialise_agent(data,Hp,dt,map, accel_rate=0):

    return predicted_vectors_generation_V2(Hp, data, dt, map, accel_rate)[0][:,-2:]

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

    x0 = [0.97, 0.0, 0.00, 0, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]

    if plot:
        disp = plotter(Map(),3)

    r0 = agent(N, Map(), dt, x0)

    x_old0 = None
    u_old0 = None

    while(it):

        tic = time.time()
        f0, uPred0, xPred0 = r0.one_step(u_old0, x_old0 )
        print("Control actions")
        print("-------------------------------------------------")
        print(uPred0)
        print("States")
        print("-------------------------------------------------")
        print(xPred0)
        print(time.time() - tic)
        r0.x0 = xPred0[1,:]
        x_old0 = xPred0
        u_old0 = uPred0

        if plot :
            disp.plot_step(xPred0[1, 7],xPred0[1, 8], xPred0[1, 5])
            disp.plot_step(xPred1[1, 7], xPred1[1, 8], xPred1[1, 5])
            disp.plot_step(xPred2[1, 7], xPred2[1, 8], xPred2[1, 5])

if __name__ == "__main__":

    main()




