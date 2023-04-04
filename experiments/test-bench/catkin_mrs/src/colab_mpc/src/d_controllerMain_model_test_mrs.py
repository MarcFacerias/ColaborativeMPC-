#!/usr/bin/env python27

import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time
import math

sys.path.append(sys.path[0]+'/DistributedControllerObject')
sys.path.append(sys.path[0]+'/DistributedControllerObject')
sys.path.append(sys.path[0]+'/Utilities')

from PathFollowingLPVMPC_distri import PathFollowingLPV_MPC, _buildMatEqConst, OneStepSim
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

        feas, Solution = self.Controller.solve(x0, Xpred, uPred, False, "A_L", "B_L", "C_L", 4, lambdas, agents)

        return feas, self.Controller.uPred, self.Controller.xPred




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

    x0_0 = [1.3, -0.16, 0.00, 0, 0, 0.0, 0, 0.0, 0.9]  # [vx vy psidot y_e thetae theta s x y]

    map = Map()
    states =  predicted_vectors_generation_V2(10, x0_0, 1/30, map)[0][0,:]
    agent_obj = agent(10, map, 1/30, x0_0)
    disp = plotter(map,2)


    controls = [i.split() for i in open("/home/marc/Escritorio/results_simu_test/test_bank/Trajectory_generation_ux/agent1/control.dat").readlines()]
    u_hist = np.empty((len(controls),2))

    controls = [i.split() for i in open("/home/marc/Escritorio/results_simu_test/test_bank/Trajectory_generation_ux/agent1/global_pose.dat").readlines()]
    pose_hist = np.empty((len(controls), 3))

    for k,control in enumerate(controls):
        u_hist[k,:] = float(control[0]), float(control[1])

    for k,pose in enumerate(controls):
        pose_hist[k,:] = float(pose[0]), float(pose[1]),float(pose[2])

    for t,u in enumerate(u_hist):
        states = OneStepSim(agent_obj.Controller, states, u)
        print(u)
        pose = np.array(map.getGlobalPosition(states[6],states[3])).flatten()
        disp.plot_step(pose[0],pose[1], pose[2])
        # disp.plot_step(pose_hist[t,0], pose_hist[t,1], pose_hist[t,2 ])



if __name__ == "__main__":

    main()




