#!/usr/bin/env python27

import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time
import math

sys.path.append(sys.path[0]+'/ControllerObject')
sys.path.append(sys.path[0]+'/Utilities')
sys.path.append(sys.path[0]+'/plotter')

from PathFollowingLPVMPC_NOROS import PathFollowingLPV_MPC
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
        self.R = 1 * np.diag([3, 0.8])  # [delta ; a]
        self.dR = 15 * np.array([1.0, 1.5])  #[delta ; a]
        self.Controller = PathFollowingLPV_MPC(self.Q, self.R, self.dR, N,0, dt, Map, "OSQP", 0, 0)
        self.x0 = x0

    def one_step(self, uPred = None, xPred = None):

        if (xPred is None):
            xPred, uPred = predicted_vectors_generation_V2(self.N, np.array(self.x0), self.dt)

        feas, uPred, xPred, planes = self._solve(self.x0, xPred, uPred)

        return feas,uPred, xPred, planes


    def _solve(self, x0, Xpred, uPred):

        feas, Solution, planes = self.Controller.solve(x0, Xpred, uPred, False, np.ones(10)*7.5, 0, "A_L", "B_L" ,"C_L", 1)

        return feas, self.Controller.uPred, self.Controller.xPred, planes



def predicted_vectors_generation_V2(Hp, x0, dt, accel_rate = 0):
    # We need a prediction of the states for the start-up proces of the controller (To instantiate the LPV variables)
    # [vx vy psidot y_e thetae theta s x y ]

    Vx      = np.zeros((Hp+1, 1))
    Vx[0]   = x0[0]
    S       = np.zeros((Hp+1, 1))
    S[0]    = 0
    Vy      = np.zeros((Hp+1, 1))
    Vy[0]   = x0[1]
    W       = np.zeros((Hp+1, 1))
    W[0]    = x0[2]
    Ey      = np.zeros((Hp+1, 1))
    Ey[0]   = x0[3]
    Epsi    = np.zeros((Hp+1, 1))
    Epsi[0] = x0[4]

    Accel   = 1.0
    curv    = 0

    for i in range(0, Hp):
        Vy[i+1]      = x0[1]
        W[i+1]       = x0[2]
        Ey[i+1]      = x0[3]
        Epsi[i+1]    = x0[4]

    Accel   = Accel + np.array([ (accel_rate * i) for i in range(0, Hp)])

    for i in range(0, Hp):
        Vx[i+1]    = Vx[i] + Accel[i] * dt
        S[i+1]      = S[i] + Vx[i] * dt

    xx  = np.hstack([ Vx, Vy, W, Epsi ,S ,Ey]) # [vx vy omega theta_e s y_e]
    uu = np.zeros(( Hp, 1 ))
    return xx, uu

def main():

#########################################################
#########################################################
    # set constants


    N = 10
    dt = 0.01

    x0_0 = [1.3, -0.16, 0.00, 0.0, 0, 0.0]  # [vx vy psidot y_e thetae theta s x y]

    maps = [Map()]

    if plot:
        disp = plotter(maps[0],1)

    r0 = agent(N, maps[0], dt, x0_0)

    x_old0 = None
    u_old0 = None

    while(it):

        # TODO acces the subset of lambdas of our problem

        tic = time.time()
        f0, uPred0, xPred0, planes0 = r0.one_step( u_old0, x_old0 )
        print(time.time() - tic)

        if f0:
            print(xPred0[0,:])
            r0.x0 = xPred0[1,:]
            x_old0 = xPred0
            u_old0 = uPred0

            if plot :
                print("Plot Time")
                tic = time.time()
                disp.plot_step(xPred0[1, 7], xPred0[1, 8], 0)
                print(time.time() - tic)
                print("End plot time")
        else:
            print("Drame, problem not solvable")
            quit()



if __name__ == "__main__":

    main()




