
# CODE to test the corridor version of the map where 4 agents colaborate using asumed
# hyperplanes to constraint the free space of the solution

import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time
import os

sys.path.append(sys.path[0]+'/DistributedPlanner') # folder containing the controller
sys.path.append(sys.path[0]+'/Utilities')
sys.path.append(sys.path[0]+'/plotter')

from PathFollowingLPVMPC_independent_hyperplanes import PathFollowingLPV_MPC
from trackInitialization import Map, wrap # additional funtions
from plot_vehicle import *  #plooting functions

# hyperparameters
plot = True
plot_end = False
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class agent():
# Agent class that parses to the controller

    def __init__(self, N, Map, dt, x0, id, Q=np.diag([120.0, 1.0, 1.0, 1500.0, 70.0, 0.0, 0.0, 0, 0, 0]), R=1000 * np.diag([1, 1])):
        self.map = Map
        self.N = N
        self.dt = dt
        self.Q  = Q   #[vx ; vy ; psiDot ; e_psi ; s ; e_y]
        self.R  = R                         #[delta ; a]
        self.Controller = PathFollowingLPV_MPC(self.Q, self.R, N, dt, Map, "OSQP", id)
        self.x0 = x0
        self.states = []
        self.u = []
        self.planes = []
        self.output_opti = []
        self.time_op = []
        self.status = []
        self.id = id

    #TODO clear this redundant functions calls
    def one_step(self, agents, agents_id, pose, uPred = None, xPred = None):

        if (xPred is None):
            xPred, uPred = predicted_vectors_generation_V2(self.N, np.array(self.x0), self.dt, self.map)

        feas, uPred, xPred, planes, raw = self._solve(self.x0, agents, agents_id, pose, xPred, uPred)

        return feas,uPred, xPred, planes, raw

    def _solve(self, x0, agents, agents_id, pose, Xpred, uPred):

        tic = time.time()
        feas, Solution, planes = self.Controller.solve(x0, Xpred, uPred, agents, agents_id, pose)
        self.time_op.append(time.time() - tic)
        self.status.append(feas)
        return feas, self.Controller.uPred, self.Controller.xPred, planes, Solution

    # Plotting function used to add all the data into the plotting library
    def plot_experiment(self):

        disp = plotter_offline(self.map)
        disp.add_agent_ti(self)
        disp.add_planes_ti(self)

    # keep current values as values for time k
    def save(self, xPred, uPred, planes):

        self.states.append(xPred[0,:])
        self.u.append(uPred[0,:])
        self.planes.append(planes[0,:])

    def save_to_csv(self):
        # save agent data in .dat (suitable for matlab plotting) in a hard coded path within the experimetn folder
        #TODO: automate this to have consequtive experiments not overwrite each other
        path = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/DistributedPlanner/corr/" + str(self.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path +'/states.dat', self.states, fmt='%.5e',delimiter=' ')
        np.savetxt(path + '/u.dat', self.u, fmt='%.5e', delimiter=' ')
        np.savetxt(path + '/time.dat', self.time_op, fmt='%.5e', delimiter=' ')

    def save_var_to_csv(self,var, name):
        # save any data in .dat (suitable for matlab plotting) in a hard coded path within the experimetn folder
        path = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/DistributedPlanner/corr/" + str(self.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path + '/' + str(name) + '.dat', var, fmt='%.5e',delimiter=' ')

def initialise_agents(data,Hp,dt,map, accel_rate=0):
    # initialisation function for u0s and x0s
    agents = np.zeros((Hp+1,len(data),2))
    for id, el in enumerate(data):

        agents[:,id,:] = predicted_vectors_generation_V2(Hp, el, dt, map[id], accel_rate)[0][:,-2:]

    return agents

def predicted_vectors_generation_V2(Hp, x0, dt, map, accel_rate = 0):
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

    aux = map.getGlobalPosition(S[0], Ey[0])
    Theta = np.zeros((Hp+1, 1))
    Theta[0] = aux[2]
    X = np.zeros((Hp+1, 1))
    X[0] = aux[0]
    Y = np.zeros((Hp+1, 1))
    Y[0] = aux[1]

    Accel   = 1.0

    for i in range(0, Hp):
        Vy[i+1]      = x0[1]
        W[i+1]       = x0[2]
        Ey[i+1]      = x0[3]
        Epsi[i+1]    = x0[4]


    Accel   = Accel + np.array([ (accel_rate * i) for i in range(0, Hp)])

    for i in range(0, Hp):
        Vx[i+1]    = Vx[i] + Accel[i] * dt
        S[i+1]      = S[i] + Vx[i] * dt
        X[i+1], Y[i+1], Theta[i+1] = map.getGlobalPosition(S[i], Ey[i])

    xx  = np.hstack([ Vx, Vy, W,Ey, Epsi, Theta ,S ,X,Y]) # [vx vy psidot y_e thetae theta s x y]
    uu = np.zeros(( Hp, 2 ))
    return xx, uu


def main():

#########################################################
#########################################################
    # controller constants

    N = 25
    dt = 0.01

    # define coms
    n_0 = [1,2,3]
    n_1 = [0,2,3]
    n_2 = [0,1,3]
    n_3 = [0,1,2]

    # define initial positions
    x0_0 = [1.3, -0.16, 0.00, 0.45, 0, 0.0, 0, 0.0, 1.45]  # [vx vy psidot y_e thetae theta s x y]
    x0_1 = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]
    x0_2 = [1.3, -0.16, 0.00, 0.25, 0, 0.0, 0.25, 0.0, 1.5]  # [vx vy psidot y_e thetae theta s x y]
    x0_3 = [1.3, -0.16, 0.00, -0.25, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]

    # initialise map object
    maps = [Map("oval"),Map("oval"),Map("oval"),Map("oval")]
    # maps = [Map(), Map(), Map(), Map()]

    # get initial values for the agents
    agents = initialise_agents([x0_0,x0_1,x0_2,x0_3],N,dt,maps)

    #set up plotting
    if plot:
        disp = plotter(maps[0],4)

    if plot_end:
        d = plotter_offline(maps[0])

    # initialise controller objects
    r0 = agent(N, maps[0], dt, x0_0, 0)
    r1 = agent(N, maps[1], dt, x0_1, 1)
    r2 = agent(N, maps[2], dt, x0_2, 2)
    r3 = agent(N, maps[3], dt, x0_3, 3)

    # initialise update placeholders
    x_old0 = None
    x_old1 = None
    x_old2 = None
    x_old3 = None

    u_old0 = None
    u_old1 = None
    u_old2 = None
    u_old3 = None
    it = 0

    time_alg = []

    while(it<550):

        tic = time.time()

        # run an optimisation
        f0, uPred0, xPred0, planes0, raw0 = r0.one_step(agents[:,n_0,:], n_0, agents[:,0,:], u_old0, x_old0)
        f1, uPred1, xPred1, planes1, raw1 = r1.one_step(agents[:,n_1,:], n_1, agents[:,1,:], u_old1, x_old1)
        f2, uPred2, xPred2, planes2, raw2 = r2.one_step(agents[:,n_2,:], n_2, agents[:,2,:], u_old2, x_old2)
        f3, uPred3, xPred3, planes3, raw3 = r3.one_step(agents[:,n_3,:], n_3, agents[:,3,:], u_old3, x_old3)

        # break if any of the problems could not be solved
        if not (f0 and f1 and f2 and f3):
            break

        ## print if needed
        # print(xPred0[0,:])
        # print(xPred1[0,:])
        # print(xPred2[0,:])
        # print(xPred3[0,:])

        # update agents (x,y positions)
        agents[:, 0, :] = xPred0[:, -2:]
        agents[:, 1, :] = xPred1[:, -2:]
        agents[:, 2, :] = xPred2[:, -2:]
        agents[:, 3, :] = xPred3[:, -2:]

        # save current values for the next iteration
        r0.save(xPred0, uPred0, planes0)
        r1.save(xPred1, uPred1, planes1)
        r2.save(xPred2, uPred2, planes2)
        r3.save(xPred3, uPred3, planes3)

        r0.x0 = xPred0[1,:]
        r1.x0 = xPred1[1,:]
        r2.x0 = xPred2[1,:]
        r3.x0 = xPred3[1,:]

        x_old0 = xPred0[1:,:]
        x_old1 = xPred1[1:,:]
        x_old2 = xPred2[1:,:]
        x_old3 = xPred3[1:,:]

        u_old0 = uPred0
        u_old1 = uPred1
        u_old2 = uPred2
        u_old3 = uPred3
        time_alg.append((time.time() - tic) / 2)

        # print(time.time() - tic)
        it += 1

        if plot :
            disp.plot_step(xPred0[1, 7], xPred0[1, 8], xPred0[1, 5], 0)
            disp.plot_step(xPred1[1, 7], xPred1[1, 8], xPred1[1, 5], 1)
            disp.plot_step(xPred2[1, 7], xPred2[1, 8], xPred2[1, 5], 2)
            disp.plot_step(xPred3[1, 7], xPred3[1, 8], xPred3[1, 5], 3)


    if plot_end:
        d.plot_offline_experiment(r0, "oc", "-y")
        d.plot_offline_experiment(r1, "ob", "-y")
        d.plot_offline_experiment(r2, "or", "-y")
        d.plot_offline_experiment(r3, "oy", "-y")
        r0.save_var_to_csv(time_alg, "time_alg")
        r0.save_to_csv()
        r1.save_to_csv()
        r2.save_to_csv()
        r3.save_to_csv()
        input("Press enter to continue...")

def plot_performance( agent):

    fig_status = plt.figure()
    fig_status.add_subplot(2, 1, 1)
    x = np.arange(0,len(agent.status))
    plt.scatter(x, np.array(agent.status))
    fig_status.add_subplot(2, 1, 2)
    plt.scatter(x, np.array(agent.time_op))
    plt.show()
    plt.pause(0.001)

def plot_distance( distance_hist, th):

    fig_status = plt.figure(3)
    x = np.arange(0,len(distance_hist))
    plt.scatter(x, np.array(distance_hist))
    plt.plot(x, th*np.ones(len(distance_hist)), "-r")
    plt.show()
    plt.pause(0.001)

if __name__ == "__main__":

    main()




