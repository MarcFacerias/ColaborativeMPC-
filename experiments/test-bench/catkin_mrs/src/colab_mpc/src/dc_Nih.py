
# Global Variables
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time
import os

sys.path.append(sys.path[0]+'/DistributedPlanner')
sys.path.append(sys.path[0]+'/Utilities')
sys.path.append(sys.path[0]+'/plotter')
sys.path.append(sys.path[0]+'/Config/LPV')

from LPV_Planner_HP import PlannerLPV
from trackInitialization import Map, wrap
from plot_vehicle import *
from utilities import checkEnd
from config import *  #Important!! Containts system definitions

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class agent(initialiserLPV):
    # Agents class, interfaces with the planner, saves data etc
    #  Q: [vx ; vy ; psiDot ; e_psi ; s ; e_y]
    #  R:  [delta ; a] la R es sobre el dU
    def __init__(self, N, Map, dt, x0, id):
        super().__init__("LPV") # initialise the initialiser
        self.map = Map
        self.N = N
        self.dt = dt
        self.Controller = PlannerLPV(self.Q,self.Qs, self.R, N, dt, Map, id, self.model_param, self.sys_lim)
        self.x0 = x0
        self.states = []
        self.u = []
        self.planes = []
        self.time_op = []
        self.status = []
        self.id = id

    def one_step(self, agents, agents_id, pose, uPred = None, xPred = None):

        if (xPred is None or uPred is None):
            xPred, uPred = predicted_vectors_generation_V2(self.N, np.array(self.x0), self.dt, self.map)

        tic = time.time()
        feas, raw, planes = self.Controller.solve(self.x0, xPred, uPred, agents, agents_id, pose)
        uPred, xPred = self.Controller.uPred, self.Controller.xPred
        self.time_op.append(time.time() - tic)
        self.status.append(feas)
        self.save(xPred, uPred, planes)

        return feas,uPred, xPred, planes, raw

    def plot_experiment(self):

        disp = plotter_offline(self.map)
        disp.add_agent_ti(self)
        disp.add_planes_ti(self)

    def save(self, xPred, uPred, planes):

        self.states.append(xPred[0,:])
        self.u.append(uPred[0,:])
        self.planes.append(planes[0,:])

    def save_to_csv(self):

        path = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/DistributedPlanner/TestsPaperL/" + str(self.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path +'/states.dat', self.states, fmt='%.5e',delimiter=' ')
        np.savetxt(path + '/u.dat', self.u, fmt='%.5e', delimiter=' ')
        np.savetxt(path + '/time.dat', self.time_op, fmt='%.5e', delimiter=' ')

    def save_var_to_csv(self,var, name):

        path = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/DistributedPlanner/TestsPaperL/" + str(self.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path + '/' + str(name) + '.dat', var, fmt='%.5e',delimiter=' ')

def initialise_agents(data,Hp,dt,map, accel_rate=0):
    agents = np.zeros((Hp+1,len(data),2))
    x_pred = [''] * len(data)
    for id, el in enumerate(data):

        x_pred[id] = predicted_vectors_generation_V2(Hp, el, dt, map[id], accel_rate)[0]
        agents[:,id,:] = x_pred[id][:,-2:]
    return agents,x_pred

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
    # set constants
    x0 = x0_database[0:n_agents]
    ns = [[i for i in range(0, n_agents)] for j in range(0, n_agents)]

    for j,n in enumerate(ns):
        n.remove(j)

    x_pred = [None] * n_agents
    u_pred = [None] * n_agents
    u_old  = [None] * n_agents
    feas   = [None] * n_agents
    raws   = [None] * n_agents
    planes = [None] * n_agents
    rs     = [None] * n_agents

    maps = [Map(map_type)]*n_agents

    agents,x_old = initialise_agents(x0,N,dt,maps)
    states_hist = [agents]

    if plot:
        disp = plotter(maps[0],n_agents)

    if plot_end:
        d = plotter_offline(maps[0])

    for i in range (0,n_agents):

        rs[i] = agent(N, maps[i], dt, x0[i], i)

    it = 0

    while(it<max_it and not checkEnd(x_pred, maps)):

        tic = time.time()
        for i,r in enumerate(rs):
            feas[i], u_pred[i], x_pred[i], planes[i], raws[i] = r.one_step(agents[:, ns[i], :], ns[i], agents[:, i, :], u_old[i], x_old[i])
            r.x0 = x_pred[i][1, :]
            x_old[i] = x_pred[i][1:, :]

        u_old = u_pred
        agents = np.swapaxes(np.asarray(x_pred)[:, :, -2:],0,1)
        states_hist.append(agents)
        toc = time.time()

        it += 1
        if plot :
            for idx in range(0,n_agents):
                disp.plot_step(x_pred[idx][1, 7], x_pred[idx][1, 8], x_pred[0][1, 5], idx)

        if verb:

            print("--------------------------------------------------------------")
            print("it: " + str(it))
            print("agents x : " + str(agents[0,:,0]))
            print("agents y : " + str(agents[0,:,1]))

            for i in range(0,n_agents):
                print("---------------------Agents---------------------------------------")

                print("Agent " + str(i) + " track s: " + str(x_pred[i][0,-3]) + "/" + str(maps[i].TrackLength[0]))
                print("Agent " + str(i) + " u0: " + str(u_pred[i][0,0]) + " u1: " + str(u_pred[i][0,1]))

            print("---------------------END Agents---------------------------------------")
            print("avg computational time: " + str((toc-tic)/n_agents))
            print("--------------------------------------------------------------")

    if plot_end:
        for j,r in enumerate(rs):
            d.plot_offline_experiment(r, color_list[j])
            r.save_to_csv()
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




