
# Global Variables
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time
import os

# sys.path.append(sys.path[0]+'/DistributedControllerObjectLegacy')
sys.path.append(sys.path[0]+'/DistributedPlanner')

sys.path.append(sys.path[0]+'/Utilities')
sys.path.append(sys.path[0]+'/plotter')
sys.path.append(sys.path[0]+'/Config/LPV')

# from PathFollowingLPVMPC_independent_hyperplanes import PathFollowingLPV_MPC
from LPV_Planner_Hp import PlannerLPV
from trackInitialization import Map, wrap
from plot_vehicle import *
from utilities import checkEnd, initialise_agents
from config import *  #Important!! Containts system definitions

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


class agent(initialiserLPV):
    # Agents class, interfaces with the planner, saves data etc
    #  Q: [vx ; vy ; psiDot ; e_psi ; s ; e_y]
    #  R:  [delta ; a] la R es sobre el dU
    def __init__(self, N, Map, dt, x0, id):
        super().__init__() # initialise the initialiser
        self.map = Map
        self.N = N
        self.dt = dt
        # self.Controller = PathFollowingLPV_MPC(self.Q, self.R, N, dt, Map, "OSQP", id)
        self.Controller = PlannerLPV(self.Q, self.wq, self.Qs, self.R, N, dt, Map, id, self.model_param, self.sys_lim)
        self.x0 = x0
        self.states = []
        self.u = []
        self.planes = []
        self.time_op = []
        self.status = []
        self.id = id

    def one_step(self, agents, agents_id, pose, uPred = None, xPred = None):

        tic = time.time()
        feas, raw, planes = self.Controller.solve(self.x0, xPred, uPred, agents, agents_id, pose)

        if (self.Controller.sPred[:,1] >= 0.1).any():
            print("WARNING slack violated !")

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

        path = path_csv + str(self.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path +'/states.dat', self.states, fmt='%.5e',delimiter=' ')
        np.savetxt(path + '/u.dat', self.u, fmt='%.5e', delimiter=' ')
        np.savetxt(path + '/time.dat', self.time_op, fmt='%.5e', delimiter=' ')

    def save_var_to_csv(self,var, name):

        path = path_csv + str(self.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path + '/' + str(name) + '.dat', var, fmt='%.5e',delimiter=' ')

def main():

#########################################################
#########################################################
    # set constants

    x_pred = [None] * n_agents
    u_pred = [None] * n_agents
    feas   = [None] * n_agents
    raws   = [None] * n_agents
    planes = [None] * n_agents
    rs     = [None] * n_agents

    it = 0
    error = False

    x0 = x0_database[0:n_agents]
    ns = [[i for i in range(0, n_agents)] for j in range(0, n_agents)]

    for j, n in enumerate(ns):
        n.remove(j)

    maps = [Map(map_type)]*n_agents

    agents,x_old,u_old = initialise_agents(x0,N,dt,maps)
    states_hist = [agents]

    if plot:
        disp = plotter(maps[0],n_agents)

    for i in range (0,n_agents):

        rs[i] = agent(N, maps[i], dt, x_old[i][0,:], i)


    while(it<max_it and not checkEnd(x_pred, maps)):

        tic = time.time()
        for i,r in enumerate(rs):
            feas[i], u_pred[i], x_pred[i], planes[i], raws[i] = r.one_step(agents[:, ns[i], :], ns[i], agents[:, i, :], u_old[i], x_old[i])
            r.x0 = x_pred[i][1, :]

        u_old = u_pred
        for i in range(0,n_agents):
            x_old[i] = x_pred[i][1:, :]


        if not np.all(np.asarray(feas)):
            error = True
            break

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
            print("agents x : " + str(agents[1,:,0]))
            print("agents y : " + str(agents[1,:,1]))

            for i in range(0,n_agents):
                print("---------------------Agents---------------------------------------")

                print("Agent " + str(i) + " track s: " + str(x_pred[i][1,-3]) + "/" + str(maps[i].TrackLength[0]))
                print("Agent " + str(i) + " u0: " + str(u_pred[i][1,0]) + " u1: " + str(u_pred[i][1,1]))
                print("Agent " + str(i) + " v: " + str(x_pred[i][1,0]) + " ey: " + str(x_pred[i][1,3]))

            print("---------------------END Agents---------------------------------------")
            print("avg computational time: " + str((toc-tic)/n_agents))
            print("--------------------------------------------------------------")

    if plot_end or error:
        d = plotter_offline(maps[0])
        for j,r in enumerate(rs):
            d.plot_offline_experiment(r, color_list[j], path = path_img)
            r.save_to_csv()

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




