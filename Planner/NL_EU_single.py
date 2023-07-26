
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import os
import warnings

sys.path.append(sys.path[0]+'/NonLinDistribPlanner')
sys.path.append(sys.path[0]+'/Utilities')
sys.path.append(sys.path[0]+'/plotter')
sys.path.append(sys.path[0]+'/Config/NL_EU')

from NL_Planner_Eu_single import Planner_Eud
from trackInitialization import Map, wrap
from plot_tools import *
from utilities import checkEnd, initialise_agents
from config import *

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class agent(initialiserNL_EU):

    def __init__(self, N, Map, dt, x0, id, dth):
        super().__init__()
        self.map = Map
        self.dt = dt
        self.N = N
        self.x0 = x0
        self.Controller = Planner_Eud(self.Q,self.Qs, self.R, N, dt, Map, id, dth)
        self.states = []
        self.u = []
        self.time_op = []
        self.status = []
        self.data_opti = []
        self.data_collec = []
        self.id = id

    def one_step(self, lambdas, agents, agents_id, uPred = None, xPred = None):

        tic = time.time()
        feas, Solution, self.data_opti = self.Controller.solve(self.x0, xPred, uPred, lambdas, agents, agents_id, self.data_collec)
        self.time_op.append(time.time() - tic)
        self.status.append(feas)

        return feas, self.Controller.uPred, self.Controller.xPred, Solution

    def plot_experiment(self):

        disp = plotter_offline(self.map)
        disp.add_agent_ti(self)

    def save(self, xPred, uPred):

        self.states.append(xPred[0,:])
        self.u.append(uPred[0,:])

    def save_to_csv(self):

        path = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/NonLinDistribPlanner/TestsPaperNLcs/" + str(self.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path+'/states.dat', self.states, fmt='%.5e',delimiter=' ')
        np.savetxt(path + '/u.dat', self.u, fmt='%.5e', delimiter=' ')
        np.savetxt(path + '/time.dat', self.time_op, fmt='%.5e', delimiter=' ')

    def save_var_to_csv(self,var, name):

        path = path_csv + str(self.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path + '/' + str(name) + '.dat', var, fmt='%.5e',delimiter=' ')

def eval_constraint(x1, x2, D):

    cost1 = D - np.sqrt(sum((x1-x2)**2)) # the OCD update depends on on the diference between the minimum D and the euclidean dist

    return np.array(cost1)

def main():

#########################################################
#########################################################

    # set constants
    x0 = [x0_database[0]]

    # initialise data structures
    maps = [Map(map_type)]
    agents,x_old,u_old = initialise_agents(x0,N,dt,maps)
    rs = agent(N, maps, dt, x_old, 0, dth)

    x_pred = [None]
    raws   = [None]

    if plot:
        disp = plotter(maps[0],n_agents)

    if plot_end:
        d = plotter_offline(maps[0])

    it = 0

    while(it<max_it and not checkEndS(x_pred, maps)):

        feas, u_pred, x_pred, raws = rs.one_step( 0, 0, 0, u_old, raws)

        rs.save(x_pred, u_pred)
        rs.x0 = [x_pred[1:, :]]

        u_old = [u_pred]
        toc = time.time()
        it += 1


        if verb :

            print("--------------------------------------------------------------")
            print(x_pred)
            print("--------------------------------------------------------------")

    if plot_end:

        d.plot_offline_experiment(rs, color_list[0],path_csv)
        rs.save_to_csv()

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

def checkEndS(x, maps):

    status = False
    agent = x[0]
    i = 0
    if agent is not None:
        if np.isclose(agent[-3],maps[i].TrackLength[maps[i].lane],atol=0.15) or (agent[-3] > maps[i].TrackLength[maps[i].lane]) :
            status = True
            return status

        else:
            return False

    return status

if __name__ == "__main__":

    main()




