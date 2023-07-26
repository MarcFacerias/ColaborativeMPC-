
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

from NL_Planner_Eu import Planner_Eud
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
        self.Controller = Planner_Eud(self.Q,self.Qs, self.R, N, dt, Map, id, dth, self.model_param, self.sys_lim)
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

    # controller constants
    finished = False
    finished_ph = False
    time_OCD = []

    # set constants
    x0 = x0_database[0:n_agents]
    ns = [[i for i in range(0, n_agents)] for j in range(0, n_agents)]

    for j, n in enumerate(ns):
        n.remove(j)

    # initialise data structures
    maps = [Map(map_type)]*n_agents
    agents,x_old,u_old = initialise_agents(x0,N,dt,maps)

    x_pred = [None] * n_agents
    u_pred = [None] * n_agents
    feas   = [None] * n_agents
    raws   = [None] * n_agents
    rs     = [None] * n_agents
    data   = [None] * n_agents

    if plot:
        disp = plotter(maps[0],n_agents)

    if plot_end:
        d = plotter_offline(maps[0])

    for i in range(0, n_agents):
        data[i] = [x_old[i].flatten(), u_old[i].flatten(), np.zeros((N, 4)), np.zeros((N, n_agents)),
                    np.zeros((N, n_agents))]

    # initialise controllers and data holders
    for i in range (0,n_agents):
        rs[i] = agent(N, maps[i], dt, x_old[i], i, dth)
        rs[i].data_collec = [data[j] for j in ns[i]]

    cost_old = np.zeros((n_agents, n_agents, N))
    lambdas_hist = []
    cost_hist = []
    it = 0

    while(it<max_it and not checkEnd(x_pred, maps)):

        tic = time.time()
        lambdas = np.zeros((n_agents, n_agents, N))
        it_OCD = 0
        itc = 0

        while(not (it_OCD > 2 and finished)) :
            # OCD loop, we want to force at least 2 iterations + it_conv iterations without significant changes
            # run an instance of the optimisation problems

            for i, r in enumerate(rs):
                feas[i], u_pred[i], x_pred[i], raws[i] = r.one_step( lambdas[[i],ns[i],:], agents[:,ns[i],:], ns[i], u_old[i], raws[i])

            for j,r in enumerate(rs):
                r.data_collec = [rs[i].data_opti for i in ns[j]]

            cost = np.zeros((n_agents,n_agents,N))

            # update the values of x,y for the obstacle avoidance constraints
            agents = np.swapaxes(np.asarray(x_pred)[:, :, -2:], 0, 1)

            for k in range(1,N+1):
                for i in range(0,n_agents):
                    for j in range(0,n_agents):

                        if (i != j) and i<j:
                            cost[i,j,k-1] = eval_constraint(agents[k,i,:],agents[k,j,:],dth)

            alpha = get_alpha()
            lambdas += alpha*cost
            lambdas_hist.append(lambdas)

            # check if the values of x changed, if they are close enough for two iterations the algorithm has converged
            if it_OCD != 0:
                finished_ph = 1
                for i in range(0,n_agents):
                    finished_ph &= np.allclose(x_old[i], x_pred[i], atol=0.01)

                finished_ph &= np.allclose(cost, cost_old, atol=0.01)
                itc += 1

            # store values from current iteration into the following one
            x_old = x_pred
            cost_old = cost

            if not finished_ph :
                itc = 0

            elif itc > it_conv:
                finished = True
                print("Iteration finished with " + str(it_OCD) + " steps")

            if it_OCD > max_it_OCD:
                print("max it reached")
                finished = True

            it_OCD += 1

            if verb_OCD:

                print("-------------------------------------------------")
                print("length " + str(it_OCD))
                for i in range(0, n_agents):
                    print("---------------------Agents---------------------------------------")
                    print("Agent " + str(i) + " track s: " + x_pred[i][1,:])
                print("-------------------------------------------------")

        for j,r in enumerate(rs):
            r.save(x_pred[j], u_pred[j])
            r.x0 = x_pred[j][1:, :]

        u_old = u_pred
        toc = time.time()
        finished = False
        time_OCD.append((toc - tic)/n_agents)
        it += 1

        if plot :

            for idx in range(0, n_agents):
                disp.plot_step(x_pred[idx][1, 7], x_pred[idx][1, 8], x_pred[0][1, 5], idx)

        if verb :

            print("--------------------------------------------------------------")
            print("it: " + str(it))
            print("length " + str(it_OCD))
            print(time.time() - tic)
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

        r[0].save_var_to_csv(time_OCD, "time_OCD")
        r[0].save_var_to_csv(cost_hist, "cost_hist2")
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

if __name__ == "__main__":

    main()




