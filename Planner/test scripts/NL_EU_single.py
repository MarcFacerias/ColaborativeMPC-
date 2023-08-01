
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import os
import warnings
import pickle

sys.path.append(sys.path[0]+'/nonLinDistribPlanner')
sys.path.append(sys.path[0]+'/utilities')
sys.path.append(sys.path[0]+'/plotter')
sys.path.append(sys.path[0]+'/config/NL')

from NL_Planner_Eu_singleNM import Planner_Eud
# from NL_Planner_Eu_single import Planner_Eud
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
        self.uPred_hist = []
        self.sPred_hist = []
        self.Controller = Planner_Eud(self.Q,self.Qs, self.R, self.dR, N, dt, Map, id, dth)
        self.states = []
        self.u = []
        self.time_op = []
        self.status = []
        self.data_opti = []
        self.data_collec = []
        self.id = id

    def one_step(self, lambdas, agents, agents_id, uPred = None, xPred = None):

        tic = time.time()
        feas, xPred, self.data_opti = self.Controller.solve(self.x0, xPred, uPred, lambdas, agents, agents_id, self.data_collec)
        self.time_op.append(time.time() - tic)
        self.status.append(feas)
        self.uPred_hist.append(self.Controller.uPred)
        self.sPred_hist.append(xPred)

        return feas, self.Controller.uPred, xPred

    def save(self, xPred, uPred):

        self.states.append(xPred[0,:])
        self.u.append(uPred[0,:])

    def save_to_csv(self):

        path = path_csv + str(self.id)

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

    def save_var_pickle(self, vars = None, tags = None):
        path = path_csv + str(self.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        if vars is None:
                with open(path + '/u.pkl', 'wb') as f1:  # Python 3: open(..., 'wb')
                    pickle.dump(self.uPred_hist, f1)
                with open(path + '/states.pkl', 'wb') as f2:  # Python 3: open(..., 'wb')
                    pickle.dump(self.sPred_hist, f2)
        else:

            for i, var in enumerate(vars):

                try:
                    with open(path + '/' + tags[i] + '.pkl', 'wb') as f1:  # Python 3: open(..., 'wb')
                        pickle.dump(var, f1)

                except:
                    with open(path +'/def' + str(i) + '.pkl', 'wb') as f2:  # Python 3: open(..., 'wb')
                        pickle.dump(var, f2)
                        msg = "WARNING no name asigned !"
                        warnings.warn(msg)


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
    raws   = x_old[0]

    if plot:
        disp = plotter(maps[0],n_agents)

    if plot_end:
        d = plotter_offline(maps[0])

    it = 0

    while(it<max_it and not checkEndS(x_pred, maps)):

        feas, u_pred, x_pred = rs.one_step( 0, 0, 0, u_old, x_pred)

        if not feas:
            break

        rs.save(x_pred, u_pred)
        rs.x0 = [x_pred[1:, :]]

        u_old = [u_pred]
        toc = time.time()
        it += 1


        if verb :

            print("--------------------------------------------------------------")
            print(x_pred)
            print(u_pred)
            print("--------------------------------------------------------------")

    if plot_end:

        d.plot_offline_experiment(rs, color_list[0],path_csv)
        rs.save_to_csv()
        rs.save_var_pickle()

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




