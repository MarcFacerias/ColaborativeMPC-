
# Global Variables
import sys
import numpy as np
import warnings

sys.path.append(sys.path[0]+'/DistributedPlanner')
sys.path.append(sys.path[0]+'/Utilities')
sys.path.append(sys.path[0]+'/plotter')
sys.path.append(sys.path[0]+'/Config/LPV')

from LPV_Planner_Hp import PlannerLPV
from trackInitialization import Map, wrap
from plot_tools import *
from utilities import checkEnd, initialise_agents
from io_module import io_class
from config import settings, initialiserLPV, x0_database

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


class agent(initialiserLPV):
    # Agents class, interfaces with the planner, saves data etc
    #  Q: [vx ; vy ; psiDot ; e_psi ; s ; e_y]
    #  R:  [delta ; a] la R es sobre el dU
    def __init__(self, settings, maps, x0, id):
        super().__init__(self, settings['path_csv'], settings['path_pck']) # initialise the initialiser
        self.map = maps
        self.N = settings["N"]
        self.dt = settings["dt"]
        self.Controller = PlannerLPV(self.Q, self.wq, self.Qs, self.R, self.dR, self.N, self.dt, self.map, id, self.model_param, self.sys_lim)
        self.x0 = x0
        self.states = []
        self.u = []
        self.planes = []
        self.time_op = []
        self.status = []
        self.id = id

    def one_step(self, agents, agents_id, pose, uPred = None, xPred = None):

        feas, raw, planes = self.Controller.solve(self.x0, xPred, uPred, agents, agents_id, pose)

        if not feas:
            return feas,uPred, xPred, planes, raw

        if (self.Controller.sPred[:,1] >= 0.1).any():
            msg = "WARNING slack violated !"
            warnings.warn(msg)
            print(self.Controller.sPred[:,1:])

        uPred, xPred = self.Controller.uPred, self.Controller.xPred
        self.save(xPred, uPred, planes, feas)

        return feas,uPred, xPred, planes, raw


def main():

    #########################################################
    #########################################################

    # Map settings

    n_agents = settings["n_agents"]
    map_type = settings["map_type"]
    N        = settings["N"]
    dt       = settings["dt"]
    max_it   = settings["max_it"]
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

    for i in range (0,n_agents):
        rs[i] = agent(settings, maps[i], x_old[i][0,:], i)

    io = io_class(settings, rs)

    while(it<max_it and not checkEnd(x_pred, maps)):

        io.tic()
        for i,r in enumerate(rs):
            feas[i], u_pred[i], x_pred[i], planes[i], raws[i] = r.one_step(agents[:, ns[i], :], ns[i], agents[:, i, :], u_old[i], x_old[i])

            if not feas[i]:
                error = True
                break

            r.x0 = x_pred[i][1, :]

        io.toc()

        if error:
            break

        u_old = u_pred
        for i in range(0,n_agents):
            x_old[i] = x_pred[i][1:, :]

        agents = np.swapaxes(np.asarray(x_pred)[:, :, -2:],0,1)

        io.update( x_pred, u_pred ,agents, it)
        it += 1

    io.update(x_pred, u_pred, agents, it, end=True)

if __name__ == "__main__":

    main()




