import time

# Global Variables
import numpy as np
import warnings

from Planner.planners.distributedPlanner import PlannerLPV
from Planner.packages.mapManager import Map
from Planner.packages.utilities import checkEnd, initialise_agents
from Planner.packages.IOmodule import io_class  # TODO change this for ros type and add plotting
from Planner.packages.config.LPV import initialiserLPV, settings
from Planner.packages.config import x0_database

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


class agentROS_LPV(initialiserLPV):
    # Agents class, interfaces with the planner, saves data etc
    #  Q: [vx ; vy ; psiDot ; e_psi ; s ; e_y]
    #  R:  [delta ; a] la R es sobre el dU
    def __init__(self, settings, x0, id, connections):
        super().__init__(self, settings) # initialise the initialiser
        self.map = Map(settings["map_type"])
        self.N = settings["N"]
        self.dt = settings["dt"]
        self.Controller = PlannerLPV(self.Q, self.Qs, self.R, self.dR, self.N, self.dt, self.map, id, self.wq, self.model_param, self.sys_lim)
        self.x0 = x0
        self.states = []
        self.u = []
        self.planes = []
        self.time_op = []
        self.status = []
        self.id = id

        # TODO initialise and update this, we should clean it a bit
        self. agents_id
        self. agents
        self.pose

    def one_step(self, uPred = None, xPred = None):

        tic = time.time()
        feas, raw, planes = self.Controller.solve(self.x0, xPred, uPred, self.agents, self.agents_id, self.pose) # TODO redundant variables! mayb worth to refactor (agents_id and pose)
        self.time_op.append(time.time() - tic)
        if not feas:
            return feas,uPred, xPred, planes, raw

        if (self.Controller.sPred[:,1] >= 0.1).any():
            msg = "WARNING slack violated !"
            warnings.warn(msg)
            print(self.Controller.sPred[:,1:])

        uPred, xPred = self.Controller.uPred, self.Controller.xPred
        self.save(xPred, uPred, feas, planes)
        # TODO: send the xPred to the system
        return feas,uPred, xPred, planes, raw

    def gather_states(self,x_pred):
        # TODO: forward this to the system
        self.agents = np.swapaxes(np.asarray(x_pred)[:, :, -2:],0,1)


def main():

    #########################################################
    #########################################################

    # Map settings

    n_agents = settings["n_agents"]
    N        = settings["N"]
    dt       = settings["dt"]
    max_it   = settings["max_it"]
    # set constants

    x_pred = [None] * n_agents
    u_pred = [None] * n_agents

    it = 0

    x0 = x0_database[0:n_agents]
    ns = [[i for i in range(0, n_agents)] for j in range(0, n_agents)]

    for j, n in enumerate(ns):
        n.remove(j)

    maps = [Map(settings["map_type"])]*n_agents # TODO This needs to be done through the class

    agents,x_old,u_old = initialise_agents(x0,N,dt,maps)

    rs = agent(settings,  x_old[0][0,:]) #TODO initialise with our x0

    io = io_class(settings, rs)

    while(it<max_it and not checkEnd(x_pred, maps)):

        io.tic()
        feas, u_pred, x_pred, planes, raws = rs.one_step(u_old, x_old)

        if not feas:
            break

        rs.x0 = x_pred[1, :]

        io.toc()

        u_old = u_pred
        x_old = x_pred[1:, :]

        io.update( x_pred, u_pred ,agents, it)
        it += 1

    io.update(x_pred, u_pred, agents, it, end=True)

if __name__ == "__main__":

    main()




