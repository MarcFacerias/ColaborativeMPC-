import time
import warnings
import random
from copy import copy, deepcopy

# Global Variables
import numpy as np

from plan_lib.IOmodule import io_class
from plan_lib.config import x0_database
from plan_lib.config.LPV import initialiserLPV
from plan_lib.mapManager import Map
from plan_lib.utilities import checkEnd, initialise_agents
from plan_lib.distributedPlanner import PlannerLPV

from config_files.config_LPV import settings

from scipy.io import savemat

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# USED TO RUN THE LPV EXPERIMENTS, as it is worked for the paper

class agent(initialiserLPV):
    # Agents class, interfaces with the planner, saves data etc
    #  Q: [vx ; vy ; psiDot ; e_psi ; s ; e_y]
    #  R:  [delta ; a] la R es sobre el dU
    def __init__(self, settings, maps, x0, id):
        super().__init__(self, settings) # initialise the initialiser
        self.map = maps
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

    def one_step(self, uPred, xPred, agents=None, agents_id=None, pose=None ):

        tic = time.time()
        feas, raw, planes = self.Controller.solve(self.x0, xPred, uPred, agents, agents_id, pose)
        self.time_op.append(time.time() - tic)
        if not feas:
            return feas,uPred, xPred, planes, raw

        if (self.Controller.sPred[:,1] >= 0.1).any():
            msg = "WARNING slack violated !"
            warnings.warn(msg)
            print(self.Controller.sPred[:,1:])

        uPred, xPred = self.Controller.uPred, self.Controller.xPred
        self.save(xPred, uPred, feas, planes)

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

    maps = [Map(map_type)]* n_agents

    agents,x_old,u_old = initialise_agents(x0,N,dt,maps)

    x_old = x_old[0]
    u_old = u_old[0]

    sim = agent(settings, maps[0], x_old[0, :], 0)
    conv = False
    it = 0
    while not conv and it < 25:
        feas, u_pred, x_pred, planes, raws = sim.one_step(u_old, x_old)
        conv = np.allclose(x_pred[:,[7,8]], x_old[:, [7, 8]], atol=0.05)
        # print(abs(x_pred[:, [7, 8]] - x_old[:, [7, 8]]))
        x_old = deepcopy(x_pred)
        u_old = deepcopy(u_pred)
        it += 1
    # start experiment

    x_gen = []
    u_gen = []
    exp_range = 100

    for _ in range(exp_range+1):

        if _ == 0:
            x_pol = x_old
            u_pol = u_old
        else:

            x_pol = x_old + np.random.uniform(-0.1,0.1, x_old.shape)
            u_pol = u_old + np.random.uniform(-0.1, 0.1, u_old.shape)

        feas, u_pred, x_pred, planes, raws = sim.one_step(u_pol, x_pol)

        x_gen += [x_pred]
        u_gen += [u_pred]

    # compute statistical measures
    print("Help")

    for i in range(1, 100):

        try:
            cum_x += (x_gen[i] - x_gen[0])**2
            cum_u += (u_gen[i] - u_gen[0]) ** 2

        except:
            cum_x = (x_gen[i] - x_gen[0]) ** 2
            cum_u = (u_gen[i] - u_gen[0]) ** 2

    x_dev = np.sqrt(cum_x/100)
    u_dev = np.sqrt(cum_u/100)

    mdic_x = {"a": x_dev, "label": "x_dev"}
    mdic_u = {"a": u_dev, "label": "u_dev"}

    mdic_rawx = {"a": np.asarray(x_gen), "label": "x_gen"}
    mdic_rawu = {"a": np.asarray(u_gen), "label": "u_gen"}

    savemat("spreading120/x_dev.mat", mdic_x)
    savemat("spreading120/u_dev.mat", mdic_u)
    savemat("spreading120/x_gen.mat", mdic_rawx)
    savemat("spreading120/u_gen.mat", mdic_rawu)


if __name__ == "__main__":

    main()




