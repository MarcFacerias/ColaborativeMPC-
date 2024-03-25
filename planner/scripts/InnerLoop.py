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
from plan_lib.plotter.plot_tools import get_numeric_limtis
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
    def __init__(self, settings, maps, x0, id, scaler = None):
        super().__init__(self, settings) # initialise the initialiser
        self.map = maps
        self.N = settings["N"]
        self.dt = settings["dt"]
        self.Controller = PlannerLPV(self.Q, self.Qs, self.R, self.dR, self.N, self.dt, self.map, id, self.wq, self.model_param, self.sys_lim,
                                     scaler)
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


def main(exp):

    #########################################################
    #########################################################

    # Map settings
    settings["N"] = exp["N"]
    settings["dt"] = exp["dt"]
    n_agents = settings["n_agents"]
    map_type = settings["map_type"]
    N        = settings["N"]
    dt       = settings["dt"]
    # set constants

    x0 = x0_database[0:n_agents]
    ns = [[i for i in range(0, n_agents)] for j in range(0, n_agents)]

    for j, n in enumerate(ns):
        n.remove(j)

    maps = [Map(map_type)]* n_agents

    agents,x_old,u_old = initialise_agents(x0,N,dt,maps, scaler=exp["scl"])

    x_old = x_old[0]
    u_old = u_old[0]

    sim = agent(settings, maps[0], x_old[0, :], 0, exp["scl"])
    conv = False
    it = 0

    for i in range(3):
        feas, u_pred, x_pred, planes, raws = sim.one_step(u_old, x_old)
        x_old = x_pred
        u_old = u_pred

    mdic_x = {"states": x_pred, "dt": exp["dt"], "N": exp["N"]}
    mdic_u = {"control": u_pred}


    savemat("iCTL/x_" + exp["tag"] + ".mat", mdic_x)
    savemat("iCTL/u_" + exp["tag"] + ".mat", mdic_u)

    # return x_old
if __name__ == "__main__":

    exps = [
        # {"N":300, "dt":0.0125, "scl": None, "tag": "10ms"},
        # {"N": 30, "dt": 0.125, "scl": None, "tag": "125ms"},
        # {"N": 30, "dt": 0.125, "scl": 4, "tag": "125msScaled"},
        # {"N": 150, "dt": 0.025, "scl": None, "tag": "25ms"},
        {"N": 60, "dt": 0.05, "scl": None, "tag": "50ms"},
        {"N": 120, "dt": 0.025, "scl": None, "tag": "25ms"},
        {"N": 60, "dt": 0.05, "scl": 2, "tag": "50msScl2"},
        {"N": 30, "dt": 0.1, "scl": 4, "tag": "50msScl4"},

        # {"N": 10, "dt": 0.1, "scl": 5, "tag": "100ms_scl5"}
    ]
    tags = []
    for exp in exps:

        main(exp)
        tags += [exp["tag"]]

    tgs = {"a": tags, "label": "tags"}

    savemat("iCTL/tags.mat", tgs)

    Points0, Points1, Points2 = get_numeric_limtis(Map(settings["map_type"]))
    lims = {"cen": Points0, "up": Points1, "low": Points2}
    savemat("iCTL/road.mat", lims)
