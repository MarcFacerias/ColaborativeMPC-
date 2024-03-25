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

from plan_lib.utilities.misc import predicted_vectors_generation

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# USED TO RUN THE LPV EXPERIMENTS, as it is worked for the paper

def get_error(plan, map):
    forward_error = np.zeros((plan.shape[0], 3))
    backward_error = np.zeros((plan.shape[0], 3))

    for i in range(plan.shape[0]):
        forward_error[i, :] = map.getGlobalPosition(plan[i, 6], plan[i, 3])
        backward_error[i, :] = map.getLocalPosition(plan[i, 7], plan[i, 8], plan[i, 5])[0:3]

    return forward_error, backward_error

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
        feas, raw, planes = self.Controller.solve(xPred[0,:], xPred, uPred, agents, agents_id, pose)
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

        k = 30

        auxx, auxu = predicted_vectors_generation(int((k) * exp["scl"]), x_pred[k, :], dt / exp["scl"], maps[0])

        x_old = x_pred #np.vstack((x_pred[k+1:, :], auxx[::exp["scl"],:]))
        u_old = u_pred #np.vstack((u_pred[k:, :], auxu[::exp["scl"], :]))


    fwd, bwd = get_error(x_pred, maps[0])

    mdic_x = {"states": x_pred, "dt": exp["dt"], "N": exp["N"], "fwd": fwd, "bwd": bwd}
    mdic_u = {"control": u_pred}


    savemat("iCTL/x_" + exp["tag"] + ".mat", mdic_x)
    savemat("iCTL/u_" + exp["tag"] + ".mat", mdic_u)

    # return x_old
if __name__ == "__main__":

    exps = [
        {"N": 40, "dt": 0.025, "scl": 10, "tag": "Errors"},

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
