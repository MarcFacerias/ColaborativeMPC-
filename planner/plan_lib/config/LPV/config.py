# Imports
import matplotlib.colors as mcolors
import numpy as np
from planner.plan_lin.config import experiment_utilities
from planner.plan_lin.utilities import path_gen, save_config

settings = {
    "plot" : -1, # 0: none, 1: online, 2: offline, -1: only save picture
    "save_data" : True,
    "verb" : 2,
    "color_list" : list(mcolors.TABLEAU_COLORS),
    "n_agents" : 3,
    "max_it" : 3000,
    "N" : 125,
    "dt" : 0.025,
    "vx_ref" : 3.0,

    "map_type" : "Highway",
    # "map_type" : "oval",
    # "map_type" : "SL",

}

path_gen(settings, "LPV3_agent")
save_config(settings)
class initialiserLPV(experiment_utilities):
    def __init__(self, data, settings, model = "SCALED CAR"):
        super().__init__(data, settings, model)

        self.Qs = 10000000 * np.eye(3)
        self.Q  = np.diag([10.0, 0.0, 0.0, 25.0, 10.0, 0.0, 0.0, 0, 0])
        # self.Q  = np.diag([10.0, 0.0, 0.0, 100.0, 50.0, 0.0, 0.0, 0, 0])
        self.R  = 0 * np.diag([1, 1])
        self.dR = 50 * np.diag([1, 1])
        self.wq = 5.0