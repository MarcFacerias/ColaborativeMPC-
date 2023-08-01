# Imports
import matplotlib.colors as mcolors
import numpy as np

from Planner.packages.config import experiment_utilities

settings = {
    "plot" : -1, # 0: none, 1: online, 2: offline, -1: only save picture
    "save_data" : True,
    "verb" : 0,
    "color_list" : list(mcolors.TABLEAU_COLORS),
    "n_agents" : 3,
    "max_it" : 5000,
    "N" : 15,
    "dt" : 0.01,

    "map_type" : "Highway",
    # map_type : "oval",
    # map_type : "SL",

    "path_csv" : "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/distributedPlanner/TestLPVPrePrint/",
    "path_img" : "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/distributedPlanner/TestLPVPrePrint/",
    "path_pck": "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/distributedPlanner/TestLPVPrePrint/",

}

class initialiserLPV(experiment_utilities):
    def __init__(self, data, path, path_pickle, model = "SCALED CAR"):
        super().__init__(data, path, path_pickle, model)

        self.Qs = 10000000 * np.eye(3)
        self.Q  = np.diag([10.0, 0.0, 0.0, 100.0, 50.0, 0.0, 0.0, 0, 0])
        self.R  = 0 * np.diag([1, 1])
        self.dR = 50 * np.diag([1, 1])
        self.wq = 500.0