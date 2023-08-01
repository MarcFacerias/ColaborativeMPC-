# Imports
import matplotlib.colors as mcolors
import numpy as np

from Planner.packages.config import experiment_utilities

settings = {
    "plot" : -1, # 0: none, 1: online, 2: offline, -1: only save picture
    "save_data" : True,
    "verb" : 2,
    "color_list" : list(mcolors.TABLEAU_COLORS),
    "n_agents" : 1,
    "max_it" : 5000,
    "min_dist": 0.25,
    "N" : 15,
    "dt" : 0.01,
    "map_type" : "Highway",
    # map_type : "oval",
    # map_type : "SL",
    "path_csv" : "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/data/TestNL/",
    "path_img" : "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/data/TestNL/",
    "path_pck": "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/data/TestNL/",

    #OCD specific
    "it_conv" : 1,
    "max_it_OCD" : 10,
    "verb_OCD" : True,
    "LPV": True,
}

x0_database = [""] * 4
x0_database[0] = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.45]  # [vx vy psidot y_e thetae theta s x y]
x0_database[1] = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]
x0_database[2] = [1.3, -0.16, 0.00, 0.25, 0, 0.0, 0.25, 0.0, 1.5]  # [vx vy psidot y_e thetae theta s x y]
x0_database[3] = [1.3, -0.16, 0.00, -0.25, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]

def get_alpha():

    alpha = 0.25
    return alpha

class initialiserNL(experiment_utilities):
    def __init__(self, data, path, path_pickle, model = "SCALED CAR"):
        super().__init__(data, path, path_pickle, model)
        self.Qs = np.diag([10000000,1000000,1000000])
        self.Q = np.diag([10.0, 0.0, 0.0, 200.0, 50.0, 0.0, 0.0, 0, 0])
        self.R = 0 * np.diag([1, 1])
        self.dR = 50 * np.diag([1, 1])

def eval_constraintEU(x1, x2, D):

    cost1 = D - np.sqrt(sum((x1-x2)**2)) # the OCD update depends on on the diference between the minimum D and the euclidean dist

    return np.array(cost1)