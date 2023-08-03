# Imports
import matplotlib.colors as mcolors
import numpy as np
from Planner.packages.config import experiment_utilities, path_gen

settings = {
    "plot" : -1, # 0: none, 1: online, 2: offline, -1: only save picture
    "save_data" : True,
    "verb" : 2,
    "color_list" : list(mcolors.TABLEAU_COLORS),
    "n_agents" : 3,
    "max_it" : 5000,
    "min_dist": 0.25,
    # "N" : 125,
    # "dt" : 0.025,
    "N" : 75,
    "dt" : 0.025,
    "vx_ref": 3.0,

    "map_type" : "Highway",
    # "map_type" : "oval",
    # "map_type" : "SL",

    #OCD specific
    "it_conv" : 2,
    "max_it_OCD" : 1000,
    "min_it_OCD": 2,
    "verb_OCD" : True,
    "LPV": False,
}

path_gen(settings, "NL3_agent_lh")

x0_database = [""] * 4
x0_database[0] = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.45]  # [vx vy psidot y_e thetae theta s x y]
x0_database[1] = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]
x0_database[2] = [1.3, -0.16, 0.00, 0.25, 0, 0.0, 0.25, 0.0, 1.5]  # [vx vy psidot y_e thetae theta s x y]
x0_database[3] = [1.3, -0.16, 0.00, -0.25, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]

def get_alpha():

    alpha = 0.25
    return alpha

class initialiserNL(experiment_utilities):
    def __init__(self, data, settings, model = "SCALED CAR"):
        super().__init__(data, settings, model)
        self.Qs = np.diag([10000000,1000000,1000000])
        self.Q  = np.diag([5.0, 0.0, 0.0, 20.0, 5.0, 0.0, 0.0, 0, 0])
        # self.Q = np.diag([5.0, 0.0, 0.0, 50.0, 10.0, 0.0, 0.0, 0, 0])
        self.R = 0 * np.diag([1, 1])
        self.dR = 25 * np.diag([1, 1])

def eval_constraintEU(x1, x2, D):

    cost1 = D - np.sqrt(sum((x1-x2)**2)) # the OCD update depends on on the diference between the minimum D and the euclidean dist

    return np.array(cost1)