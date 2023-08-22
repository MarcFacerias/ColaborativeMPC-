import matplotlib.colors as mcolors
import numpy as np
from plan_lib.utilities import path_gen, lbp_gen, save_config
# gains
Qs = 10000000 * np.eye(3)
Q = np.diag([10.0, 0.0, 0.0, 35.0, 20.0, 0.0, 0.0, 0, 0])
# self.Q  = np.diag([10.0, 0.0, 0.0, 100.0, 50.0, 0.0, 0.0, 0, 0])
R = 0.5 * np.diag([1, 1])
dR = 0.5 * np.diag([1, 1])
wq = 5.0

settings = {
    "plot" : -1, # 0: none, 1: online, 2: offline, -1: only save picture
    "save_data" : True,
    "verb" : 2,
    "color_list" : list(mcolors.TABLEAU_COLORS),
    "n_agents" : 3,
    "max_it" : 500,
    "min_dist": 0.25,
    # "N" : 125,
    "N" : 30,
    "dt" : 0.02,
    "vx_ref": 3.0,

    "map_type" : "Highway",
    # "map_type" : "oval",
    # "map_type" : "SL",

    #OCD specific
    "it_conv" : 2,
    "max_it_OCD" : 1000,
    "min_it_OCD": 2,
    "verb_OCD" : False,

    # Gains
    "Q": Q,
    "Qs": Qs,
    "R": R,
    "dR": dR,
    "wq": wq
}

path_gen(settings, "NL_3agents_20h_d2")
lbp_gen(settings, "lambdas")
save_config(settings)
