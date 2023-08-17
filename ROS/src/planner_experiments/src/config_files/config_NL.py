import matplotlib.colors as mcolors
import numpy as np
from plan_lib.utilities import path_gen, save_config

## SEE LPV CONFIG##

# gains
Qs = 10000000 * np.eye(3)
Q = np.diag([10.0, 0.0, 0.0, 15.0, 10.0, 0.0, 0.0, 0, 0])
R = 0 * np.diag([1, 1])
dR = 50 * np.diag([1, 1])
wq = 5.0

settings = {
    "plot" : -1, # 0: none, 1: online, 2: offline, -1: only save picture
    "save_data" : True,
    "verb" : 2,
    "color_list" : list(mcolors.TABLEAU_COLORS),
    "n_agents" : 3,
    "max_it" : 4000,
    "min_dist": 0.25, # maximum distance to the neigh
    "N" : 75,
    "dt" : 0.025,
    "vx_ref": 3.0,

    "map_type" : "Highway",

    #OCD specific
    "it_conv" : 2, # iterations that need to happen without changes on x
    "max_it_OCD" : 50, # maximum OCD iterations
    "min_it_OCD": 2, # minimum OCD iterations
    "verb_OCD" : True, # verbosity of the OCD algorithm

    # Gains
    "Q": Q,
    "Qs": Qs,
    "R": R,
    "dR": dR,
    "wq": wq,

    # ROS
    "log_agent": 0,
}

path_gen(settings, "NLR_test_sh3", ros=True)
save_config(settings)
