from plan_lib.utilities import path_gen, save_config
import matplotlib.colors as mcolors
import numpy as np

# gains
Qs = 10000000 * np.eye(3)
Q = np.diag([10.0, 0.0, 0.0, 25.0, 10.0, 0.0, 0.0, 0, 0])
# self.Q  = np.diag([10.0, 0.0, 0.0, 100.0, 50.0, 0.0, 0.0, 0, 0])
R = 0 * np.diag([1, 1])
dR = 50 * np.diag([1, 1])
wq = 5.0

settings = {
    "plot" : -1, # 0: none, 1: online, 2: offline, -1: only save picture
    "save_data" : True,
    "verb" : 2,
    "color_list" : list(mcolors.TABLEAU_COLORS),
    "n_agents" : 1,
    "max_it" : 5000,
    "N" : 75,
    "dt" : 0.025,
    "vx_ref" : 3.0,

    "map_type" : "Highway",

    # Gains
    "Q"  : Q,
    "Qs" : Qs,
    "R"  : R,
    "dR" : dR,
    "wq" : wq,

    #ROS
    "log_agent" : 0,
}

path_gen(settings, "LPVR_agent")
save_config(settings)