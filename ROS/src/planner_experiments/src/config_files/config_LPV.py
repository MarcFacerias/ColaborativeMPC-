from plan_lib.utilities import path_gen, save_config
import matplotlib.colors as mcolors
import numpy as np


#Configuration file used to set up an experiment, ideally this should be a json or something like this
# gains
Qs = 10000000 * np.eye(3) # cost asociated to the slack variables
Q = np.diag([10.0, 0.0, 0.0, 25.0, 10.0, 0.0, 0.0, 0, 0]) #cost asociated to the states
R = 0 * np.diag([1, 1]) # cost asociated to the control actions
dR = 50 * np.diag([1, 1]) # cost asociated to the deritive of u
wq = 5.0 # weight asociated to the distance reward

settings = {
    "plot" : -1, # 0: none, 1: online, 2: offline, -1: only save picture
    "save_data" : True, # save data at the end of the experiment
    "verb" : 2, # levels of verbosity, 0 to 1
    "color_list" : list(mcolors.TABLEAU_COLORS),
    "n_agents" : 3, # number of agents
    "max_it" : 5000, # max duration of the experiment
    "min_dist": 0.25,
    "N" : 75, #planning horizon
    "dt" : 0.025, #model sampling time
    "vx_ref" : 3.0, # velocity we aim to reach

    "map_type" : "Highway", # track used in the experiment

    # Gains
    "Q"  : Q,
    "Qs" : Qs,
    "R"  : R,
    "dR" : dR,
    "wq" : wq,

    #ROS
    "log_agent" : 0, # agent info that will be displayed
}

path_gen(settings, "LPVR_agent") # function to generate paths to save data
save_config(settings) # save the configuration file with the experiment to track the settings