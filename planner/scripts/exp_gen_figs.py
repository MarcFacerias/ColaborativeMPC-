import pickle
import os
from plan_lib.mapManager import Map
from plan_lib.plotter import plotter_offline

from config_files.config_NL import settings as settings_NL
import matplotlib.colors as mcolors
import numpy as np

# USED TO PLOT TWO AGENTS TO COMPARE LPV WITH NONL FORMULATIONS

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
def load_s(path):
    # Load states of agent id

    with open(path + '/states.pkl', 'rb') as f:  # Python 3: open(..., 'wb')
        return pickle.load(f)

class proxy():
    # proxy class to be able to use the rest of the library
    def __init__(self,states):
        self.states = []
        for state in states:
            self.states.append(state[0,:])

def main():

    #########################################################
    #########################################################
    # set constants
    name = "NL_3agents_def"
    files = "data/"+ name + "/pck/"

    maps = [Map(settings_NL["map_type"])] # load map
    path = "data/results/" + name + "_"
    agents = []
    for file_path in os.listdir(files):
        # check if current file_path is a file
        if os.path.isdir(os.path.join(files, file_path)):
            # add filename to list
            agents.append(proxy(load_s(os.path.join(files, file_path))))

    colors = list(mcolors.TABLEAU_COLORS)

    d = plotter_offline(maps[0]) # make an offline plotter
    for i, agent in enumerate(agents):
        d.plot_offline_experiment(agent,path=path,style_agent = colors[i],legend="LPV")


if __name__ == "__main__":

    main()




