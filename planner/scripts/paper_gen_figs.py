

import pickle

from plan_lib.mapManager import Map
from plan_lib.plotter import plotter_offline

from config_files.config_NL import settings as settings_NL
import matplotlib.colors as mcolors
import numpy as np

# USED TO PLOT TWO AGENTS TO COMPARE LPV WITH NONL FORMULATIONS

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
def load_s(path):
    # Load states of agent id

    with open(path + 'states.pkl', 'rb') as f:  # Python 3: open(..., 'wb')
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
    files_CS = "data/experiments_paper/NL_3agents_lh/pck/2/"
    files_LPV = "data/experiments_paper/LPV3_agent_lh/pck/2/"

    maps = [Map(settings_NL["map_type"])] # load map
    path = "data/experiments_paper/two_agents/"
    agent_NL = proxy(load_s(files_CS))
    agent_LPV = proxy(load_s(files_LPV))
    colors = list(mcolors.TABLEAU_COLORS)

    d = plotter_offline(maps[0]) # make an offline plotter

    d.plot_offline_experiment( agent_LPV,path=path,style_agent = colors[0],legend="LPV")
    d.plot_offline_experiment( agent_NL,path=path, style_agent = colors[1],legend="NL")


if __name__ == "__main__":

    main()




