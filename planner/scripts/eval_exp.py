import pickle

from plan_lib.mapManager import Map
from plan_lib.plotter import plotter_offline

from config_files.config_NL import settings as settings
import numpy as np


np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# USED TO REPRODUCE OLD EXPERIMETNS; NEEDS TO BE UPDATED

def load_s(id = 0):
    # Load states of agent id
    path = settings["path_pck"] + "pck/" + str(id) + "/"
    with open(path + 'states.pkl', 'rb') as f:  # Python 3: open(..., 'wb')
        return pickle.load(f)

class proxy():
    # proxy class to be able to use the rest of the library
    def __init__(self,states):
        self.states = states

def main():

    #########################################################
    #########################################################
    # set constants

    maps = [Map(settings["map_type"])] # load map

    states = load_s() # load states
    d = plotter_offline(maps[0]) # make an offline plotter

    for s in states:
        d.animate_step(s) # print an state to the screen
        input("Press enter to continue...")


if __name__ == "__main__":

    main()




