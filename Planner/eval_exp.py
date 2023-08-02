

from Planner.packages.mapManager import Map
from Planner.packages.plotter import *
from Planner.packages.config.LPV import *  #Important!! Containts system definitions

import pickle

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

def load_s(id = 0):
    path = settings["path_pck"] + "pck/" + str(id) + "/"
    with open(path + 'states.pkl', 'rb') as f:  # Python 3: open(..., 'wb')
        return pickle.load(f)



def main():

    #########################################################
    #########################################################
    # set constants

    maps = [Map(settings["map_type"])]

    states = load_s()
    d = plotter_offline(maps[0])

    for s in states:
        d.animate_step(s)
        input("Press enter to continue...")


if __name__ == "__main__":

    main()




