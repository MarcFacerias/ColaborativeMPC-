
import sys

import numpy as np
from Planner.packages.mapManager import Map
from Planner.packages.plotter import plotter_offline

plot = False
plot_end = True
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

def main():

    path = sys.path[0] + "/drawings/"
    disp = plotter_offline(Map("SL"))
    disp.plot_map(path)

    input("Press enter to continue...")

if __name__ == "__main__":

    main()




