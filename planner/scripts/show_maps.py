
import sys

import numpy as np
from plan_lib.mapManager import Map
from plan_lib.plotter import plotter_offline

# USED TO PLOT A MAP WITH MATPLOTLIB

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




