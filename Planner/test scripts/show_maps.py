
import sys
import numpy as np


sys.path.append(sys.path[0].strip("test scripts")+'utilities')
sys.path.append(sys.path[0].strip("test scripts")+'plotter')

from trackInitialization import Map, wrap
from plot_tools import *

plot = False
plot_end = True
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

def main():

    path = r"/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/drawings/map2.png"
    disp = plotter_offline(Map("SL"))
    disp.plot_map(path)

    input("Press enter to continue...")

if __name__ == "__main__":

    main()




