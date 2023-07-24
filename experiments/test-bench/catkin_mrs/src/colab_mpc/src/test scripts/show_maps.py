
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time
import os

sys.path.append(sys.path[0]+'/Utilities')
sys.path.append(sys.path[0]+'/plotter')

from trackInitialization import Map, wrap
from plot_vehicle import *

plot = False
plot_end = True
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

def main():

    path = r"I:\Marc\PhD\ColaborativeMPC-\experiments\test-bench\catkin_mrs\src\colab_mpc\src\drawings\png\map.png"
    disp = plotter_offline(Map("Highway"))
    disp.plot_map(path)

    input("Press enter to continue...")

if __name__ == "__main__":

    main()



