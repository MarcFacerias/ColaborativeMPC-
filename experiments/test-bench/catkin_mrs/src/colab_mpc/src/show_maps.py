
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

    disp = plotter_offline(Map("TestOpenMap"))
    # disp = plotter_offline(Map())
    disp.plot_map()
    input("Press enter to continue...")

if __name__ == "__main__":

    main()




