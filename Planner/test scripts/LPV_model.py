
# Global Variables
import sys
import numpy as np

sys.path.append(sys.path[0]+'/distributedPlanner')
sys.path.append(sys.path[0]+'/utilities')
sys.path.append(sys.path[0]+'/plotter')
sys.path.append(sys.path[0]+'/config/LPV')

from LPV_model_val import LPV_Model
from trackInitialization import Map, wrap
from plot_tools import *
from utilities import checkEnd, initialise_agents
from config import *  #Important!! Containts system definitions
import pickle

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


class agent(initialiserLPV):
    # Agents class, interfaces with the planner, saves data etc
    #  Q: [vx ; vy ; psiDot ; e_psi ; s ; e_y]
    #  R:  [delta ; a] la R es sobre el dU
    def __init__(self, N, dt, Map):
        super().__init__() # initialise the initialiser
        self.map = Map
        self.N = N
        self.dt = dt
        self.model = LPV_Model(N, dt, self.map, model_param = None, sys_lim = None)
        self.states = []
    def save(self, xPred):

        self.states.append(xPred[0,:])


def load_u(id = 0):
    path = path_csv + "/" + str(id) + "/"
    with open(path + '/u.pkl', 'rb') as f:  # Python 3: open(..., 'wb')
        return pickle.load(f)



def main():

    #########################################################
    #########################################################
    # set constants

    x0 = x0_database[0]

    maps = [Map(map_type)]

    agents,x_old,u_old = initialise_agents([x0],N,dt,maps)

    r = agent(N, dt, maps[0])
    r.model.xPred = x_old[0]
    us = load_u()

    for u in us:

        states = r.model.sim(u)
        r.save(states)

        print("--------------------------------------------------------------")
        print(states)
        print("--------------------------------------------------------------")

    if plot_end or error:
        d = plotter_offline(maps[0])
        rs =[r]
        for j,r in enumerate(rs):
            d.plot_offline_experiment(r, color_list[j], path = path_img)


    input("Press enter to exit...")


if __name__ == "__main__":

    main()




