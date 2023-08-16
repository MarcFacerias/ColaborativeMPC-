
import os
from math import ceil

import matplotlib.colors as mcolors
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

np.set_printoptions(formatter={'float': lambda x: "{0:0.1f}".format(x)})

default_color = list(mcolors.TABLEAU_COLORS)
class plotter_offline():

    '''
    https://osqp.org/docs/interfaces/status_values.html
    '''
    def __init__(self,map, sm = 1):
        self.l = 0.15
        self.w = 0.15
        self.fig, self.axtr = _initializeFigure_xy(map, sm = sm)

    def add_agent(self, agent, style):

        states = np.concatenate( agent.states, axis=0 ).reshape((-1,9))
        plt.plot(states[:, 7],states[:, 8], style)
        self.fig.canvas.draw()
        plt.pause(0.001)

    def plot_offline_experiment(self, agent, path = None, style_agent = ".b", show = False ):
        states = np.concatenate( agent.states, axis=0 ).reshape((-1,9))
        plt.plot(states[:, 7],states[:, 8], style_agent)

        if path is not None:
            n = ceil(states.shape[0]/15)
            # colors = get_color_gradient(n) # TODO fix this!
            for j in range(0,states.shape[0],n):
                plt.plot(states[j, 7], states[j, 8], 'k', marker='x')

            if not os.path.exists(path):
                os.makedirs(path, exist_ok=True)

            plt.savefig(path + "track.png")

        if show:
            self.fig.canvas.draw()
            plt.pause(0.001)

    def animate_offline_experiment(self, sys, style_agents = default_color):

        N = len(sys[0].sPred_hist)
        rec_sim = initialise_cars(self.axtr, len(sys), style_agents)
        lines = ['']*len(sys)
        for i in range(0,N):

            for j,s in enumerate(sys):

                car_sim_x, car_sim_y = getCarPosition(s.sPred_hist[i][0,7], s.sPred_hist[i][0,8], s.sPred_hist[i][0,4], self.w, self.l)
                rec_sim[j].set_xy(np.array([car_sim_x, car_sim_y]).T)
                lines[j] = plt.plot(s.sPred_hist[i][1::,7],s.sPred_hist[i][1::,8],style_agents[j])

            plt.show()
            self.fig.canvas.draw()
            plt.pause(0.001)

            for line in lines:
                line[0].remove()

    def animate_step(self, xPred, style_agents = default_color):

        plt.plot(xPred[:,7],xPred[:,8],style_agents[0])

        plt.show()
        self.fig.canvas.draw()
        plt.pause(0.001)



    def plot_map(self,path = None):
        plt.show()
        self.fig.canvas.draw()
        plt.pause(0.001)
        if not path is None:
            if not os.path.exists(path):
                os.makedirs(path, exist_ok=True)

            plt.savefig(path + "track.png")

class plotter():

    def __init__(self,map, n_agents):
        self.fig, self.axtr = _initializeFigure_xy(map)
        self.rec_sim = initialise_cars(self.axtr, n_agents)
        self.l = 0.15
        self.w = 0.15

    def plot_step(self,x_sim,y_sim, psi_sim, i = 0):

        car_sim_x, car_sim_y = getCarPosition(x_sim, y_sim, psi_sim, self.w, self.l)
        self.rec_sim[i].set_xy(np.array([car_sim_x, car_sim_y]).T)

        self.fig.canvas.draw()
        plt.pause(0.001)



def getCarPosition(x, y, psi, w, l, shape = "triangle"):

    if shape == "square":
        car_x = [ x + l * np.cos(psi) - w * np.sin(psi), x + l * np.cos(psi) + w * np.sin(psi),
                  x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
        car_y = [ y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
                  y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]

    else:
        # Triangle
        car_x = [ x + 1.2*l * np.cos(psi), x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
        car_y = [ y + 1.2*l * np.sin(psi), y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]



    return car_x, car_y

# ===================================================================================================================================== #
# ============================================================= Internal Functions ==================================================== #
# ===================================================================================================================================== #

def _initializeFigure_xy(map, sm = 1):
    fig = plt.figure(figsize=(10,8))
    plt.ion()
    axtr = plt.axes()

    for j in range(0,map.PointAndTangent.shape[-1]):
        Points = int(np.floor(10 * (map.PointAndTangent[-1, 3, j] + map.PointAndTangent[-1, 4, j])))
        Points1 = np.zeros((Points, 3))
        Points2 = np.zeros((Points, 3))
        Points0 = np.zeros((Points, 3))
        Points3 = np.zeros((Points, 3))
        Points4 = np.zeros((Points, 3))

        for i in range(0, int(Points)):
            Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth, j, True)
            Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth, j, True)
            Points0[i, :] = map.getGlobalPosition(i * 0.1, 0, j, True)
            Points3[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth * sm, j, True)
            Points4[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth * sm, j, True)

        plt.plot(map.PointAndTangent[:, 0, j], map.PointAndTangent[:, 1, j], 'o')
        plt.plot(Points0[:, 0], Points0[:, 1], linestyle='dashed')
        plt.plot(Points1[:, 0], Points1[:, 1], '-b')
        plt.plot(Points2[:, 0], Points2[:, 1], '-b')
        plt.plot(Points3[:, 0], Points3[:, 1], '-b', linestyle='dashed')
        plt.plot(Points4[:, 0], Points4[:, 1], '-b', linestyle='dashed')

    return fig, axtr


def initialise_cars (axtr, n_agents, style_agents = default_color):

    v = np.array([[ 1.,  1.],
                  [ 1., -1.],
                  [-1., -1.],
                  [-1.,  1.]]) /5

    rec_sim_list = []

    for i in range(0,n_agents):

        rec_sim = patches.Polygon(v, alpha=0.7, closed=True,facecolor=style_agents[i] ,ec='k', zorder=10)
        rec_sim_list.append(rec_sim)
        axtr.add_patch(rec_sim_list[-1])

    return rec_sim_list
# ===================================================================================================================================== #
# ========================================================= End of Internal Functions ================================================= #
# ===================================================================================================================================== #

def plot_performance( agent):

    fig_status = plt.figure()
    fig_status.add_subplot(2, 1, 1)
    x = np.arange(0,len(agent.status))
    plt.scatter(x, np.array(agent.status))
    fig_status.add_subplot(2, 1, 2)
    plt.scatter(x, np.array(agent.time_op))
    plt.show()
    plt.pause(0.001)

def plot_distance( distance_hist, th):

    fig_status = plt.figure(3)
    x = np.arange(0,len(distance_hist))
    plt.scatter(x, np.array(distance_hist))
    plt.plot(x, th*np.ones(len(distance_hist)), "-r")
    plt.show()
    plt.pause(0.001)


def hex_to_RGB(hex_str):
    """ #FFFFFF -> [255,255,255]"""
    #Pass 16 to the integer function for change of base
    return [int(hex_str[i:i+2], 16) for i in range(1,6,2)]

def get_color_gradient(n, c1 = '#FFFFFF', c2 = '#000000'):
    """
    Given two hex colors, returns a color gradient
    with n colors.
    """
    assert n > 1
    c1_rgb = np.array(hex_to_RGB(c1))/255
    c2_rgb = np.array(hex_to_RGB(c2))/255
    mix_pcts = [x/(n-1) for x in range(n)]
    rgb_colors = [((1-mix)*c1_rgb + (mix*c2_rgb)) for mix in mix_pcts]
    return ["#" + "".join([format(int(round(val*255)), "02x") for val in item]) for item in rgb_colors]