#!/usr/bin/env python

import sys
sys.path.append(sys.path[0]+'/ControllerObject')
sys.path.append(sys.path[0]+'/Utilities')

import matplotlib.colors as mcolors
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os

np.set_printoptions(formatter={'float': lambda x: "{0:0.1f}".format(x)})

class plotter_offline():

    '''
    https://osqp.org/docs/interfaces/status_values.html
    '''
    def __init__(self,map):

        self.fig = plt.figure(figsize=(10, 8))
        plt.ion()
        self.axtr = plt.axes()

        for j in range(0, map.PointAndTangent.shape[-1]):
            Points = int(np.floor(10 * (map.PointAndTangent[-1, 3, j] + map.PointAndTangent[-1, 4, j])))
            Points1 = np.zeros((Points, 3))
            Points2 = np.zeros((Points, 3))
            Points0 = np.zeros((Points, 3))
            Points3 = np.zeros((Points, 3))
            Points4 = np.zeros((Points, 3))

            for i in range(0, int(Points)):
                Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth, j)
                Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth, j)
                Points0[i, :] = map.getGlobalPosition(i * 0.1, 0, j)
                Points3[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth*0.8, j)
                Points4[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth*0.8, j)

            plt.plot(map.PointAndTangent[:, 0, j], map.PointAndTangent[:, 1, j], 'o')
            plt.plot(Points0[:, 0], Points0[:, 1], linestyle='dashed')
            plt.plot(Points1[:, 0], Points1[:, 1], '-b')
            plt.plot(Points2[:, 0], Points2[:, 1], '-b')
            plt.plot(Points3[:, 0], Points3[:, 1], '-b', linestyle='dashed')
            plt.plot(Points4[:, 0], Points4[:, 1], '-b', linestyle='dashed')
    def add_agent(self, agent, style):

        states = np.concatenate( agent.states, axis=0 ).reshape((-1,9))
        plt.plot(states[:, 7],states[:, 8], style)
        self.fig.canvas.draw()
        plt.pause(0.001)

    def plot_offline_experiment(self, agent, style_agent = ".b", path = None):
        states = np.concatenate( agent.states, axis=0 ).reshape((-1,9))
        plt.plot(states[:, 7],states[:, 8], style_agent)
        # self.fig.canvas.draw()
        # plt.pause(0.001)

        if not path is None:
            if not os.path.exists(path):
                os.makedirs(path, exist_ok=True)

            plt.savefig(path + "track.png")

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
        ( self.fig, self.axtr, self.rec, self.rec_sim ) = _initializeFigure_xy(map, n_agents)
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

def _initializeFigure_xy(map, n_agents):
    fig = plt.figure(figsize=(10,8))
    plt.ion()
    axtr = plt.axes()

    colors = mcolors.TABLEAU_COLORS
    names = list(colors)

    for j in range(0,map.PointAndTangent.shape[-1]):
        Points = int(np.floor(10 * (map.PointAndTangent[-1, 3,j] + map.PointAndTangent[-1, 4,j])))
        Points1 = np.zeros((Points, 3))
        Points2 = np.zeros((Points, 3))
        Points0 = np.zeros((Points, 3))



        for i in range(0, int(Points)):
            Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth,j)
            Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth,j)
            Points0[i, :] = map.getGlobalPosition(i * 0.1, 0,j)

        plt.plot(map.PointAndTangent[:, 0,j], map.PointAndTangent[:, 1,j], 'o')
        plt.plot(Points0[:, 0], Points0[:, 1], '--')
        plt.plot(Points1[:, 0], Points1[:, 1], '-b')
        plt.plot(Points2[:, 0], Points2[:, 1], '-b')

    v = np.array([[ 1.,  1.],
                  [ 1., -1.],
                  [-1., -1.],
                  [-1.,  1.]]) /5

    rec_list = []
    rec_sim_list = []

    for i in range(0,n_agents):

        rec_sim = patches.Polygon(v, alpha=0.7, closed=True,facecolor=colors[names[i]] ,ec='k', zorder=10)
        rec_sim_list.append(rec_sim)
        axtr.add_patch(rec_sim_list[-1])

    plt.show()

    return fig, axtr, rec_list, rec_sim_list

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
