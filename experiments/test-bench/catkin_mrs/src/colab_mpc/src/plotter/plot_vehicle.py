#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Author: J. Noonan
# Email: jpnoonan@berkeley.edu
#
# This code provides a way to see the car's trajectory, orientation, and velocity profile in
# real time with referenced to the track defined a priori.
#
# ---------------------------------------------------------------------------
import sys
sys.path.append(sys.path[0]+'/ControllerObject')
sys.path.append(sys.path[0]+'/Utilities')

import matplotlib.colors as mcolors
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

np.set_printoptions(formatter={'float': lambda x: "{0:0.1f}".format(x)})

class plotter_offline():

    def __init__(self,map):

        self.fig = plt.figure(figsize=(10, 8))
        plt.ion()
        self.axtr = plt.axes()

        for j in range(0, map.PointAndTangent.shape[-1]):
            Points = int(np.floor(10 * (map.PointAndTangent[-1, 3, j] + map.PointAndTangent[-1, 4, j])))
            Points1 = np.zeros((Points, 3))
            Points2 = np.zeros((Points, 3))
            Points0 = np.zeros((Points, 3))

            for i in range(0, int(Points)):
                Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth, j)
                Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth, j)
                Points0[i, :] = map.getGlobalPosition(i * 0.1, 0, j)

            plt.plot(map.PointAndTangent[:, 0, j], map.PointAndTangent[:, 1, j], 'o')
            plt.plot(Points0[:, 0], Points0[:, 1], '--')
            plt.plot(Points1[:, 0], Points1[:, 1], '-b')
            plt.plot(Points2[:, 0], Points2[:, 1], '-b')

    def add_agent(self, agent, style):

        states = np.concatenate( agent.states, axis=0 ).reshape((-1,9))
        plt.plot(states[:, 7],states[:, 8], style)
        self.fig.canvas.draw()
        plt.pause(0.001)

    def add_agent_ti(self, agent, style):

        states = np.concatenate( agent.states[-1], axis=0 ).reshape((-1,9))
        plt.plot(states[:, 7],states[:, 8], style)
        self.fig.canvas.draw()
        plt.pause(0.001)

    def add_planes_ti(self, agent):

        for row in range(0, agent.planes.shape[0]):

            # Define hyperplane equation
            a = agent.planes[row, 0]
            b = agent.planes[row, 1]
            c = -agent.planes[row, 2]

            # Generate random data points
            y = np.random.rand(10)

            # Calculate values of x and y based on hyperplane equation

            if isclose(b, 0):
                y_hyperplane = (-b * y - c) / a
                plt.plot(y_hyperplane, y, color='red')
            else:
                y_hyperplane = (-a * y - c) / b
                plt.plot(y, y_hyperplane, color='red')

            self.fig.canvas.draw()
            plt.pause(0.001)



class plotter():

    def __init__(self,map, n_agents):
        ( self.fig, self.axtr, self.rec, self.rec_sim ) = _initializeFigure_xy(map, n_agents)
        self.l = 0.2
        self.w = 0.2

    def plot_step(self,x_sim,y_sim, psi_sim, i = 0):

        car_sim_x, car_sim_y = getCarPosition(x_sim, y_sim, psi_sim, self.w, self.l)
        self.rec_sim[i].set_xy(np.array([car_sim_x, car_sim_y]).T)

        self.fig.canvas.draw()
        plt.pause(0.001)



def getCarPosition(x, y, psi, w, l):
    car_x = [ x + l * np.cos(psi) - w * np.sin(psi), x + l * np.cos(psi) + w * np.sin(psi),
              x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
    car_y = [ y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
              y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]
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

