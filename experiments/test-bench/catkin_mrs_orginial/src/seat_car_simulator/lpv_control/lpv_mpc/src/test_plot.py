#!/usr/bin/env python2.7
import numpy as np
from Utilities.trackInitialization_mod import Map
import matplotlib.pyplot as plt
import pdb
import matplotlib.patches as patches

import scipy.io as sio

def _initializeFigure_xy(map, mode):
    xdata = []; ydata = []
    fig = plt.figure(figsize=(10,8))
    plt.ion()
    axtr = plt.axes()
    # print("-------------- point tangent --------------")
    # print(map.PointAndTangent[:, :, 0])
    # print("-------------- point tangent --------------")
    # print(map.PointAndTangent[:, :, 1])
    # print("-------------- point tangent --------------")

    for j in range(0,map.PointAndTangent.shape[-1]):

        Points = int(np.floor(10 * (map.PointAndTangent[-1, 3, j] + map.PointAndTangent[-1, 4,j] +10)))

        Points1 = np.zeros((Points, 3))
        Points2 = np.zeros((Points, 3))
        Points0 = np.zeros((Points, 3))

        for i in range(0, int(Points)):
            # print("S: ", i * 0.1, "Vector: ",map.getGlobalPosition(i * 0.1, map.halfWidth, j))
            Points1[i, :] = map.getGlobalPosition(i * 0.1, map.halfWidth, j)
            Points2[i, :] = map.getGlobalPosition(i * 0.1, -map.halfWidth, j)
            Points0[i, :] = map.getGlobalPosition(i * 0.1, 0, j)
            

        plt.plot(map.PointAndTangent[:, 0, j], map.PointAndTangent[:, 1, j], 'o')
        plt.plot(Points0[:, 0], Points0[:, 1], '--')
        plt.plot(Points1[:, 0], Points1[:, 1], '-b')
        plt.plot(Points2[:, 0], Points2[:, 1], '-b')

        # These lines plot the planned offline trajectory in the main figure:
        # plt.plot(X_Planner_Pts[0, 0:290], Y_Planner_Pts[0, 0:290], '--r')
        # plt.plot(X_Planner_Pts[0, 290:460], Y_Planner_Pts[0, 290:460], '--r')
        # plt.plot(X_Planner_Pts[0, :], Y_Planner_Pts[0, :], '--r')


        line_cl,        = axtr.plot(xdata, ydata, '-k')
        line_gps_cl,    = axtr.plot(xdata, ydata, '--ob')  # Plots the traveled positions
        line_tr,        = axtr.plot(xdata, ydata, '-or')       # Plots the current positions
        line_SS,        = axtr.plot(xdata, ydata, 'og')
        line_pred,      = axtr.plot(xdata, ydata, '-or')
        line_planning,  = axtr.plot(xdata, ydata, '-ok')

        v = np.array([[ 1.,  1.],
                      [ 1., -1.],
                      [-1., -1.],
                      [-1.,  1.]])

    plt.show(block=True)

        # rec = patches.Polygon(v, alpha=0.7, closed=True, fc='r', ec='k', zorder=10)
        # axtr.add_patch(rec)
        #
        # # Vehicle:
        # rec_sim = patches.Polygon(v, alpha=0.7, closed=True, fc='G', ec='k', zorder=10)
        #
        # if mode == "simulations":
        #     axtr.add_patch(rec_sim)

        # Planner vehicle:
        # rec_planning = patches.Polygon(v, alpha=0.7, closed=True, fc='k', ec='k', zorder=10)
        # axtr.add_patch(rec_planning)



        # return fig, axtr, line_planning, line_tr, line_pred, line_SS, line_cl, line_gps_cl, rec, rec_sim, rec_planning

_initializeFigure_xy(Map(), "mode")
print("Hello")