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

import rospy
import matplotlib.colors as mcolors
import numpy as np
from trackInitialization import Map
from lpv_mpc.msg import pos_info, prediction, simulatorStates, My_Planning, Racing_Info
import matplotlib.pyplot as plt
import matplotlib.patches as patches

np.set_printoptions(formatter={'float': lambda x: "{0:0.1f}".format(x)})

def main():

    rospy.init_node("realTimePlotting")
    n_agents = rospy.get_param("/n_agents")
    data        = Estimation_Mesures_Planning_Data(n_agents)
    map         = Map()

    loop_rate   = 60.0
    rate        = rospy.Rate(loop_rate)

    ( fig, axtr, rec, rec_sim ) = _initializeFigure_xy(map, n_agents)

    while not rospy.is_shutdown():

        for j in range(0, n_agents):

            l = 0.2; w = 0.2

            x_sim   = data.sim_x[j][-1]
            y_sim   = data.sim_y[j][-1]
            psi_sim = data.sim_psi[j][-1]
            car_sim_x, car_sim_y = getCarPosition(x_sim, y_sim, psi_sim, w, l)
            rec_sim[j].set_xy(np.array([car_sim_x, car_sim_y]).T)

        fig.canvas.draw()
        rate.sleep()



def getCarPosition(x, y, psi, w, l):
    car_x = [ x + l * np.cos(psi) - w * np.sin(psi), x + l * np.cos(psi) + w * np.sin(psi),
              x - l * np.cos(psi) + w * np.sin(psi), x - l * np.cos(psi) - w * np.sin(psi)]
    car_y = [ y + l * np.sin(psi) + w * np.cos(psi), y + l * np.sin(psi) - w * np.cos(psi),
              y - l * np.sin(psi) - w * np.cos(psi), y - l * np.sin(psi) + w * np.cos(psi)]
    return car_x, car_y

class Estimation_Mesures_Planning_Data():
    """Object collecting closed loop data points
    Attributes:
        updateInitialConditions: function which updates initial conditions and clear the memory
    """
    def __init__(self, n_agents):

        self.plotGPS = rospy.get_param("/visualization/plotGPS")

        for i in range(0,int(n_agents)):
            base = "/agent" + str(i+1) + "/"

            print(base)
            rospy.Subscriber(base +"pos_info", pos_info, self.simState_callback,i)

        self.sim_x   = [[0.0] for x in xrange(n_agents)]
        self.sim_y   = [[0.0] for x in xrange(n_agents)]
        self.sim_psi = [[0.0] for x in xrange(n_agents)]
        self.sim_vx  = [[0.0] for x in xrange(n_agents)]
        self.sim_vy  = [[0.0] for x in xrange(n_agents)]
        self.psiDot  = [[0.0] for x in xrange(n_agents)]

    def simState_callback(self, msg, i):
        self.sim_x[i].append(msg.x)
        self.sim_y[i].append(msg.y)
        self.sim_psi[i].append(msg.psi)
        self.sim_vx[i].append(msg.vx)
        self.sim_vy[i].append(msg.vy)
        self.psiDot[i].append(msg.psiDot)

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

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
