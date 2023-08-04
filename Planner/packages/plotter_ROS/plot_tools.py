#!/usr/bin/env python

from Planner.packages.plotter import *
import numpy as np
np.set_printoptions(formatter={'float': lambda x: "{0:0.1f}".format(x)})


# proxy class used to match function calls
class proxy_agent():
    def __init__(self,states):
        self.states = states
    # change this so that the proxy is updated thorough ROS topics

class plotter_ROS():

    def __init__(self, settings, connections, map):

        self.plot = settings["plot"]
        self.n_agent = settings["n_agents"]
        self.color = settings["color_list"]
        self.path = settings["path_img"]
        self.sys = proxy_agent(connections) # change this so that the proxy is updated

        if self.plot == 1:
            self.disp = plotter(map, self.n_agent)

        elif self.plot != 0:
            self.disp = plotter_offline(map)

        self.connections = connections # TODO: make ros topics with this

    # TODO: make this a service call
    def set_pplot(self, it):
        self.it_plot = it
        self.it_count = 0
        if self.plot == 0:
            self.disp = plotter_offline(self.sys[0].map)

    def run(self):

        # TODO fill this with topics
        x_pred = []
        end = False
        error = False
        # End TODO fill this with topics

        if self.plot == 1 :

            for idx, agent in enumerate(self.sys):
                self.disp.plot_step(agent.states[1, 7], agent.states[1, 8], agent.states[1, 5], idx)

        elif self.plot == 2:
            self.disp.animate_step(self.sys[0].states[1, 7]) # TODO: Fix this

        if (self.plot == -1 and end ) or error:

            input("Press enter to save track ...")

            for j,r in enumerate(self.sys):
                self.disp.plot_offline_experiment(r, self.path, self.color[j])

        if (self.it_count == self.it_plot):
            self.it_count = 0
            for j,r in enumerate(self.sys):
                self.disp.plot_offline_experiment(r, None, self.color[j],True)

        if self.plot == 2 and end:
            self.disp.animate_offline_experiment(self.sys, style_agents=self.color)











# ===================================================================================================================================== #
# ============================================================= Internal Functions ==================================================== #
# ===================================================================================================================================== #




d
