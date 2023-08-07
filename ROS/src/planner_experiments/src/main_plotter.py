#!/usr/bin/env python

from planner.packages.plotter import *
import numpy as np
np.set_printoptions(formatter={'float': lambda x: "{0:0.1f}".format(x)})
import sys

# ROS
from planner_experiments.msg import agent_info
from std_msgs.msg import Int32
from utilities_ROS.utilities_ros import deserialise_msg
from planner.packages.mapManager import Map
import rospy
# proxy class used to match function calls
class proxy_agent():
    def __init__(self):
        self.states = []

    # change this so that the proxy is updated thorough ROS topics

class plotter_ROS():

    def __init__(self, mode = "LPV"):

        if mode == "LPV":
            from planner.packages.config.LPV import settings

        else:
            from planner.packages.config.NL import settings

        self.nh = rospy.init_node("plotter")
        self.rate = rospy.Rate(settings["Hz"])

        map = Map(settings["map"])
        self.plot = settings["plot"]
        self.n_agent = settings["n_agents"]
        self.color = settings["color_list"]
        self.path = settings["path_img"]
        self.sys = proxy_agent() # change this so that the proxy is updated

        self.subs = [''] * len(self.n_agent)
        self.agents = [proxy_agent()] * len(self.n_agent)

        if self.plot == 1:
            self.disp = plotter(map, self.n_agent)

        elif self.plot != 0:
            self.disp = plotter_offline(map)

        self.sub_pp = rospy.Subscriber("set_pplot",Int32 , self.set_pplot_callback)

        for n in range(self.n_agent):
            self.subs[n] = rospy.Subscriber('car' + str(n) + "_data", agent_info, self.callback(id=n))

    def callback(self,msg,id):
        self.agents[id].states = deserialise_msg(msg)

    def set_pplot_callback(self, msg):
        self.it_plot = msg.data
        self.it_count = 0
        if self.plot == 0:
            self.disp = plotter_offline(self.sys[0].map)

    def run(self):
        while not rospy.is_shutdown():

            if self.plot == 1 :

                for idx, agent in enumerate(self.agents):
                    self.disp.plot_step(agent.states[1, 7], agent.states[1, 8], agent.states[1, 5], idx)

            elif self.plot == 2:
                for j,r in enumerate(self.agents):
                    self.disp.animate_step(r,self.color[j])

            if (self.it_count == self.it_plot):
                self.it_count = 0
                for j,r in enumerate(self.agents):
                    self.disp.plot_offline_experiment(r, None, self.color[j],True)

            self.rate.sleep()

        if self.plot == 2:
            self.disp.animate_offline_experiment(self.agents, style_agents=self.color)

        if (self.plot == -1 ):

            input("Press enter to save track ...")

            for j,r in enumerate(self.agents):
                self.disp.plot_offline_experiment(r, self.path, self.color[j])


if __name__ == "__main__":
    myargv = rospy.myargv(argv=sys.argv)
    try:
        mode = myargv[1]
    except:
        mode = None # TODO add error handling here 

    plotter = plotter_ROS(mode)
    plotter.run()



