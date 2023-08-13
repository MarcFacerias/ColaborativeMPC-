#!/usr/bin/env python3.6

import numpy as np
np.set_printoptions(formatter={'float': lambda x: "{0:0.1f}".format(x)})
import sys

# TODO: Online plotting was not tested

# ROS
from planner_experiments.msg import agent_info
from std_msgs.msg import Int32
from utilities_ROS.utilities_ros import deserialise_np
from plan_lib.mapManager import Map
from plan_lib.plotter import plotter_offline,plotter

import rospy
# proxy class used to match function calls
class proxy_agent():
    def __init__(self):
        self.states = []

    # change this so that the proxy is updated thorough ROS topics

class plotter_ROS():

    def __init__(self, mode = "LPV"):

        if mode == "LPV":
            from config_files.config_LPV import settings

        else:
            from config_files.config_NL import settings

        self.nh = rospy.init_node("plotter")
        self.rate = rospy.Rate(1000)

        self.map = Map(settings["map_type"])
        self.plot = settings["plot"]
        try:
            self.n_agent = int(rospy.get_param("n_robots"))
        except:
            self.n_agent = settings["n_agent"]

        self.color = settings["color_list"]
        self.path = settings["path_img"]
        self.agents = []

        self.subs = [''] * self.n_agent

        for i in range(0,self.n_agent):
            placeholder = proxy_agent()
            self.agents.append(placeholder)

        if self.plot == 1:
            self.disp = plotter(self.map, self.n_agent)

        elif self.plot != 0:
            self.disp = plotter_offline(self.map)

        self.it_plot = -1
        self.it_count = 0

        self.sub_pp = rospy.Subscriber("set_pplot",Int32 , self.set_pplot_callback)

        for n in range(0,self.n_agent):
            self.subs[n] = rospy.Subscriber('car' + str(n) + "_data", agent_info, self.callback, (n))

    def callback(self,msg,id):
        self.agents[id].states.append(deserialise_np(msg)[0][1,:])

    def set_pplot_callback(self, msg):
        self.it_plot = msg.data
        self.it_count = 0
        if self.plot == 0:
            self.disp = plotter_offline(self.map)

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

            for j,r in enumerate(self.agents):
                self.disp.plot_offline_experiment(r, self.path, self.color[j])

        self.it_count +=1


if __name__ == "__main__":
    myargv = rospy.myargv(argv=sys.argv)
    try:
        mode = myargv[1]
        plotter = plotter_ROS(mode)
    except:
        plotter = plotter_ROS()

    plotter.run()



