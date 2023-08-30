#!/usr/bin/env python3.6

# TODO: https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/?answer=358386#post-id-358386
import numpy as np
np.set_printoptions(formatter={'float': lambda x: "{0:0.1f}".format(x)})
import sys

# TODO: Online plotting was not tested

# ROS
from planner_experiments.msg import agent_info
from std_msgs.msg import Int32, Bool
from utilities_ROS.utilities_ros import deserialise_np
from plan_lib.mapManager import Map
from plan_lib.plotter import plotter_offline,plotter

import rclpy
from rclpy.node import Node
import threading
# proxy class used to match function calls
class proxy_agent():
    def __init__(self):
        self.states = []

    # change this so that the proxy is updated thorough ROS topics

class plotter_ROS(Node):

    def __init__(self, mode = "LPV"):

        #  Load settings
        if mode == "LPV":
            from config_files.config_LPV import settings

        else:
            from config_files.config_NL import settings

        super().__init__("plotter")
        self.rate = self.create_rate(1000, self.get_clock())

        self.map = Map(settings["map_type"])
        self.plot = settings["plot"]

        try:
            self.n_agent = int(self.get_parameter("n_robots").get_parameter_value())
        except:
            self.n_agent = settings["n_agent"]

        self.color = settings["color_list"]
        self.path = settings["path_img"]
        self.agents = []
        self.end = False
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

        self.sub_pp = self.create_subscriber("set_pplot",Int32 , self.set_pplot_callback)
        self.sub_ros_kill = self.create_subscriber("end_signal", Bool, self.callback_end, queue_size=10)

        for n in range(0,self.n_agent):
            self.subs[n] = self.create_subscriber('car' + str(n) + "_data", agent_info, self.callback, (n))

    def callback(self,msg,id):
        self.agents[id].states.append(deserialise_np(msg)[0][1,:])

    def callback_end(self,msg):
        self.end = msg.data

    def set_pplot_callback(self, msg):
        self.it_plot = msg.data
        self.it_count = 0
        if self.plot == 0:
            self.disp = plotter_offline(self.map)

    def run(self):
        while not rclpy.ok() and not self.end:

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

            self.it_count += 1
            self.rate.sleep()

        if self.plot == 2:
            self.disp.animate_offline_experiment(self.agents, style_agents=self.color)

        if (self.plot == -1 ):

            for j,r in enumerate(self.agents):
                print(self.path)
                self.disp.plot_offline_experiment(r, self.path, self.color[j])

        raise SystemExit


if __name__ == "__main__":
    rclpy.init()
    try:
        plotter = plotter_ROS(sys.argv[1])
    except:
        plotter = plotter_ROS()

    thread = threading.Thread(target=rclpy.spin, args=(plotter,), daemon=True)
    thread.start()

    try:
        plotter.run()
    except:
        rclpy.logging.get_logger("Quitting").info('Done')

    plotter.destroy_node()
    rclpy.shutdown()



