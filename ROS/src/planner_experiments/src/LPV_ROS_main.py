#!/usr/bin/env python3.6

# ROS libs
import rospy
import sys
from utilities_ROS.utilities_ros import serialise_np, deserialise_msg
from planner_experiments.msg import agent_info

# Global Variables
import numpy as np
import warnings
import time
from plan_lib.distributedPlanner import PlannerLPV
from plan_lib.mapManager import Map
from plan_lib.utilities import checkEnd, initialise_agents
from IOmodule_ROS import io_class_ROS
from plan_lib.config.LPV import initialiserLPV, settings
from plan_lib.config import x0_database

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


class agentROS_LPV(initialiserLPV):
    # Agents class, interfaces with the planner, saves data etc
    #  Q: [vx ; vy ; psiDot ; e_psi ; s ; e_y]
    #  R:  [delta ; a] la R es sobre el dU
    def __init__(self, settings, x0, id, connections):
        super().__init__(self, settings) # initialise the initialiser
        self.map = Map(settings["map_type"])
        self.N = settings["N"]
        self.dt = settings["dt"]
        self.Controller = PlannerLPV(self.Q, self.Qs, self.R, self.dR, self.N, self.dt, self.map, id, self.wq, self.model_param, self.sys_lim)
        self.x0 = x0
        self.states = []
        self.u = []
        self.planes = []
        self.time_op = []
        self.status = []
        self.id = id
        self.pub = rospy.Publisher('car' + str(id) + "_data", agent_info, queue_size=10)
        self.subs = [''] * len(connections)
        self.agents_id = connections
        self.agents = [''] * (len(connections)+1)

        for i,n in enumerate(connections):
            self.subs[i] = rospy.Subscriber('car' + str(n) + "_data", agent_info, self.callback,n)

    def one_step(self, uPred = None, xPred = None):

        self.pose = self.x0[[7,8]]
        tic = time.time()
        feas, raw, planes = self.Controller.solve(self.x0, xPred, uPred, self.agents, self.agents_id, self.pose)
        self.time_op.append(time.time() - tic)
        if not feas:
            return feas,uPred, xPred, planes, raw

        if (self.Controller.sPred[:,1] >= 0.1).any():
            msg = "WARNING slack violated !"
            warnings.warn(msg)
            print(self.Controller.sPred[:,1:])

        uPred, xPred = self.Controller.uPred, self.Controller.xPred
        self.save(xPred, uPred, feas, planes)
        self.send_states(xPred)
        return feas,uPred, xPred, planes, raw

    def callback(self,msg,id):
        self.agents[id] = deserialise_msg(msg)

    def send_states(self):
        msg = serialise_np(self.Controller.xPred[1::,[7,8]])
        self.pub.publish(msg)


def main(id):

    #########################################################
    #########################################################

    # Map settings
    n_agents = settings["n_agents"]
    N        = settings["N"]
    dt       = settings["dt"]
    max_it   = settings["max_it"]

    # set constants
    x_pred = [None] * n_agents
    u_pred = [None] * n_agents

    it = 0

    x0 = x0_database[0:n_agents]
    ns = [j for j in range(0, n_agents)].remove(id)

    maps = [Map(settings["map_type"])]*n_agents

    agents,x_old,u_old = initialise_agents(x0,N,dt,maps)

    rospy.init_node("car" + str(id))
    rate = rospy.Rate()  # 10hz

    rs = agentROS_LPV(settings, x_old[id][0,:], id, ns)

    io = io_class_ROS(settings, rs)

    while(it<max_it and not checkEnd(x_pred, maps) and not rospy.is_shutdown()):

        io.tic()
        feas, u_pred, x_pred, planes, raws = rs.one_step(u_old, x_old)

        if not feas:
            break

        rs.x0 = x_pred[1, :]

        io.toc()

        u_old = u_pred
        x_old = x_pred[1:, :]

        io.update( x_pred, u_pred ,agents, it)
        it += 1
        rate.sleep()

    io.update(x_pred, u_pred, agents, it, end=True)

if __name__ == "__main__":
    myargv = rospy.myargv(argv=sys.argv)
    main(myargv[1])



