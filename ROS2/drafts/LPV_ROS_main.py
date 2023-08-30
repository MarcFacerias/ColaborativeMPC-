#!/usr/bin/env python3.6

# ROS libs
import rclpy
from rclpy.node import Node
import sys
from utilities_ROS.utilities_ros import serialise_np, deserialise_np
from planner_experiments.msg import agent_info
import threading

# Global Variables
import numpy as np
import warnings
import time
from plan_lib.distributedPlanner import PlannerLPV
from plan_lib.mapManager import Map
from plan_lib.utilities import checkEnd, initialise_agents
from IOmodule_ROS.IOmodule import io_class_ROS
from plan_lib.config.LPV import initialiserLPV
from plan_lib.config import x0_database
from config_files.config_LPV import settings
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


class agentROS_LPV(initialiserLPV):
    # Agents class, interfaces with the planner, saves data etc
    #  Q: [vx ; vy ; psiDot ; e_psi ; s ; e_y]
    #  R:  [delta ; a] la R es sobre el dU

    def __init__(self, settings, x0, id, connections, node):
        super().__init__(self, settings) # initialise the initialiser
        self.map = Map(settings["map_type"])
        self.N = settings["N"]
        self.dt = settings["dt"]
        self.Controller = PlannerLPV(self.Q, self.Qs, self.R, self.dR, self.N, self.dt, self.map, id, self.wq, self.model_param, self.sys_lim)
        self.x0 = x0[0,:] # inititial values of x
        self.states = []
        self.u = []
        self.planes = []
        self.time_op = []
        self.status = []
        self.id = id
        self.pub = node.create_publisher('car' + str(id) + "_data", agent_info, queue_size=10)
        self.subs = [''] * (len(connections)+1) # subscribers, there's an additional field to make easier the data acces (id matches aray position)
        self.agents_id = connections
        self.agents = np.zeros((settings["N"]+1, len(connections)+1, 2))
        self.updated = [True] * (len(connections)+1) # flag to wait till all agents send their data
        self.waiting = False

        for n in connections:
            self.subs[n] = node.create_subscriber('car' + str(n) + "_data", agent_info, self.callback,n)

    def one_step(self, uPred = None, xPred = None):

        pose = xPred[:,[7,8]] # current position of the vehicle
        tic = time.time()
        feas, raw, planes = self.Controller.solve(self.x0, xPred, uPred, self.agents[:,self.agents_id,:], self.agents_id, pose)
        self.time_op.append(time.time() - tic)
        if not feas:
            return feas,uPred, xPred, planes, raw

        if (self.Controller.sPred[:,1] >= 0.1).any():
            msg = "WARNING slack violated !"
            warnings.warn(msg)
            print(self.Controller.sPred[:,1:])

        uPred, xPred = self.Controller.uPred, self.Controller.xPred
        self.agents[:, self.id, :] = xPred[:,[7,8]]
        self.save(xPred, uPred, feas, planes)
        self.send_states()
        return feas,uPred, xPred, planes, raw

    def callback(self,msg,id):
        self.agents[:,id,:] = deserialise_np(msg)[0][:,[7,8]]
        self.updated[id] = True

    def send_states(self):
        msg = serialise_np([self.Controller.xPred])
        self.pub.publish(msg)

    def wait_coms(self):
        self.waiting = True
        for idx in self.agents_id:
            self.updated[idx] = False

class car(Node):

    def __init__(self, id):
        super().__init__("car" + str(id))
        #########################################################
        #########################################################
        # Update number of robots
        try:
            settings["n_agents"] = int( self.get_parameter('n_robots').get_parameter_value())
        except:
            pass

        # Map settings
        n_agents = settings["n_agents"]
        N = settings["N"]
        dt = settings["dt"]
        id = int(id)

        # set constants
        self.x_pred = [None] * n_agents
        self.u_pred = [None] * n_agents
        self.max_it = settings["max_it"]

        # right now all agents simulated are neighbours
        x0 = x0_database[0:n_agents]
        ns = [j for j in range(0, n_agents)]
        ns.remove(id)

        maps = [Map(settings["map_type"])] * n_agents
        agents, x_old, u_old = initialise_agents(x0, N, dt, maps)

        self.x_old = x_old[id]
        self.u_old = u_old[id]

        self.rs = agentROS_LPV(settings, self.x_old, id, ns)
        self.rs.agents = agents

        self. io = io_class_ROS(settings, self.rs)

        self.rate = self.create_rate(int(1/float(dt)), self.get_clock())
        # self.main(x_old, u_old)

    def main(self):
        it = 0
        while(it<self.max_it and not checkEnd(self.x_pred, self.rs.map) and rclpy.ok()):

            if not all(self.rs.updated):
                continue

            elif self.rs.waiting:
                self.io.toc()
                self.rs.waiting = False

            if not self.rs.waiting:
                self.io.tic()

            feas, u_pred, x_pred, planes, raws = self.rs.one_step(self.u_old, self.x_old)

            if not feas:
                break

            self.rs.x0 = x_pred[1, :]

            self.u_old = u_pred
            self.x_old = x_pred

            self.io.update( x_pred, u_pred ,self.agents, it)
            it += 1

            self.rs.wait_coms()
            self.rate.sleep()

        self.io.update(x_pred, u_pred, self.agents, self.it, end=True)
        raise SystemExit

def run(id):
    rclpy.init()
    node = car(id)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        node.main()
    except:
        rclpy.logging.get_logger("Quitting").info('Done')

    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":

    run(sys.argv[1])



