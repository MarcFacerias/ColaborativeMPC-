#!/usr/bin/env python3.6

# ROS libs
import rospy
import sys
import numpy as np
import time

from utilities_ROS.utilities_ros import serialise_np, deserialise_np
from planner_experiments.msg import agent_info
from IOmodule_ROS.IOmodule import io_class_ROS

from plan_lib.nonLinDistribPlanner import PlannerEu
from plan_lib.mapManager import Map
from plan_lib.utilities import checkEnd, initialise_agents, get_lambdas
from plan_lib.config.NL import initialiserNL, eval_constraintEU, get_alpha
from plan_lib.config import x0_database
from config_files.config_NL import settings

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class agentROS_OCD(initialiserNL):

    def __init__(self, settings, x0, id, connections):
        super().__init__(self, settings) # initialise the initialiser
        self.map = Map(settings["map_type"])
        self.dt = settings["dt"]
        self.N =  settings["N"]
        self.x0 = x0
        self.Controller = PlannerEu(self.Q,self.Qs, self.R, self.dR, self.N, self.dt, self.map, id, self.model_param, self.sys_lim)
        self.states = []
        self.u = []
        self.time_op = []
        self.status = []
        self.data_opti = []
        self.data_collec = []
        self.id = id

        # ROS CONNECTIONS
        self.pub = rospy.Publisher('car' + str(id) + "_data", agent_info, queue_size=10)
        self.subs = [''] * (len(connections)+1)
        self.agents_id = connections
        self.updated = [True] * (len(connections)+1)
        self.waiting = False
        self.agents_data = []
        for i,n in enumerate(connections):
            self.subs[i] = rospy.Subscriber('car' + str(n) + "_data", agent_info, self.callback, (n))


    def one_step(self, lambdas, uPred = None, raws = None):
        tic = time.time()
        agents = self.build_agents()
        feas, Solution, self.data_opti = self.Controller.solve(self.x0, raws, uPred, lambdas,agents , self.agents_id, self.agents_data[self.agents_id][1::])
        self.time_op.append(time.time() - tic)
        self.send_states()
        return feas, self.Controller.uPred, self.Controller.xPred, Solution

    def callback(self,msg,id):
        self.agents_data[id] = deserialise_np(msg)
        self.updated[id] = True

    def send_states(self):
        msg = serialise_np([self.Controller.xPred] + self.data_opti)
        self.pub.publish(msg)

    def wait_update(self):
        self.waiting = True
        self.updated = [not elem for elem in self.updated]

    def build_agents(self):
        placeholder = np.empty((self.N, len(self.agents_id)+1,2))
        aux = np.asarray(self.agents_data[:][0])
        return np.swapaxes(np.asarray(aux)[:, :, -2:], 0, 1)

def main(id):
# TODO Note we only use Neigbours!! not all robots from the fleet
#########################################################
#########################################################

    id = int(id)
    # Map settings
    try:
        settings["n_agents"] = int(rospy.get_param("n_robots"))
    except:
        pass

    n_agents = settings["n_agents"]
    map_type = settings["map_type"]
    N = settings["N"]
    dt = settings["dt"]
    max_it = settings["max_it"]
    dth = settings["min_dist"]
    max_it_OCD = settings["max_it_OCD"]
    min_it_OCD = settings["min_it_OCD"]
    it_conv = settings["it_conv"]

    # controller constants
    finished = False
    finished_ph = False

    # set constants
    x0 = x0_database[0:n_agents]
    ns = [j for j in range(0, n_agents)]
    ns.remove(id)

    # initialise data structures
    maps = [Map(map_type)]*n_agents
    agents,x_old,u_old = initialise_agents(x0,N,dt,maps)

    data = []
    for i in range(0,n_agents):
        data.append([x_old[id], u_old[id], np.zeros((N, 2)), np.zeros((N, 2)), np.zeros((N, n_agents)),
                np.zeros((N, n_agents))])

    rs = agentROS_OCD(settings, x_old[id][0,:],id, ns)
    rs.data_collec = [data[j] for j in ns]
    rs.agents_data = [[x_old[i]] + data[i] for i in range(0,n_agents)]
    u_old = u_old[id]
    x_pred = x_old[id]
    lambdas_hist = []
    it = 0
    lambdas = get_lambdas(settings)[id, ns, :].squeeze()
    error = False
    raws = None

    io = io_class_ROS(settings, rs)
    rospy.init_node("car" + str(id))
    rate = rospy.Rate(1000)

    while(it<max_it and not checkEnd(x_pred, maps)):

        it_OCD = 0
        itc = 0
        io.tic()

        while(not (it_OCD > min_it_OCD and finished)) :
            # OCD loop, we want to force at least 2 iterations + it_conv iterations without significant changes
            # run an instance of the optimisation problems

            if not all(rs.updated):
                continue

            elif rs.waiting:
                rs.waiting = False

            feas, u_pred, x_pred, raws = rs.one_step( lambdas, u_old, raws)

            if not feas:
                error = True
                break

            io.toc()

            cost = np.zeros((n_agents,N))
            # update the values of x,y for the obstacle avoidance constraits

            if n_agents == 1:
                break

            rs.wait_update()

            agents = rs.build_agents()

            for k in range(1,N+1):
                    for j in range(0,n_agents):
                        if (id != j) and id<j:
                            cost[j,k-1] = eval_constraintEU(agents[k,id,:],agents[k,j,:],dth)

            alpha = get_alpha()
            lambdas += alpha*cost
            lambdas_hist.append(lambdas)

            # check if the values of x changed, if they are close enough for two iterations the algorithm has converged
            metric = ['']*(n_agents+1)
            if it_OCD != 0:
                finished_ph = 1
                for i in range(0,n_agents):
                    finished_ph &= np.allclose(x_old[i], x_pred[i], atol=0.01)
                    metric[i] = x_old[i] - x_pred[i]

                itc += 1

            if not finished_ph :
                itc = 0

            elif itc > it_conv:
                finished = True
                print("Iteration finished with " + str(it_OCD) + " steps")

            if it_OCD > max_it_OCD:
                print("max it reached")
                finished = True


            io.updateOCD(x_pred, it_OCD, it)
            it_OCD += 1

        rs.save(x_pred, u_pred)
        rs.x0 = x_pred[1:, :]

        io.toc()
        u_old = u_pred
        finished = False
        io.update( x_pred, u_pred ,agents, it, error = error, OCD_ct=it_OCD)
        it += 1

        if error:
            break
        rate.sleep()

    io.update(x_pred, u_pred, agents, it, end=True,error = error)

if __name__ == "__main__":
    myargv = rospy.myargv(argv=sys.argv)
    main(myargv[1])



