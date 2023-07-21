
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import os

sys.path.append(sys.path[0]+'/NonLinDistribPlanner')
sys.path.append(sys.path[0]+'/Utilities')
sys.path.append(sys.path[0]+'/plotter')
sys.path.append(sys.path[0]+'/Config/NL_EU')

from NL_Planner_Eu import NL_Planner_EU
from trackInitialization import Map, wrap
from plot_vehicle import *
from utilities import checkEnd
from config import *  #Important!! Containts system definitions

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class agent(initialiserNL_EU):

    def __init__(self, N, Map, dt, x0, id, dth):
        super().__init__()  # initialise the initialiser
        self.map = Map
        self.dt = dt
        self.N = N
        self.x0 = x0
        self.Controller = NL_Planner_EU(self.Q, self.R, N, dt, Map, id, dth)
        self.states = []
        self.u = []
        self.time_op = []
        self.status = []
        self.data_opti = []
        self.data_collec = []
        self.id = id

    def one_step(self, lambdas, agents, agents_id, uPred = None, xPred = None):

        tic = time.time()
        feas, Solution, planes, slack, self.data_opti = self.Controller.solve(self.x0, xPred, uPred, lambdas, agents, agents_id, self.data_collec)
        self.time_op.append(time.time() - tic)
        self.status.append(feas)

        return feas, self.Controller.uPred, self.Controller.xPred, planes, slack, Solution

    def plot_experiment(self):

        disp = plotter_offline(self.map)
        disp.add_agent_ti(self)
        disp.add_planes_ti(self)

    def save(self, xPred, uPred):

        self.states.append(xPred[0,:])
        self.u.append(uPred[0,:])

    def save_to_csv(self):

        path = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/NonLinDistribPlanner/TestsPaperNLcs/" + str(self.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path+'/states.dat', self.states, fmt='%.5e',delimiter=' ')
        np.savetxt(path + '/u.dat', self.u, fmt='%.5e', delimiter=' ')
        np.savetxt(path + '/time.dat', self.time_op, fmt='%.5e', delimiter=' ')

    def save_var_to_csv(self,var, name):

        path = "/NonLinDistribPlanner/TestsPaperNLss/"

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path + '/' + str(name) + '.dat', var, fmt='%.5e',delimiter=' ')

def initialise_agents(data,Hp,dt,map, accel_rate=0):
    agents = np.zeros((Hp+1,len(data),2))
    data_holder = [i for i in range(len(data))]
    u_old  = [None] * n_agents
    x_old  = [None] * n_agents

    for id, el in enumerate(data):

        aux = predicted_vectors_generation_V2(Hp, el, dt, map[id], accel_rate)
        agents[:,id,:] = aux[0][:,-2:] # without slack
        data_holder[id] = [aux[0].flatten(),aux[1].flatten(),np.zeros((Hp,4)),np.zeros((Hp,len(data)))] # we need to initialise the slack vars
        x_old[id] = aux[0]
        u_old[id] = aux[1]
    return agents, data_holder, x_old,u_old

def predicted_vectors_generation_V2(Hp, x0, dt, map, accel_rate = 0):
    # We need a prediction of the states for the start-up proces of the controller (To instantiate the LPV variables)
    # [vx vy psidot y_e thetae theta s x y ]

    Vx      = np.zeros((Hp+1, 1))
    Vx[0]   = x0[0]
    S       = np.zeros((Hp+1, 1))
    S[0]    = x0[6]
    Vy      = np.zeros((Hp+1, 1))
    Vy[0]   = x0[1]
    W       = np.zeros((Hp+1, 1))
    W[0]    = x0[2]
    Ey      = np.zeros((Hp+1, 1))
    Ey[0]   = x0[3]
    Epsi    = np.zeros((Hp+1, 1))
    Epsi[0] = x0[4]

    aux = map.getGlobalPosition(S[0], Ey[0])
    Theta = np.zeros((Hp+1, 1))
    Theta[0] = aux[2]
    X = np.zeros((Hp+1, 1))
    X[0] = aux[0]
    Y = np.zeros((Hp+1, 1))
    Y[0] = aux[1]

    Accel   = 1.0

    for i in range(0, Hp):
        Vy[i+1]      = x0[1]
        W[i+1]       = x0[2]
        Ey[i+1]      = x0[3]
        Epsi[i+1]    = x0[4]

    Accel   = Accel + np.array([ (accel_rate * i) for i in range(0, Hp)])

    for i in range(0, Hp):
        Vx[i+1]    = Vx[i] + Accel[i] * dt
        S[i+1]      = S[i] + Vx[i] * dt
        X[i+1], Y[i+1], Theta[i+1] = map.getGlobalPosition(S[i], Ey[i])

    xx  = np.hstack([ Vx, Vy, W,Ey, Epsi, Theta ,S ,X,Y]) # [vx vy psidot y_e thetae theta s x y slack1 slack2]
    uu = np.zeros(( Hp, 2 ))
    return xx, uu

def eval_constraint(x1, x2, D):

    cost1 = D - np.sqrt(sum((x1-x2)**2)) # the OCD update depends on on the diference between the minimum D and the euclidean dist

    return np.array(cost1)

def main():

#########################################################
#########################################################

    # controller constants
    finished = False
    finished_ph = False
    time_OCD = []

    # set constants
    x0 = x0_database[0:n_agents]
    ns = [[i for i in range(0, n_agents)] for j in range(0, n_agents)]

    for j, n in enumerate(ns):
        n.remove(j)

    # initialise data structures
    maps = [Map(map_type)]*n_agents
    agents,data,x_old,u_old = initialise_agents(x0,N,dt,maps)

    x_pred = [None] * n_agents
    u_pred = [None] * n_agents
    feas   = [None] * n_agents
    raws   = [None] * n_agents
    rs     = [None] * n_agents
    lsack  = [None] * n_agents
    planes = [None] * n_agents

    if plot:
        disp = plotter(maps[0],n_agents)

    if plot_end:
        d = plotter_offline(maps[0])

    # initialise controllers
    for i in range (0,n_agents):
        rs[i] = agent(N, maps[i], dt, x_old[i], i, dth)
        rs[i].data_collec = [data[j] for j in ns[i]]

    cost_old = np.zeros((n_agents, n_agents, N))
    lambdas_hist = []
    cost_hist = []
    it = 0

    while(it<max_it and not checkEnd(x_pred, maps)):

        tic = time.time()
        lambdas = np.zeros((n_agents, n_agents, N))
        it_OCD = 0
        itc = 0

        while(not (it_OCD > 2 and finished)) :
            # OCD loop, we want to force at least 2 iterations + it_conv iterations without significant changes
            # run an instance of the optimisation problems

            for i, r in enumerate(rs):
                feas[i], u_pred[i], x_pred[i], planes[i], lsack[i], raws[i] = r.one_step( lambdas[[i],ns[i],:], agents[:,ns[i],:], ns[i], u_old[i], raws[i])

            for j,r in enumerate(rs):
                r.data_collec = [rs[i].data_opti for i in ns[j]]

            cost = np.zeros((n_agents,n_agents,N))

            # update the values of x,y for the obstacle avoidance constraints
            agents = np.swapaxes(np.asarray(x_pred)[:, :, -2:], 0, 1)

            for k in range(1,N+1):
                for i in range(0,n_agents):
                    for j in range(0,n_agents):

                        if (i != j) and i<j:
                            cost[i,j,k-1] = eval_constraint(agents[k,i,:],agents[k,j,:],dth)

            alpha = get_alpha()
            lambdas += alpha*cost
            lambdas_hist.append(lambdas)

            # check if the values of x changed, if they are close enough for two iterations the algorithm has converged
            if it_OCD != 0:
                finished_ph = 1
                for i in range(0,n_agents):
                    finished_ph &= np.allclose(x_old[i], x_pred[i], atol=0.01)

                finished_ph &= np.allclose(cost, cost_old, atol=0.01)
                itc += 1

            # store values from current iteration into the following one
            x_old = x_pred
            cost_old = cost

            if not finished_ph :
                itc = 0

            elif itc > it_conv:
                finished = True
                print("Iteration finished with " + str(it_OCD) + " steps")

            if it_OCD > max_it_OCD:
                print("max it reached")
                finished = True

            it_OCD += 1

            if verb_OCD:

                print("-------------------------------------------------")
                print("length " + str(it_OCD))
                for i in range(0, n_agents):
                    print("---------------------Agents---------------------------------------")
                    print("Agent " + str(i) + " track s: " + x_pred[i][1,:])
                print("-------------------------------------------------")

        for j,r in enumerate(rs):
            r.save(x_pred[j], u_pred[j])
            r.x0 = x_pred[j][1:, :]

        u_old = u_pred
        toc = time.time()
        finished = False
        time_OCD.append((toc - tic)/n_agents)
        it += 1

        if plot :

            for idx in range(0, n_agents):
                disp.plot_step(x_pred[idx][1, 7], x_pred[idx][1, 8], x_pred[0][1, 5], idx)

        if verb :

            print("--------------------------------------------------------------")
            print("it: " + str(it))
            print("length " + str(it_OCD))
            print(time.time() - tic)
            print("agents x : " + str(agents[0,:,0]))
            print("agents y : " + str(agents[0,:,1]))

            for i in range(0,n_agents):
                print("---------------------Agents---------------------------------------")
                print("Agent " + str(i) + " track s: " + str(x_pred[i][0,-3]) + "/" + str(maps[i].TrackLength[0]))
                print("Agent " + str(i) + " u0: " + str(u_pred[i][0,0]) + " u1: " + str(u_pred[i][0,1]))

            print("---------------------END Agents---------------------------------------")
            print("avg computational time: " + str((toc-tic)/n_agents))
            print("--------------------------------------------------------------")

    if plot_end:

        for j,r in enumerate(rs):
            d.plot_offline_experiment(r, color_list[j])
            r.save_to_csv()

        r[0].save_var_to_csv(time_OCD, "time_OCD")
        r[0].save_var_to_csv(cost_hist, "cost_hist2")
        input("Press enter to continue...")

def plot_performance( agent):

    fig_status = plt.figure()
    fig_status.add_subplot(2, 1, 1)
    x = np.arange(0,len(agent.status))
    plt.scatter(x, np.array(agent.status))
    fig_status.add_subplot(2, 1, 2)
    plt.scatter(x, np.array(agent.time_op))
    plt.show()
    plt.pause(0.001)

if __name__ == "__main__":

    main()




