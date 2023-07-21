
import sys
import numpy as np
import matplotlib.pyplot as plt
import time
import os

sys.path.append(sys.path[0]+'/NonLinearControllerObject')
sys.path.append(sys.path[0]+'/Utilities')
sys.path.append(sys.path[0]+'/plotter')
sys.path.append(sys.path[0]+'/DistributedPlanner')

from PathFollowingCASADI_param_LPVds_NOROS import PathFollowingNL_MPC
from trackInitialization import Map, wrap
from plot_vehicle import *

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# TODO: implement this suggestion https://groups.google.com/g/casadi-users/c/1B2kTOF--SI
# TODO: Add quality of life changes to the planes
plot = False
plot_end = True
it_conv = 1
n_agents = 4

def compute_hyper(x_ego,x_neg):

    a = x_neg - x_ego
    b = 0.5 * a @ (x_ego + x_neg).T

    return a,b

class agent():

    #TODO: clean redundant variables
    def __init__(self, N, Map, dt, x0, id, dth, Q=np.diag([120.0, 1.0, 1.0, 1500.0, 70.0, 0.0, 0.0,0,0,0]), R=10* np.diag([1, 1])  ):
        self.map = Map
        self.N = N
        self.dt = dt
        self.Q  = Q   #[vx ; vy ; psiDot ; e_psi ; s ; e_y]
        self.R  = R   #[delta ; a]
        self.Controller = PathFollowingNL_MPC(self.Q, self.R, N, dt, Map, id, dth)
        self.x0 = x0
        self.states = []
        self.u = []
        self.planes = []
        self.output_opti = []
        self.time_op = []
        self.status = []
        self.slack = []
        self.data_opti = []
        self.data_share = []
        self.id = id

    # TODO: clean redundant functions
    def one_step(self,x0, lambdas, agents, agents_id, uPred = None, xPred = None):

        if (xPred is None):
            xPred, uPred = predicted_vectors_generation_V2(self.N, np.array(self.x0), self.dt, self.map)

        if x0 is None:
            x0 = xPred

        tic = time.time()
        feas, Solution, planes, slack, self.data_opti = self.Controller.solve(x0, xPred, uPred, lambdas, agents, agents_id, self.data_share)
        self.time_op.append(time.time() - tic)
        self.status.append(feas)

        return feas, self.Controller.uPred, self.Controller.xPred, planes, slack, Solution



    def plot_experiment(self):

        disp = plotter_offline(self.map)
        disp.add_agent_ti(self)
        disp.add_planes_ti(self)

    def save(self, xPred, uPred, planes):
        # TODO add the functionality to save the planes somewhere for the future problems
        self.states.append(xPred[0,:])
        self.u.append(uPred[0,:])

    def save_to_csv(self):

        path = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/NonLinearControllerObject/TestsPaperNLcs/" + str(self.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path+'/states.dat', self.states, fmt='%.5e',delimiter=' ')
        np.savetxt(path + '/u.dat', self.u, fmt='%.5e', delimiter=' ')
        np.savetxt(path + '/time.dat', self.time_op, fmt='%.5e', delimiter=' ')

    def save_var_to_csv(self,var, name):

        path = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/experiments/test-bench/catkin_mrs/src/colab_mpc/src/NonLinearControllerObject/TestsPaperNLss/"

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path + '/' + str(name) + '.dat', var, fmt='%.5e',delimiter=' ')

def initialise_agents(data,Hp,dt,map, accel_rate=0):
    agents = np.zeros((Hp+1,len(data),2))
    data_holder = [i for i in range(len(data))]
    for id, el in enumerate(data):

        aux = predicted_vectors_generation_V2(Hp, el, dt, map[id], accel_rate)
        agents[:,id,:] = aux[0][:,-2:] # without slack
        data_holder[id] = [aux[0].flatten(),aux[1].flatten(),np.zeros((Hp,4)),np.zeros((Hp,len(data)))] # we need to initialise the slack vars
    return agents, data_holder

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
    N = 10
    dt = 0.01
    alpha = 0.25
    max_it = 500
    finished = False
    finished_ph = False
    dth = 0.25
    time_OCD = []

    # define neighbours
    n_0 = [1,2,3]
    n_1 = [0,2,3]
    n_2 = [0,1,3]
    n_3 = [0,1,2]

    # define initial positions
    x0_0 = [2.5, -0.16, 0.00, 0.55, 0, 0.0, 0, 0.0, 1.5]  # [vx vy psidot y_e thetae theta s x y]
    x0_1 = [2.5, -0.16, 0.00,-0.55, 0, 0.0, 0.25, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]
    x0_2 = [2.5, -0.16, 0.00, 0.25, 0, 0.0, 0.25, 0.0, 1.5]  # [vx vy psidot y_e thetae theta s x y]
    x0_3 = [2.5, -0.16, 0.00,-0.25, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]

    # versio linear del solver
    # x0_0 = [1.3, -0.16, 0.00, 0.45, 0, 0.0, 0, 0.0, 1.45]  # [vx vy psidot y_e thetae theta s x y]
    # x0_1 = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]
    # x0_2 = [1.3, -0.16, 0.00, 0.25, 0, 0.0, 0.25, 0.0, 1.5]  # [vx vy psidot y_e thetae theta s x y]
    # x0_3 = [1.3, -0.16, 0.00, -0.25, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]

    # initialise data structures
    maps = [Map(),Map(),Map(),Map()]
    agents,data = initialise_agents([x0_0,x0_1,x0_2,x0_3],N,dt,maps)

    if plot:
        disp = plotter(maps[0],4)

    if plot_end:
        d = plotter_offline(maps[0])

    # initialise controllers
    r0 = agent(N, maps[0], dt, x0_0, 0, dth)
    r1 = agent(N, maps[1], dt, x0_1, 1, dth)
    r2 = agent(N, maps[2], dt, x0_2, 2, dth)
    r3 = agent(N, maps[3], dt, x0_3, 3, dth)

    r0.data_share = [data[i] for i in n_0]
    r1.data_share = [data[i] for i in n_1]
    r2.data_share = [data[i] for i in n_2]
    r3.data_share = [data[i] for i in n_3]

    x_old0 = None
    x_old1 = None
    x_old2 = None
    x_old3 = None

    u_old0 = None
    u_old1 = None
    u_old2 = None
    u_old3 = None

    planes_old = None
    old_solution0 = None
    old_solution1 = None
    old_solution2 = None
    old_solution3 = None

    cost_old = np.zeros((n_agents, n_agents, N))
    lambdas_hist = []
    cost_hist = []
    it = 0

    while(it<max_it):

        tic = time.time()
        lambdas = np.zeros((n_agents, n_agents, N))
        it_OCD = 0
        itc = 0

        while(not (it_OCD > 2  and finished)) :
            # OCD loop, we want to force at least 2 iterations + it_conv iterations without significant changes
            # run an instance of the optimisation problems
            f0, uPred0, xPred0, planes0, lsack0, Solution0 = r0.one_step(x_old0, lambdas[0,n_0,:], agents[:,n_0,:], n_0, u_old0, old_solution0)
            f1, uPred1, xPred1, planes1, lsack1, Solution1 = r1.one_step(x_old1, lambdas[1,n_1,:], agents[:,n_1,:], n_1, u_old1, old_solution1)
            f2, uPred2, xPred2, planes2, lsack2, Solution2 = r2.one_step(x_old2, lambdas[2,n_2,:], agents[:,n_2,:], n_2, u_old2, old_solution2)
            f3, uPred3, xPred3, planes3, lsack3, Solution3 = r3.one_step(x_old3, lambdas[3,n_3,:], agents[:,n_3,:], n_3, u_old3, old_solution3)

            # share the results within the network
            r0.data_share = [r1.data_opti,r2.data_opti,r3.data_opti]
            r1.data_share = [r0.data_opti,r2.data_opti,r3.data_opti]
            r2.data_share = [r0.data_opti,r1.data_opti,r3.data_opti]
            r3.data_share = [r0.data_opti,r1.data_opti,r2.data_opti]

            cost = np.zeros((n_agents,n_agents,N))

            # update the values of x,y for the obstacle avoidance constraints
            agents[:,0,:] = xPred0[:,-2:]
            agents[:,1,:] = xPred1[:,-2:]
            agents[:,2,:] = xPred2[:,-2:]
            agents[:,3,:] = xPred3[:,-2:]

            for k in range(1,N+1):
                for i in range(0,n_agents):
                    for j in range(0,n_agents):

                        if (i != j) and i<j:
                            cost[i,j,k-1]= eval_constraint(agents[k,i,:],agents[k,j,:],dth)


            lambdas += alpha*cost # update lambdas

            lambdas_hist.append(lambdas)
            # check if the values of x changed, if they are close enough for two iterations the algorithm has converged
            if it_OCD != 0:
                finished_ph = np.allclose(x_old2, xPred2, atol=0.01) and np.allclose(x_old3, xPred3, atol=0.01) and np.allclose(x_old0, xPred0, atol=0.01) and np.allclose(x_old1, xPred1, atol=0.01) and np.allclose(cost, cost_old, atol=0.01) #convergence([xPred0,xPred1,uPred0,uPred1], [x_old0_OCD,x_old1_OCD,u_old0_OCD,u_old1_OCD]) and
                itc += 1
                print(finished_ph)

            # store values from current iteration into the following one
            x_old0 = xPred0
            x_old1 = xPred1
            x_old2 = xPred2
            x_old3 = xPred3

            u_old0 = uPred0
            u_old1 = uPred1
            u_old2 = uPred2
            u_old3 = uPred3
            cost_old = cost


            if not finished_ph :
                print("breakpoint placeholder with " + str(it_OCD))
                itc = 0

            elif itc > it_conv:
                finished = True
                print("Iteration finished with " + str(it_OCD) + " steps")

            if it_OCD > 40:
                print("max it reached")
                finished = True

            it_OCD += 1


        #save current iteration for logging purposes
        r0.save(xPred0, uPred0, planes0)
        r1.save(xPred1, uPred1, planes1)
        r2.save(xPred2, uPred2, planes2)
        r3.save(xPred3, uPred3, planes3)

        r0.x0 = xPred0[1,:]
        r1.x0 = xPred1[1,:]
        r2.x0 = xPred2[1,:]
        r3.x0 = xPred3[1,:]

        x_old0 = xPred0[1:,:]
        x_old1 = xPred1[1:,:]
        x_old2 = xPred2[1:,:]
        x_old3 = xPred3[1:,:]

        u_old0 = uPred0
        u_old1 = uPred1
        u_old2 = uPred2
        u_old3 = uPred3

        old_solution0 = Solution0
        old_solution1 = Solution1
        old_solution2 = Solution2
        old_solution3 = Solution3

        finished = False
        time_OCD.append((time.time() - tic)/4)
        cost_hist.append(r0.Controller._cost)

        ## printing states
        # print("-------------------------------------------------")
        # print("it " + str(it))
        # print("length " + str(it_OCD))
        # print(time.time() - tic)
        # print(xPred0[1,:])
        # print(xPred1[1,:])
        # print(xPred2[1,:])
        # print(xPred3[1,:])
        # print(uPred0[0,:])
        # print(uPred1[0,:])
        # print(uPred2[0,:])
        # print(uPred3[0,:])
        # print("-------------------------------------------------")

        it += 1
        if plot :
            disp.plot_step(xPred0[1, 7], xPred0[1, 8], xPred0[1, 5], 0)
            disp.plot_step(xPred1[1, 7], xPred1[1, 8], xPred1[1, 5], 1)

    if plot_end:
        d.plot_offline_experiment(r0, "oc", "-y")
        d.plot_offline_experiment(r1, "ob", "-y")
        d.plot_offline_experiment(r2, "or", "-y")
        d.plot_offline_experiment(r3, "oy", "-y")
        r0.save_to_csv()
        r1.save_to_csv()
        r2.save_to_csv()
        r3.save_to_csv()
        r0.save_var_to_csv(time_OCD, "time_OCD")
        r0.save_var_to_csv(cost_hist, "cost_hist2")
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




