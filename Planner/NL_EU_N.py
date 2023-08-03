import time
import numpy as np

from Planner.planners.nonLinDistribPlanner import PlannerEu
from Planner.packages.mapManager import Map
from Planner.packages.utilities import checkEnd, initialise_agents
from Planner.packages.IOmodule import io_class
from Planner.packages.config.NL import initialiserNL, x0_database, settings, eval_constraintEU, get_alpha

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class agent(initialiserNL):

    def __init__(self, settings, maps, x0, id ):
        super().__init__(self, settings) # initialise the initialiser
        self.map = maps
        self.dt = settings["dt"]
        self.N =  settings["N"]
        self.x0 = x0
        self.Controller = PlannerEu(self.Q,self.Qs, self.R, self.dR, self.N, self.dt, maps, id, self.model_param, self.sys_lim)
        self.states = []
        self.u = []
        self.time_op = []
        self.status = []
        self.data_opti = []
        self.data_collec = []
        self.id = id

    def one_step(self, lambdas, agents, agents_id, uPred = None, xPred = None):
        tic = time.time()
        feas, Solution, self.data_opti = self.Controller.solve(self.x0, xPred, uPred, lambdas, agents, agents_id, self.data_collec)
        self.time_op.append(time.time() - tic)
        self.save(self.Controller.xPred, self.Controller.uPred, feas)


        return feas, self.Controller.uPred, self.Controller.xPred, Solution


def main():

#########################################################
#########################################################

    # Map settings

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
    ns = [[i for i in range(0, n_agents)] for j in range(0, n_agents)]

    for j, n in enumerate(ns):
        n.remove(j)

    # initialise data structures
    maps = [Map(map_type)]*n_agents
    agents,x_old,u_old = initialise_agents(x0,N,dt,maps)

    x_pred = [None] * n_agents
    u_pred = [None] * n_agents
    feas   = [None] * n_agents
    raws   = [None] * n_agents
    rs     = [None] * n_agents
    data   = [None] * n_agents

    for i in range(0, n_agents):
        data[i] = [x_old[i], u_old[i], np.zeros((N, 2)), np.zeros((N, 2)), np.zeros((N, n_agents)),
                    np.zeros((N, n_agents))]

    # initialise controllers and data holders
    for i in range (0,n_agents):
        rs[i] = agent(settings, maps[i], x_old[i], i)
        rs[i].data_collec = [data[j] for j in ns[i]]

    io = io_class(settings, rs)

    cost_old = np.zeros((n_agents, n_agents, N))
    lambdas_hist = []
    it = 0
    lambdas = np.zeros((n_agents, n_agents, N))
    error = False

    while(it<max_it and not checkEnd(x_pred, maps)):

        it_OCD = 0
        itc = 0

        while(not (it_OCD > min_it_OCD and finished)) :
            # OCD loop, we want to force at least 2 iterations + it_conv iterations without significant changes
            # run an instance of the optimisation problems
            io.tic()

            for i, r in enumerate(rs):
                feas[i], u_pred[i], x_pred[i], raws[i] = r.one_step( lambdas[[i],ns[i],:], agents[:,ns[i],:], ns[i], u_old[i], raws[i])

            if not np.any(feas):
                error = True
                break

            io.toc()

            for j,r in enumerate(rs):
                r.data_collec = [rs[i].data_opti for i in ns[j]]

            cost = np.zeros((n_agents,n_agents,N))
            # update the values of x,y for the obstacle avoidance constraints
            agents = np.swapaxes(np.asarray(x_pred)[:, :, -2:], 0, 1)

            if n_agents == 1:
                break

            for k in range(1,N+1):
                for i in range(0,n_agents):
                    for j in range(0,n_agents):

                        if (i != j) and i<j:
                            cost[i,j,k-1] = eval_constraintEU(agents[k,i,:],agents[k,j,:],dth)

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

            if not finished_ph :
                itc = 0

            elif itc > it_conv:
                finished = True
                print("Iteration finished with " + str(it_OCD) + " steps")

            if it_OCD > max_it_OCD:
                print("max it reached")
                finished = True

            # store values from current iteration into the following one
            x_old = x_pred
            cost_old = cost

            io.updateOCD(x_pred, it_OCD, it)
            it_OCD += 1

        for j,r in enumerate(rs):
            r.save(x_pred[j], u_pred[j])
            r.x0 = x_pred[j][1:, :]

        if it == 0:
            rs[0].save_var_pickle([lambdas], ["ini_lambdas"])


        u_old = u_pred
        finished = False
        io.update( x_pred, u_pred ,agents, it, error = error, OCD_ct=it_OCD)
        it += 1

        if error:
            break

    io.update(x_pred, u_pred, agents, it, end=True,error = error)

if __name__ == "__main__":

    main()




