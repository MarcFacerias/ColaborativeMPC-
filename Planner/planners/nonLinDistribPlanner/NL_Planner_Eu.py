#!/usr/bin/env python
import time
from .base_nl import *

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class PlannerEu(base_nl_constr):
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given ini_xPredicted computes the control action
    """
    def __init__(self, Q, Qs, R, dR, N, dt, map, id, model_param = None, sys_lim = None):
        super().__init__(Q, Qs, R, dR, N, dt, map, id, model_param, sys_lim) # initialise the initialiser

    def cost(self):

        # parametric generation cost function
        J  = 0
        for j in range (1,self.N+1):

            # cost asociated to the current agent
            J += (self.Q[0,0]*(self.x[j,0] - self.vx_ref)**2 + self.Q[1,1]*self.x[j,1]**2 +
                  self.Q[2,2]*self.x[j,2]**2 + self.Q[3,3]*self.x[j,3]**2 +
                  self.Q[4,4]*self.x[j,4]**2 + self.Q[5,5]*self.x[j,5]**2 +
                  self.Q[6,6]*self.x[j,6]**2 + self.Q[7,7]*self.x[j,7]**2 +\
                  self.Q[8,8]*self.x[j,8]**2 +
                 self.dR[0,0]*self.du[j-1,0]**2 + self.dR[1,1]*self.du[j-1,1]**2 +
                 self.R[0,0] * self.u[j-1,0] ** 2 + self.R[1,1] * self.u[j-1,1] ** 2 +
                 self.model_slack*(self.slack_agent[j-1,0]**2 + self.slack_agent[j-1,1]**2 ))

            for i, el in enumerate(self.agent_list):

                # cost asociated to the neighbouring agents
                J += (self.Q[0,0] * (self.states_param[i][j,0] - self.vx_ref)** 2+ self.Q[1,1] * self.states_param[i][j,1] ** 2 +
                      self.Q[2,2] * self.states_param[i][j,2] ** 2 + self.Q[3,3] * self.states_param[i][j,3] ** 2 +
                      self.Q[4,4] * self.states_param[i][j,4] ** 2 + self.Q[5,5] * self.states_param[i][j,5] ** 2 +
                      self.Q[6,6] * self.states_param[i][j,6] ** 2 + self.Q[7,7] * self.states_param[i][j,7] ** 2 +
                      self.Q[8,8] * self.states_param[i][j,8] ** 2 +
                      self.R[0,0] * self.u_param[i][j-1,0] ** 2 + self.R[1,1] * self.u_param[i][j-1,1] ** 2 +
                      self.dR[0,0]* self.du_param[i][j-1,0] ** 2 + self.dR[1,1]*self.du_param[i][j-1,1] ** 2  +
                      self.model_slack * (self.s_agent_param[i][j-1,0] ** 2 + self.s_agent_param[i][j-1,1]**2))


                if self.id < el:
                    J+= self.lambdas[i,j-1]*(self.param_slack_dis[i][j-1,self.id] + self.dth - sqrt((self.x[j,7] - self.pose_param[i][j-1,0])**2 + (self.x[j,8]
                         - self.pose_param[i][j-1,1])**2)) + self.obs_slack*(self.param_slack_dis[i][j-1,i]**2)

                else:
                    J += self.obs_slack * (self.slack_dis[j-1,i] ** 2)

        return J

    def ineq_constraints(self):
        #parametric definition of inequality constraints along the horizon

        for j in range(1, self.N + 1):

            # bound linear velocity
            self.opti.subject_to(self.opti.bounded(self.min_vel,self.x[j,0] + self.slack_agent[j-1,1], self.max_vel))
            # bound lateral error acording to the part of the track being traversed
            self.opti.subject_to(self.opti.bounded(self.ey_lb[j-1], self.x[j,3] + self.slack_agent[j-1,0], self.ey_ub[j-1]))

            # bound control actions
            self.opti.subject_to(self.opti.bounded(-self.max_ls, self.u[j-1,0], self.max_rs))
            self.opti.subject_to(self.opti.bounded(-self.max_dc, self.u[j-1,1], self.max_ac))

            for i,el in enumerate(self.agent_list):
                if self.id > el:
                    #If ego vehicle add euclidean distance constraint, otherwise this will be part of the slave cost function
                    self.opti.subject_to( sqrt((self.x[j,7] - self.pose_param[i][j-1,0])**2 + (self.x[j,8] - self.pose_param[i][j-1,1])**2) + self.slack_dis[j-1,i] > self.dth )

    def solve(self, ini_xPredicted = None, Last_xPredicted = None, uPred = None, lambdas = None, x_agents = None, agents_id = None, data = None):
        """Computes control action
        Arguments:
            ini_xPredicted: current state predicted
            EA: Last_xPredicted: it is just used for the warm up
            EA: uPred: set of last predicted control inputs used for updating matrix A LPV
            EA: A_L, B_L ,C_L: Set of LPV matrices
        """
        startTimer              = time.time()

        # if it's the first iteration we need to generate the system matrixes
        if not self.initialised:

            # check how many times we are the ego or the slave
            self.agent_list = np.asarray(agents_id)
            self.aux = (self.agent_list < self.id).sum()

            self.n_neighbours = self.agent_list.size #set the number of neigbours
            self.states_fixed = np.zeros(((self.N), self.n_neighbours, 3)) # set the pose of the neighbours TODO: resize this to 2 (x,y)
            self.lambdas = self.opti.parameter(self.n_neighbours, self.N) # alocate the lambdas

            # as parameters only allow 2D structures we will do a list where each entry containts the info of one of the neighbouring agents
            for i in range(0, len(self.agent_list)):

                placeholder_pose = self.opti.parameter(self.N, 2)
                self.pose_param.append(placeholder_pose)

                placeholder_states = self.opti.parameter(self.N + 1,self.n_s)
                self.states_param.append(placeholder_states)

                placeholder_du = self.opti.parameter(self.N,2)
                placeholder_u = self.opti.parameter(self.N,2)

                self.du_param.append(placeholder_du)
                self.u_param.append(placeholder_u)

                placeholder_sa = self.opti.parameter(self.N,2)
                self.s_agent_param.append(placeholder_sa)

                placeholder_sp = self.opti.parameter(self.N,self.n_neighbours+1)
                self.param_slack_dis.append(placeholder_sp)

            # if we are masters we will add a slack to the distance constraint
            if self.aux != 0:
                self.slack_dis = self.opti.variable(self.N,self.aux)

            # store constraint and cost function
            self.ineq_constraints()
            self.eq_constraints()
            J = self.cost()
            self.opti.minimize(J)
            self.initialised = True

        # set initial statse
        if not Last_xPredicted is None:
            self.opti.set_initial(self.x,Last_xPredicted)

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        # set control actions
        if not uPred is None:
            self.opti.set_initial(self.u,uPred)
            self.opti.set_value(self.initial_u, uPred[0,:])
        else:
            self.opti.set_value(self.initial_u, np.zeros((self.n_u,1)))

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        # set lambdas
        if lambdas is None:
            self.opti.set_value(self.lambdas,np.ones((self.n_neighbours, self.N+1)))

        else:
            self.opti.set_value(self.lambdas,lambdas)

        # unpack data [x,du,slack_agent,slack]

        for i,agent in enumerate(data):

            self.opti.set_value(self.states_param[i], agent[0])
            self.opti.set_value(self.u_param[i], agent[1])
            self.opti.set_value(self.du_param[i], agent[2])
            self.opti.set_value(self.s_agent_param[i], agent[3])
            self.opti.set_value(self.param_slack_dis[i], agent[4])

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        if x_agents is None:
            x_agents = np.zeros((self.N,self.n_neighbours,2))

        self.states_fixed = x_agents

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        self.update_parameters(ini_xPredicted,uPred) #update parameters in the control problem and run the optimisation

        p_opts = {"ipopt.print_level":0, "ipopt.sb":"yes", "print_time":0}
        s_opts = {"max_iter": 1}
        self.opti.solver("ipopt", p_opts,
                    s_opts)

        self.settingTime = time.time() - startTimer

        # we use try except structures to catch cases were the solver is not accurate enough or some variables are not
        # generated due to a particular master slave distribution

        try:
            sol = self.opti.solve()
            status = True
            x = sol.value(self.x)
            u = sol.value(self.u)
            du = sol.value(self.du)

            # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

            try:
                slack_agent = sol.value(self.slack_agent)
            except:
                slack_agent = None

            if not self.aux == 0:
                self._slack = sol.value(self.slack_dis)


        except Exception as e:
            print(e)
            status = False
            x = self.opti.debug.value(self.x)
            u = self.opti.debug.value(self.u)
            du = self.opti.debug.value(self.du)

            # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

            try:
                slack_agent = self.opti.debug.value(self.slack_agent)
            except:
                slack_agent = None

            # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

            if not self.aux == 0:
                self._slack = self.opti.debug.value(self.slack_dis)

        self.xPred = x
        self.uPred = u

        self.solverTime = time.time() - startTimer # keep the solver

        # load the slack variables into the placeholder

        slack = np.zeros(((self.N),(self.n_neighbours+1)))

        if not self.aux == 0:
            slack[:,0:(self.id)] = self._slack.reshape((self.N,-1))


        data = [x,u,du,slack_agent,slack] #return the data

        return status, x, data # the 0 is a placeholder to the generated planes, which do not exist here
