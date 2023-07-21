#!/usr/bin/env python
import time
from casadi import *
import numpy as np
from utilities import Curvature, get_ey

# TODO Arreglar el cost funcion

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# slack weight constants
model_slack = 10000000
control_slack = 1
obs_slack = 1000000

class NL_Planner_EU:
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given ini_xPredicted computes the control action
    """
    def __init__(self, Q, R, N, dt, map, id, dth):

        # system parameters:
        self.n_exp = 9
        self.dth = dth
        self.lf = 0.12
        self.lr = 0.14
        self.m  = 2.250
        self.I  = 0.06
        self.Cf = 60.0
        self.Cr = 60.0
        self.mu = 0.0
        self.vx_ref = 4.5
        self.id = id
        self.initialised = False # Flag to instantiate the optimisation problem
        self.g  = 9.81
        self.max_vel = 10
        self.min_vel = -10

        # declaration of the optimisation variables
        self.opti = casadi.Opti()
        self.x = self.opti.variable(self.n_exp*(N+1)) # x0, ..., xN
        self.u  = self.opti.variable(2 * (N))  # u1, ..., uN
        self.du = self.opti.variable(2 * (N))  # du1, ..., duN
        self.slack_agent = self.opti.variable(N,4)  # s00, s10, s20, s30, ..., s0N, s1N, s2N, s3N,
        self.ey_ub = self.opti.parameter(N)
        self.ey_lb = self.opti.parameter(N)

        # sistem matrices delaration
        self.A    = []
        self.B    = []
        self.C    = []
        self.N    = N
        self.n    = Q.shape[0]
        self.d    = R.shape[0]
        self.Q    = Q
        self.R    = R
        self.dt = dt              # Sample time
        self.map = map            # Used for getting the road curvature
        self.planes_param = []    # neighbouring planes
        self.pose_param = []      # neighbour x,y
        self.du_param = []        # neighbour du
        self.states_param = []    # neighbour states
        self.s_agent_param = []   # neighbour states slack
        self.param_slack_dis = [] # distance constraint slack
        self._cost = 0 # cost logging

        # parameters
        self.initial_states = self.opti.parameter(self.n_exp)

        # LPV placeholders
        # vx
        self.A12 = self.opti.parameter(self.N)
        self.A13 = self.opti.parameter(self.N)
        self.B11 = self.opti.parameter(self.N)

        # vy
        self.A22 = self.opti.parameter(self.N)
        self.A23 = self.opti.parameter(self.N)
        self.B21 = self.opti.parameter(self.N)

        # wz
        self.A32 = self.opti.parameter(self.N)
        self.A33 = self.opti.parameter(self.N)
        self.B31 = self.opti.parameter(self.N)

        # ey
        self.A44 = self.opti.parameter(self.N)

        # epsi
        self.A51 = self.opti.parameter(self.N)
        self.A52 = self.opti.parameter(self.N)

        # s
        self.A61 = self.opti.parameter(self.N)
        self.A62 = self.opti.parameter(self.N)

        # x
        self.A81 = self.opti.parameter(self.N)
        self.A82 = self.opti.parameter(self.N)

        # y
        self.A91 = self.opti.parameter(self.N)
        self.A92 = self.opti.parameter(self.N)


    def cost(self):

        # parametric generation cost function
        J  = 0
        for j in range (1,self.N+1):
            mod = j * self.n_exp
            mod_u = (j-1) * 2

            # cost asociated to the current agent
            J += self.Q[0,0]*(self.x[0+mod] - self.vx_ref)**2 + self.Q[1,1]*self.x[1+mod]**2 + self.Q[2,2]*self.x[2+mod]**2 +\
                 self.Q[3,3]*self.x[3+mod]**2 + self.Q[4,4]*self.x[4+mod]**2 + self.Q[5,5]*self.x[5+mod]**2 + self.Q[6,6]*self.x[6+mod]**2 + self.Q[7,7]*self.x[7+mod]**2 \
                 + self.Q[8,8]*self.x[8+mod]**2 + self.R[0,0]*self.du[0+(mod_u)]**2 + self.R[1,1]*self.du[1+mod_u]**2 + model_slack*(self.slack_agent[j-1,0]**2 + self.slack_agent[j-1,1]**2)**2 + control_slack*(self.slack_agent[j-1,2]**2 + self.slack_agent[j-1,3]**2)

            for i, el in enumerate(self.agent_list):

                slack_idx = (j - 1) * self.aux + i

                # cost asociated to the neighbouring agents
                J += self.Q[0,0] * (self.states_param[i][0 + mod] - self.vx_ref)** 2+ self.Q[1,1] * self.states_param[i][1 + mod] ** 2 + self.Q[2,2] * self.states_param[i][2 + mod] ** 2 + \
                     self.Q[3,3] * self.states_param[i][3 + mod] ** 2 + self.Q[4,4] * self.states_param[i][4 + mod] ** 2 + self.Q[5,5] * self.states_param[i][5 + mod] ** 2 + self.Q[6,6] * self.states_param[i][6 + mod] ** 2 + \
                     self.Q[7,7] * self.states_param[i][7 + mod] ** 2 + self.R[0,0]*self.du_param[i][0 + (mod_u)] ** 2 + self.R[1,1]*self.du_param[i][1 + mod_u] ** 2  + model_slack * (
                     self.s_agent_param[i][j-1,0] ** 2 + self.s_agent_param[i][j-1,1]**2) + control_slack*(self.s_agent_param[i][j-1,2]**2 + self.s_agent_param[i][j-1,3] ** 2)

                # add cost depending on if you are a follower or an ego vehicle in the pair

                if self.id < el:
                    J+= self.lambdas[i,j-1]*(self.param_slack_dis[i][j-1,self.id] + self.dth - sqrt((self.x[7+mod] - self.pose_param[i][j-1,0])**2 + (self.x[8+mod]
                         - self.pose_param[i][j-1,1])**2)) + obs_slack*(self.param_slack_dis[i][j-1,self.id]**2)

                else:
                    J += obs_slack * (self.slack_dis[slack_idx] ** 2)

        return J

    def ineq_constraints(self):
        #parametric definition of inequality constraints along the horizon

        for j in range(1, self.N + 1):
            mod = j * self.n_exp
            mod_u = (j-1) * 2

            # bound linear velocity
            self.opti.subject_to(self.opti.bounded(self.min_vel,self.x[0+mod] + self.slack_agent[j-1,0],self.max_vel))
            # bound lateral error acording to the part of the track being traversed
            self.opti.subject_to(self.opti.bounded(self.ey_lb[j-1], self.x[4+mod] + self.slack_agent[j-1,1], self.ey_ub[j-1]))

            # bound control actions
            self.opti.subject_to(self.opti.bounded(-0.45,self.u[0+mod_u], 0.45))
            self.opti.subject_to(self.opti.bounded(-8.00, self.u[1 + mod_u], 8.0))

            for i,el in enumerate(self.agent_list):
                slack_idx = (j - 1) * self.aux + i
                if self.id > el:
                    #If ego vehicle add euclidean distance constraint, otherwise this will be part of the slave cost function
                    self.opti.subject_to( sqrt((self.x[7+mod] - self.pose_param[i][j-1,0])**2 + (self.x[8+mod] - self.pose_param[i][j-1,1])**2) + self.slack_dis[slack_idx] > self.dth )

    def eq_constraints(self):

        # set initial states
        for i in range(0, self.n_exp):

            self.opti.subject_to(
                self.x[i] == self.initial_states[i]
            )

        # add the model
        for j in range(1, self.N + 1):
            mod = j * self.n_exp
            mod_prev = (j-1) * self.n_exp
            mod_u    = (j-1) * 2

            # vx
            self.opti.subject_to(
                self.x[0+mod] == self.x[0+mod_prev] + (-self.mu*self.x[0+mod_prev] + self.A12[j-1]*self.x[1+mod_prev] + self.A13[j-1]*self.x[2+mod_prev] + self.B11[j-1]*self.u[0+mod_u] + self.u[1+mod_u])*self.dt
            )

            # vy
            self.opti.subject_to(
                self.x[1+mod] == self.x[1+mod_prev] + (self.A22[j-1]*self.x[1+mod_prev] + self.A23[j-1]*self.x[2+mod_u] + self.B21[j-1]*self.u[0+mod_u])*self.dt
            )

            # wz
            self.opti.subject_to(
                self.x[2+mod] == self.x[2+mod_prev] + (self.A32[j-1]*self.x[1+mod_prev] + self.A33[j-1]*self.x[2+mod_u] + self.B31[j-1]*self.u[0+mod_u])*self.dt
            )

            # ey

            self.opti.subject_to(
                self.x[3+mod] == self.x[3+mod_prev] + (self.x[1+mod_prev] + self.A44[j-1]*self.x[4+mod_prev])*self.dt
            )

            # epsi

            self.opti.subject_to(
                self.x[4+mod] == self.x[4+mod_prev] + (self.A51[j-1] * self.x[0+mod_prev] + self.A52[j-1] * self.x[1+mod_prev] + self.x[2+mod_prev])*self.dt
            )

            # theta

            self.opti.subject_to(
                self.x[5+mod] == self.x[5+mod_prev] + (self.x[2+mod_prev])*self.dt
            )

            # s

            self.opti.subject_to(
                self.x[6+mod] == self.x[6+mod_prev] + (self.A61[j-1]*self.x[0+mod_prev] + self.A62[j-1]*self.x[1+mod_prev])*self.dt
            )

            # x

            self.opti.subject_to(
                self.x[7+mod] == self.x[7+mod_prev] + (self.A81[j-1]*self.x[0+mod_prev] + self.A82[j-1]*self.x[1+mod_prev])*self.dt
            )

            # y

            self.opti.subject_to(
                self.x[8+mod] == self.x[8+mod_prev] + (self.A91[j-1] * self.x[0+mod_prev] + self.A92[j-1] * self.x[1+mod_prev])*self.dt
            )

            # propagate the control ation
            if j < self.N:
                self.opti.subject_to(self.u[0 + mod_u + 2] == self.u[0 + mod_u] + self.du[0 + mod_u])
                self.opti.subject_to(self.u[1 + mod_u + 2] == self.u[1 + mod_u] + self.du[1 + mod_u])

    def update_parameters(self,states,u):

        # Update the parametric expresions inside the solver

        ey = get_ey(states[:, 6], self.map) # update lateral error limits

        try:
            self.opti.set_value(self.ey_ub, ey)
            self.opti.set_value(self.ey_lb, -ey)
        except:
            self.opti.set_value(self.ey_ub, ey[1:])
            self.opti.set_value(self.ey_lb, -ey[1:])

        # update model variables
        for j in range (0,self.N):

            vx = states[j, 0]
            vy = states[j, 1]
            ey = states[j, 3]
            epsi = states[j, 4]
            theta = states[j, 5]
            s = states[j, 6]

            cur = Curvature(s, self.map)
            delta = u[j, 0]  # EA: steering angle at K-1

            self.opti.set_value(self.A12[j], (np.sin(delta) * self.Cf) / (self.m * vx))
            self.opti.set_value(self.A13[j], (np.sin(delta) * self.Cf * self.lf) / (self.m * vx) + vy)

            self.opti.set_value(self.A22[j], -(self.Cr + self.Cf * np.cos(delta)) / (self.m * vx))
            self.opti.set_value(self.A23[j], -(self.lf * self.Cf * np.cos(delta) - self.lr * self.Cr) / (self.m * vx) - vx)

            self.opti.set_value(self.A32[j], -(self.lf * self.Cf * np.cos(delta) - self.lr * self.Cr) / (self.I * vx))
            self.opti.set_value(self.A33[j], -(self.lf * self.lf * self.Cf * np.cos(delta) + self.lr * self.lr * self.Cr) / (self.I * vx))

            self.opti.set_value(self.B11[j], -(np.sin(delta) * self.Cf) / self.m)

            self.opti.set_value(self.A51[j], (1 / (1 - ey * cur)) * (-cur))
            self.opti.set_value(self.A52[j], (1 / (1 - ey * cur)) * (np.sin(epsi) * cur))

            self.opti.set_value(self.A61[j], np.cos(epsi) / (1 - ey * cur))
            self.opti.set_value(self.A62[j], -np.sin(epsi) / (1 - ey * cur))

            self.opti.set_value(self.A44[j], vx)

            self.opti.set_value(self.A81[j], np.cos(theta))
            self.opti.set_value(self.A82[j], -np.sin(theta))

            self.opti.set_value(self.A91[j], np.sin(theta))
            self.opti.set_value(self.A92[j], np.cos(theta))

            self.opti.set_value(self.B21[j], (np.cos(delta) * self.Cf) / self.m)
            self.opti.set_value(self.B31[j], (self.lf * self.Cf * np.cos(delta)) / self.I)

            for i, el in enumerate(self.agent_list):

                self.opti.set_value(self.pose_param[i][j,0], self.states_fixed[j,i,0])
                self.opti.set_value(self.pose_param[i][j,1], self.states_fixed[j,i,1])

        # update neighbouring agents
        for i, el in enumerate(self.agent_list):
            self.opti.set_value(self.pose_param[i][-1, 0], self.states_fixed[-1, i, 0])
            self.opti.set_value(self.pose_param[i][-1, 1], self.states_fixed[-1, i, 1])

        # update initial states
        for i in range(0, self.n_exp):
            self.opti.set_value(self.initial_states[i] ,states[0,i])

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

                placeholder_states = self.opti.parameter(self.n_exp * (self.N + 1))
                self.states_param.append(placeholder_states)

                placeholder_u = self.opti.parameter(2 * (self.N))
                self.du_param.append(placeholder_u)

                placeholder_sa = self.opti.parameter(self.N,4)
                self.s_agent_param.append(placeholder_sa)

                placeholder_sp = self.opti.parameter((self.N), (self.n_neighbours+1))
                self.param_slack_dis.append(placeholder_sp)

            # if we are masters we will add a slack to the distance constraint
            if self.aux != 0:
                self.slack_dis = self.opti.variable((self.N) * self.aux)

            # store constraint and cost function
            self.ineq_constraints()
            self.eq_constraints()
            J = self.cost()
            self.opti.minimize(J)
            self.initialised = True

        # set initial statse
        if not Last_xPredicted is None:
            self.opti.set_initial(self.x,Last_xPredicted.flatten())

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        # set control actions
        if not uPred is None:
            self.opti.set_initial(self.u,uPred.flatten())

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        # set lambdas
        if lambdas is None:
            self.opti.set_value(self.lambdas,np.ones((self.n_neighbours, self.N+1)))

        else:
            self.opti.set_value(self.lambdas,lambdas)

        # unpack data [x,du,slack_agent,slack]

        for i,agent in enumerate(data):

            self.opti.set_value(self.states_param[i], agent[0])
            self.opti.set_value(self.du_param[i], agent[1])
            self.opti.set_value(self.s_agent_param[i], agent[2])
            self.opti.set_value(self.param_slack_dis[i], agent[3])

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        if x_agents is None:
            x_agents = np.zeros((10,self.n_neighbours,2))

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
            self._cost = sol.stats()['iterations']['obj'][-1]

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


        # Create the output values TODO: is this necesary?
        idx = np.arange(0, 9)
        d = 2
        for i in range(1, self.N + 1):
            aux = np.arange(0, 9) + i * self.n_exp
            idx = np.hstack((idx, aux))

        self.xPred = np.reshape((x)[idx], (self.N + 1, self.n_exp)) # retrieve states
        self.uPred = np.reshape(u, (self.N, d)) # retrieve control actions

        self.solverTime = time.time() - startTimer # keep the solver

        # load the slack variables into the placeholder

        slack = np.zeros(((self.N),(self.n_neighbours+1)))

        if not self.aux == 0:
            slack[:,0:(self.id)] = self._slack.reshape((self.N, -1))

        data = [x,du,slack_agent,slack] #return the data

        return status, x, 0, slack, data # the 0 is a placeholder to the generated planes, which do not exist here
