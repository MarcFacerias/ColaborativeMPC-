#!/usr/bin/env python
import time
from casadi import *
import numpy as np
from utilities import Curvature, GBELLMF
from compute_plane import hyperplane_separator

# TODO Arreglar els idx per a fer que es pugui fer la decomposicio amb mes vehicles

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class PathFollowingNL_MPC:
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given x0 computes the control action
    """
    def __init__(self, Q, R, N, dt, map, id):

        # Vehicle parameters:
        self.n_s = 9
        self.n_neighbours = 1
        self.n_slack = 0
        self.n_exp = self.n_s + self.n_slack# slack variables
        self.dth = 0.2
        self.lf = 0.12
        self.lr = 0.14
        self.m  = 2.250
        self.I  = 0.06
        self.Cf = 60.0
        self.Cr = 60.0
        self.mu = 0.1
        self.id = id
        self.plane_comp = hyperplane_separator(self.n_neighbours, N)
        self.initialised = False

        self.g  = 9.81

        self.max_vel = 10
        self.opti = casadi.Opti()
        self.x = self.opti.variable(self.n_exp*(N+1)) # x11, ... , x1N, s11, ... xTN
        self.u = self.opti.variable(2 * (N))  # x11, ... , x1N, s11, ... xTN
        self.slack_agent = self.opti.variable(N+1)  # x11, ... , x1N, s11, ... xTN
        self.planes_fixed = np.zeros(((N),self.n_neighbours,3))
        self.states_fixed = np.zeros(((N), self.n_neighbours, 3))

        self.A    = []
        self.B    = []
        self.C    = []
        self.N    = N
        self.n    = Q.shape[0]
        self.d    = R.shape[0]
        self.Q    = Q
        self.R    = R
        self.LinPoints = np.zeros((self.N+2,self.n))
        self.dt = dt                # Sample time 33 ms
        self.map = map              # Used for getting the road curvature
        self.planes_constraints = []

        self.first_it = 1
        self.flag_lambdas = True

        self.min_vel = 6
        self.min_vel = -6

        # parameters
        self.initial_states = self.opti.parameter(self.n_exp - self.n_slack)
        self.cur = self.opti.parameter(self.N)

        self.planes_param = []
        self.states_param = []

    def cost(self):

        J  = 0
        for j in range (1,self.N+1):
            mod = j * self.n_exp
            mod_u = (j-1) * 2

            J += 120*self.x[0+mod]**2 + self.x[1+mod]**2 + self.x[2+mod]**2 +\
                 1500*self.x[3+mod]**2 + 70*self.x[4+mod]**2 \
                 + self.u[0+(mod_u)]**2 + self.x[1+mod_u]**2 - 600*self.x[0+mod] + 10000*(self.slack_agent[j]**2)

            for i, el in enumerate(self.agent_list):
                planes_idx = (j - 1) * 3 * self.n_neighbours + 3 * i
                if self.id < el:
                    J+= self.lambdas[i,j-1]*(-(self.planes[planes_idx+0]*self.states_param[i][j-1,0]+ self.planes[planes_idx+1]*self.states_param[i][j-1,1] +self.planes[planes_idx+2]- self.dth/2 ) )

        return J

    def ineq_constraints(self):

        for j in range(1, self.N + 1):
            mod = j * self.n_exp
            mod_u = (j-1) * 2
            mod_planes = (j - 1) * self.n_neighbours

            self.opti.subject_to(self.opti.bounded(self.min_vel,self.x[0+mod],self.max_vel))
            self.opti.subject_to(self.opti.bounded(-0.60, self.x[4+mod] + self.slack_agent[j], 0.60))

            self.opti.subject_to(self.opti.bounded(-0.45,self.u[0+mod_u], 0.45))
            self.opti.subject_to(self.opti.bounded(-8.00,self.u[1+mod_u], 8.0))

            #planes
            for i,el in enumerate(self.agent_list):

                planes_idx = (j-1)*3*self.n_neighbours + 3*i

                if self.id < el:
                    #TODO: Repasar aquesta constraint
                    self.opti.subject_to( self.planes[planes_idx+0]*self.x[7+mod] + self.planes[planes_idx+1]*self.x[8+mod] + self.planes[planes_idx+2] <= self.dth)
                    # self.opti.subject_to( norm_2([self.planes[planes_idx+0]**2,self.planes[planes_idx+1]]) == 1.0)
                    self.opti.subject_to((self.planes[planes_idx + 0]**2 + self.planes[planes_idx + 1]**2) == 1.0)

                else:
                    # self.flag_lambdas = True
                    # cts = (-self.planes_param[i][j-1,0]*self.x[7+mod] - self.planes_param[i][j-1,1]*self.x[8+mod] - self.planes_param[i][j-1,2] + self.dth/2) <= 0.0
                    # self.planes_constraints.append(cts)
                    self.opti.subject_to((-self.planes_param[i][j-1,0]*self.x[7+mod] - self.planes_param[i][j-1,1]*self.x[8+mod] - self.planes_param[i][j-1,2] + self.dth) <= 0.0)



    def eq_constraints(self,states):
        # model constants
        lf = self.lf
        lr = self.lr
        m  = self.m
        I  = self.I
        Cf = self.Cf
        Cr = self.Cr
        mu = self.mu

        # set initial states
        for i in range(0, self.n_exp-self.n_slack):

            self.opti.subject_to(
                self.x[i] == self.initial_states[i]
            )


        for j in range(1, self.N + 1):
            mod = j * self.n_exp
            mod_prev = (j-1) * self.n_exp
            mod_u    = (j-1) * 2

            # vx check
            A11 = -mu
            A12 = (np.sin(self.u[0+mod_u]) * Cf) / (m * self.x[0+mod_prev])
            A13 = (np.sin(self.u[0+mod_u]) * Cf * lf) / (m * self.x[0+mod_prev]) + self.x[1+mod_prev]
            B11 = -(np.sin(self.u[0+mod_u]) * Cf) / m
            B12 = 1

            # vy check
            A22 = -(Cr + Cf * np.cos(self.u[0+mod_u])) / (m * self.x[0+mod_prev])
            A23 = -(lf * Cf * np.cos(self.u[0+mod_u]) - lr * Cr) / (m * self.x[0+mod_prev]) - self.x[0+mod_prev]
            B21 = (np.cos(self.u[0 + mod_u]) * Cf) / m

            # wz check
            A32 = -(lf * Cf * np.cos(self.u[0+mod_u]) - lr * Cr) / (I * self.x[0+mod_prev])
            A33 = -(lf * lf * Cf * np.cos(self.u[0+mod_u]) + lr * lr * Cr) / (I * self.x[0+mod_prev])
            B31 = (lf * Cf * np.cos(self.u[0 + mod_u])) / I

            # ey check
            A41 = 1
            A44 = self.x[0+mod_prev]

            # epsi check
            A51 = (1 / (1 - self.x[3+mod_prev] * self.cur[j-1])) * (-self.cur[j-1])
            A52 = (1 / (1 - self.x[3+mod_prev] * self.cur[j-1])) * (np.sin( self.x[4+mod_prev]) * self.cur[j-1])

            # s check
            A61 = np.cos( self.x[4+mod_prev]) / (1 - self.x[3+mod_prev] * self.cur[j-1])
            A62 = -np.sin( self.x[4+mod_prev]) / (1 - self.x[3+mod_prev] * self.cur[j-1])

            # x
            A81 = np.cos(self.x[5+mod_prev])
            A82 = -np.sin(self.x[5+mod_prev])

            # y
            A91 = np.sin(self.x[5+mod_prev])
            A92 = np.cos(self.x[5+mod_prev])


            # vx
            self.opti.subject_to(
                self.x[0+mod] == self.x[0+mod_prev] + (A11*self.x[0+mod_prev] + A12*self.x[1+mod_prev] + A13*self.x[2+mod_prev] + B11*self.u[0+mod_u] + B12*self.u[1+mod_u])*self.dt
            )

            # vy
            self.opti.subject_to(
                self.x[1+mod] == self.x[1+mod_prev] + (A22*self.x[1+mod_prev] + A23*self.x[2+mod_u] + B21*self.u[0+mod_u])*self.dt
            )

            # wz
            self.opti.subject_to(
                self.x[2+mod] == self.x[2+mod_prev] + (A32*self.x[1+mod_prev] + A33*self.x[2+mod_u] + B31*self.u[0+mod_u])*self.dt
            )

            # ey

            self.opti.subject_to(
                self.x[3+mod] == self.x[3+mod_prev] + (A41*self.x[1+mod_prev] + A44*self.x[4+mod_prev])*self.dt
            )

            # epsi

            self.opti.subject_to(
                self.x[4+mod] == self.x[4+mod_prev] + (A51 * self.x[0+mod_prev] + A52 * self.x[1+mod_prev] + self.x[2+mod_prev])*self.dt
            )

            # theta

            self.opti.subject_to(
                self.x[5+mod] == self.x[5+mod_prev] + (self.x[2+mod_prev])*self.dt
            )

            # s

            self.opti.subject_to(
                self.x[6+mod] == self.x[6+mod_prev] + (A61*self.x[0+mod_prev] + A62*self.x[1+mod_prev])*self.dt
            )

            # x

            self.opti.subject_to(
                self.x[7+mod] == self.x[7+mod_prev] + (A81*self.x[0+mod_prev] + A82*self.x[1+mod_prev])*self.dt
            )

            # y

            self.opti.subject_to(
                self.x[8+mod] == self.x[8+mod_prev] + (A91 * self.x[0+mod_prev] + A92 * self.x[1+mod_prev])*self.dt
            )

    def update_parameters(self,states):

        # set_planes fixed

        for j in range (0,self.N):

            self.opti.set_value(self.cur[j], Curvature(states[j, 6], self.map))

            for i, el in enumerate(self.agent_list):

                self.opti.set_value(self.planes_param[i][j,0], self.planes_fixed[j,0,i])
                self.opti.set_value(self.planes_param[i][j,1], self.planes_fixed[j,1,i])
                self.opti.set_value(self.planes_param[i][j,2], self.planes_fixed[j,2,i])

                self.opti.set_value(self.states_param[i][j,0], self.states_fixed[j,i,0])
                self.opti.set_value(self.states_param[i][j,1], self.states_fixed[j,i,1])



        for i, el in enumerate(self.agent_list):
            self.opti.set_value(self.states_param[i][-1, 0], self.states_fixed[-1, i, 0])
            self.opti.set_value(self.states_param[i][-1, 1], self.states_fixed[-1, i, 1])

        for i in range(0, self.n_exp-self.n_slack):
            self.opti.set_value(self.initial_states[i] ,states[0,i])



    def solve(self, x0 = None, Last_xPredicted = None, uPred = None, lambdas = None, x_agents = None, planes_fixed = None, agents_id = None, pose = None):
        """Computes control action
        Arguments:
            x0: current state positionsolve
            EA: Last_xPredicted: it is just used for the warm up
            EA: uPred: set of last predicted control inputs used for updating matrix A LPV
            EA: A_L, B_L ,C_L: Set of LPV matrices
        """
        startTimer              = time.time()

        # self.opti.set_initial(self.planes, Last_xPredicted.flatten())
        if not Last_xPredicted is None:
            self.opti.set_initial(self.x,Last_xPredicted.flatten())
            # print("ID " +str(self.id))
            # print(Last_xPredicted)

        if not uPred is None:
            self.opti.set_initial(self.u,uPred.flatten())

        if lambdas is None:
            self.lambdas = np.ones((self.n_neighbours, self.N+1))

        else:
            self.lambdas = lambdas

        if x_agents is None:
            x_agents = np.zeros((10,self.n_neighbours,2))

        if planes_fixed is None:
            self.planes_fixed = self.plane_comp.compute_hyperplane(x_agents, pose, self.id, agents_id)

        else:
            self.planes_fixed = planes_fixed

        self.agent_list = np.asarray(agents_id)
        aux = (self.agent_list < self.id).sum()

        # TODO: make an array of parameters mimiquing the 3 index
        if not self.initialised:

            for i in range(0, len(self.agent_list)):
                placeholder_planes = self.opti.parameter(self.N, 3)
                self.planes_param.append(placeholder_planes)
                placeholder_states = self.opti.parameter(self.N, 2)
                self.states_param.append(placeholder_states)

            if aux == 0:
                self.planes = self.opti.variable(
                    3 * (self.N) * self.n_neighbours)  # theta111, theta121 ... , theta11N, theta12N
                self.opti.set_initial(self.planes, self.planes_fixed.flatten()) #TODO: for more than 2 robots we'll need to fix this optimisation

            tic = time.time()
            setting_time_ABC = time.time() - tic

            tic = time.time()
            self.ineq_constraints()
            setting_time_ineq = time.time() - tic

            tic = time.time()
            self.eq_constraints(x0)
            setting_time_eq = time.time() - tic

            tic = time.time()
            J = self.cost()
            self.opti.minimize(J)
            setting_time_cost = time.time() - tic
            self.initialised = True

        self.update_parameters(x0)

        # tic = time.time()
        p_opts = {"ipopt.print_level":0, "ipopt.sb":"yes", "print_time":0}
        s_opts = {"max_iter": 1}
        self.opti.solver("ipopt", p_opts,
                    s_opts)
        tic = time.time()

        # self.opti.solver("cplex")
        self.settingTime = time.time() - startTimer

        try:
            sol = self.opti.solve()
            x = sol.value(self.x)
            u = sol.value(self.u)
            try:
                planes = np.reshape(sol.value(self.planes), (self.N, 3, -1))

            except:
                planes = None
                lambdas = []
                for cts in self.planes_constraints:
                    lambdas.append(sol.value(self.opti.dual(cts)))

        except Exception as e:
            print(e)
            # print("not solved")
            x = self.opti.debug.value(self.x)
            u = self.opti.debug.value(self.u)
            try:
                planes = np.reshape(self.opti.debug.value(self.planes), (self.N, 3, -1))

            except:
                planes = None

        if self.flag_lambdas:
            lambdas = []
            for cts in self.planes_constraints:
                lambdas.append(self.opti.debug.value(self.opti.dual(cts)))
            self.flag_lambdas = False
            self.planes_constraints = []

        opti_time = time.time() - tic

        idx = np.arange(0, 9)
        d = 2
        for i in range(1, self.N + 1):
            aux = np.arange(0, 9) + i * self.n_exp
            idx = np.hstack((idx, aux))

        self.xPred = np.reshape((x)[idx], (self.N + 1, self.n_s))
        self.uPred = np.reshape(u, (self.N, d))

        self.solverTime = time.time() - startTimer

        # print("---------------------------------------------")
        # print("totalTime")
        # print(self.solverTime)
        # print("ABC")
        # print(setting_time_ABC)
        # print("Ineq constraint")
        # print(setting_time_ineq)
        # print("Eq constraint")
        # print(setting_time_eq)
        # print("setting_time_cost")
        # print(setting_time_cost)
        # print("opti_time")
        # print(opti_time)
        # print("---------------------------------------------")

        #TODO: check how to retrieve feasibility conditions
        return True, x, planes
