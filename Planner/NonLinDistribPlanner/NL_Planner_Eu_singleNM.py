#!/usr/bin/env python
import time
from casadi import *
import numpy as np
from utilities import Curvature, get_ey

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


class Planner_Eud:
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given ini_xPredicted computes the control action
    """

    def __init__(self, Q, Qs, R, N, dt, map, id, dth, model_param=None, sys_lim=None):

        # system parameters:
        self.n_s = 9
        self.n_u = 2
        self.n_sl = 3
        self.dth = dth
        self.id = id
        self.initialised = False  # Flag to instantiate the optimisation problem
        self.g = 9.81

        # Vehicle parameters:
        if model_param is None:
            self.lf = 0.125
            self.lr = 0.125
            self.m = 1.98
            self.I = 0.06
            self.Cf = 60.0
            self.Cr = 60.0
            self.mu = 0.05

        else:
            self.lf = model_param["lf"]
            self.lr = model_param["lr"]
            self.m = model_param["m"]
            self.I = model_param["I"]
            self.Cf = model_param["Cf"]
            self.Cr = model_param["Cr"]
            self.mu = model_param["mu"]

        if sys_lim is None:
            self.vx_ref = 4.5
            self.max_vel = 5
            self.min_vel = 0.2
            self.max_rs = 0.45
            self.max_ls = 0.45
            self.max_ac = 4.0
            self.max_dc = 3.0
            self.sm = 0.9

        else:
            self.vx_ref = sys_lim["vx_ref"]
            self.max_vel = sys_lim["max_vel"]
            self.min_vel = sys_lim["min_vel"]
            self.max_rs = sys_lim["max_rs"]
            self.max_ls = sys_lim["max_ls"]
            self.max_ac = sys_lim["max_ac"]
            self.max_dc = sys_lim["max_dc"]
            self.sm = sys_lim["sm"]

        # declaration of the optimisation variables
        self.opti = casadi.Opti()
        self.x = self.opti.variable((N + 1), self.n_s)  # x0, ..., xN
        self.u = self.opti.variable(N, 2)  # u1, ..., uN
        self.du = self.opti.variable(N, 2)  # du1, ..., duN
        self.slack_agent = self.opti.variable(N, 1)  # s00, s10, s20, s30, ..., s0N, s1N, s2N, s3N,
        self.ey_ub = self.opti.parameter(N)
        self.ey_lb = self.opti.parameter(N)

        # sistem matrices delaration
        self.A = []
        self.B = []
        self.C = []

        if Q.shape[0] == self.n_s:
            self.Q = Q
        else:
            msg = 'Q has not the correct shape!, defaulting to identity of ' + str(self.n_s)
            warnings.warn(msg)
            self.Q = np.eye(self.n_s)

        if Qs.shape[0] == self.n_sl:
            self.model_slack = Qs[0, 0]
            self.control_slack = Qs[1, 1]
            self.obs_slack = Qs[2, 2]
        else:
            msg = "Qs has not the correct shape!, defaulting to inf of " + str(self.n_sl)
            warnings.warn(msg)
            self.model_slack = 1000000
            self.control_slack = 1000000
            self.obs_slack = 1000000

        if R.shape[0] == self.n_u:
            self.R = R
        else:
            msg = "Qs has not the correct shape!, defaulting to identity of " + str(self.n_u)
            warnings.warn(msg)
            self.R = np.eye(self.n_u)

        self.N = N
        self.dt = dt  # Sample time
        self.map = map[0]  # Used for getting the road curvature
        self.pose_param = []  # neighbour x,y
        self.states_param = []  # neighbour x,y
        self.du_param = []  # neighbour du
        self.s_agent_param = []  # neighbour states slack
        self.param_slack_dis = []  # distance constraint slack

        # parameters
        self.initial_states = self.opti.parameter(self.n_s)
        self.initial_u      = self.opti.parameter(self.n_u)
        self.cur            = self.opti.parameter(self.N)

        # LPV placeholders
        # vx
        self.A12 = [""]*(self.N+1)
        self.A13 = [""]*(self.N+1)
        self.B11 = [""]*(self.N+1)

        # vy
        self.A22 = [""]*(self.N+1)
        self.A23 = [""]*(self.N+1)
        self.B21 = [""]*(self.N+1)

        # wz
        self.A32 = [""]*(self.N+1)
        self.A33 = [""]*(self.N+1)
        self.B31 = [""]*(self.N+1)

        # ey
        self.A44 = [""]*(self.N+1)

        # epsi
        self.A51 = [""]*(self.N+1)
        self.A52 = [""]*(self.N+1)

        # s
        self.A61 = [""]*(self.N+1)
        self.A62 = [""]*(self.N+1)

        # x
        self.A81 = [""]*(self.N+1)
        self.A82 = [""]*(self.N+1)

        # y
        self.A91 = [""]*(self.N+1)
        self.A92 = [""]*(self.N+1)

    def cost(self):

        # parametric generation cost function
        J = 0
        for j in range(1, self.N + 1):
            mod_u = (j - 1)

            # cost asociated to the current agent
            J += self.Q[0, 0] * (self.x[j, 0] - self.vx_ref) ** 2 + self.Q[1, 1] * self.x[j, 1] ** 2 + self.Q[2, 2] * \
                 self.x[j, 2] ** 2 + \
                 self.Q[3, 3] * self.x[j, 3] ** 2 + self.Q[4, 4] * self.x[j, 4] ** 2 + self.Q[5, 5] * self.x[
                     j, 5] ** 2 + self.Q[6, 6] * self.x[j, 6] ** 2 + self.Q[7, 7] * self.x[j, 7] ** 2 + \
                 self.Q[8, 8] * self.x[j, 8] ** 2 + self.R[0, 0] * self.du[mod_u, 0] ** 2 + self.R[1, 1] * self.du[
                     mod_u, 1] ** 2 + \
                 self.model_slack * (self.slack_agent[j - 1, 0] ** 2)

        return J

    def ineq_constraints(self):
        # parametric definition of inequality constraints along the horizon

        for j in range(1, self.N + 1):
            # bound linear velocity
            self.opti.subject_to(
                self.opti.bounded(self.min_vel, self.x[j, 0], self.max_vel))
            # bound lateral error acording to the part of the track being traversed
            self.opti.subject_to(
                self.opti.bounded(self.ey_lb[j - 1], self.x[j, 3] + 0*self.slack_agent[j - 1, 0], self.ey_ub[j - 1]))

            # bound control actions
            self.opti.subject_to(self.opti.bounded(-self.max_ls, self.u[j - 1, 0], self.max_rs))
            self.opti.subject_to(self.opti.bounded(-self.max_dc, self.u[j - 1, 1], self.max_ac))

    def eq_constraints(self):

        # set initial states
        for i in range(0, self.n_s):
            self.opti.subject_to(
                self.x[0, i] == self.initial_states[i]
            )
        self.opti.subject_to(self.u[0, 0] == self.initial_u[0] + self.du[0, 0])
        self.opti.subject_to(self.u[0, 1] == self.initial_u[1] + self.du[0, 1])

        # add the model
        for j in range(1, self.N + 1):
            mod_prev = (j - 1)
            # self.A12[j-1] = (np.sin(self.u[mod_prev,0]) * self.Cf) / (self.m * self.x[mod_prev,0])
            # self.A13[j-1] = (np.sin(self.u[mod_prev,0]) * self.Cf * self.lf) / (self.m * self.x[mod_prev,0]) + self.x[mod_prev,1]
            # self.B11[j-1] = -(np.sin(self.u[mod_prev,0]) * self.Cf) / self.m
            # # vx
            # self.opti.subject_to(
            #     self.x[j,0] == self.x[mod_prev,0] + (-self.mu*self.x[mod_prev,0] + self.A12[j-1]*self.x[mod_prev,1] + self.A13[j-1]*self.x[mod_prev,2] + self.B11[j-1]*self.u[mod_prev,0] + self.u[mod_prev,1])*self.dt
            # )
            self.opti.subject_to(
                    self.x[j,0] == self.x[mod_prev,0] + (
                            -self.mu*self.x[mod_prev,0] +
                            ((np.sin(self.u[mod_prev,0]) * self.Cf) / (self.m * self.x[mod_prev,0])) *self.x[mod_prev,1] +
                            ((np.sin(self.u[mod_prev,0]) * self.Cf * self.lf) / (self.m * self.x[mod_prev,0]) + self.x[mod_prev,1])*self.x[mod_prev,2] +
                            (-(np.sin(self.u[mod_prev,0]) * self.Cf) / self.m)*self.u[mod_prev,0] +
                            self.u[mod_prev,1])*self.dt
                )
            # --------------------------------------------------------------------------------------------------------------------------

            # self.A22[j - 1] = -(self.Cr + self.Cf * np.cos(self.u[mod_prev, 0])) / (self.m * self.x[mod_prev, 0])
            # self.A23[j - 1] = -(self.lf * self.Cf * np.cos(self.u[mod_prev, 0]) - self.lr * self.Cr) / (
            #             self.m * self.x[mod_prev, 0]) - self.x[mod_prev, 0]
            # self.B21[j - 1] = -(np.sin(self.u[mod_prev, 0]) * self.Cf) / self.m
            #
            # # vy
            # self.opti.subject_to(
            #     self.x[j,1] == self.x[mod_prev,1] + (self.A22[j-1]*self.x[mod_prev,1] + self.A23[j-1]*self.x[mod_prev,2] + self.B21[j-1]*self.u[mod_prev,0])*self.dt
            # )

            self.opti.subject_to(
                self.x[j,1] == self.x[mod_prev,1] + (
                        (-(self.Cr + self.Cf * np.cos(self.u[mod_prev, 0])) / (self.m * self.x[mod_prev, 0]))*self.x[mod_prev,1] +
                        (-(self.lf * self.Cf * np.cos(self.u[mod_prev, 0]) - self.lr * self.Cr) / (self.m * self.x[mod_prev, 0]) - self.x[mod_prev, 0])*self.x[mod_prev,2] +
                        (-(np.sin(self.u[mod_prev, 0]) * self.Cf) / self.m)*self.u[mod_prev,0])*self.dt
            )

            # --------------------------------------------------------------------------------------------------------------------------
            # self.A32[j-1] = -(self.lf * self.Cf * np.cos(self.u[mod_prev, 0]) - self.lr * self.Cr) / (self.I * self.x[mod_prev, 0])
            # self.A33[j-1] = -(self.lf * self.lf * self.Cf * np.cos(self.u[mod_prev, 0]) + self.lr * self.lr * self.Cr) / ( self.I * self.x[mod_prev, 0])
            # self.B31[j-1] = (self.lf * self.Cf * np.cos(self.u[mod_prev, 0])) / self.I
            #
            # # wz
            # self.opti.subject_to(
            #     self.x[j,2] == self.x[mod_prev,2] + (self.A32[j-1]*self.x[mod_prev,1] + self.A33[j-1]*self.x[mod_prev,2] + self.B31[j-1]*self.u[mod_prev,0])*self.dt
            #
            # )

            self.opti.subject_to(
                self.x[j,2] == self.x[mod_prev,2] + (
                        (-(self.lf * self.Cf * np.cos(self.u[mod_prev, 0]) - self.lr * self.Cr) / (self.I * self.x[mod_prev, 0]))*self.x[mod_prev,1] +
                        (-(self.lf * self.lf * self.Cf * np.cos(self.u[mod_prev, 0]) + self.lr * self.lr * self.Cr) / ( self.I * self.x[mod_prev, 0]))*self.x[mod_prev,2] +
                        ((self.lf * self.Cf * np.cos(self.u[mod_prev, 0])) / self.I)*self.u[mod_prev,0])*self.dt
            )

            # --------------------------------------------------------------------------------------------------------------------------

            # ey

            # self.opti.subject_to(
            #     self.x[j,3] == self.x[mod_prev,3] + (self.x[mod_prev,1] + self.x[mod_prev, 0]*self.x[mod_prev,4])*self.dt
            # )
            self.opti.subject_to(
                self.x[j,3] == self.x[mod_prev,3] + (sin(self.x[mod_prev,4])*self.x[mod_prev,0] +
                                                     cos(self.x[mod_prev,4])*self.x[mod_prev,1])*self.dt
            )
            # --------------------------------------------------------------------------------------------------------------------------

            # epsi

            # self.A51[j-1] = (1 / (1 - self.x[mod_prev, 3] * self.cur[j-1])) * (-self.cur[j-1])
            # self.A52[j-1] = (1 / (1 - self.x[mod_prev, 3] * self.cur[j-1])) * (np.sin(self.x[mod_prev, 4]) * self.cur[j-1])
            #
            # self.opti.subject_to(
            #     self.x[j,4] == self.x[mod_prev,4] + (self.A51[j-1] * self.x[mod_prev,0] + self.A52[j-1] * self.x[mod_prev,1] + self.x[mod_prev,2])*self.dt
            #
            # )

            self.opti.subject_to(
                self.x[j,4] == self.x[mod_prev,4] + (
                        ((1 / (1 - self.x[mod_prev, 3] * self.cur[j-1])) * (-np.cos(self.x[mod_prev,4]) *self.cur[j-1])) * self.x[mod_prev,0] +
                        ((1 / (1 - self.x[mod_prev, 3] * self.cur[j-1])) * (np.sin(self.x[mod_prev,4]) * self.cur[j-1])) * self.x[mod_prev,1] +
                        self.x[mod_prev,2])*self.dt
            )

            # --------------------------------------------------------------------------------------------------------------------------

            # theta

            self.opti.subject_to(
                self.x[j, 5] == self.x[mod_prev, 5] + (self.x[mod_prev, 2]) * self.dt
            )

            # --------------------------------------------------------------------------------------------------------------------------
            # self.A61[j-1] =  np.cos(self.x[mod_prev, 4]) / (1 - self.x[mod_prev, 3] * self.cur[j-1])
            # self.A62[j-1] = -np.sin(self.x[mod_prev, 4]) / (1 - self.x[mod_prev, 3] * self.cur[j-1])
            #
            # # s
            #
            # self.opti.subject_to(
            #     self.x[j,6] == self.x[mod_prev,6] + (self.A61[j-1]*self.x[mod_prev,0] + self.A62[j-1]*self.x[mod_prev,1])*self.dt
            # )

            self.opti.subject_to(
                self.x[j,6] == self.x[mod_prev,6] + (
                        (np.cos(self.x[mod_prev, 4]) / (1 - self.x[mod_prev, 3] * self.cur[j-1]))*self.x[mod_prev,0] +
                        (-np.sin(self.x[mod_prev, 4]) / (1 - self.x[mod_prev, 3] * self.cur[j-1]))*self.x[mod_prev,1])*self.dt
            )
            # --------------------------------------------------------------------------------------------------------------------------

            # self.A81[j-1] = np.cos(self.x[mod_prev, 5])
            # self.A82[j-1] = -np.sin(self.x[mod_prev, 5])
            # # x
            #
            # self.opti.subject_to(
            #     self.x[j,7] == self.x[mod_prev,7] + (self.A81[j-1]*self.x[mod_prev,0] + self.A82[j-1]*self.x[mod_prev,1])*self.dt
            # )

            self.opti.subject_to(
                self.x[j,7] == self.x[mod_prev,7] + (
                        (np.cos(self.x[mod_prev, 5]))*self.x[mod_prev,0] +
                        (-np.sin(self.x[mod_prev, 5]))*self.x[mod_prev,1])*self.dt
            )
            # --------------------------------------------------------------------------------------------------------------------------

            # self.A91[j-1] = np.sin(self.x[mod_prev, 5])
            # self.A92[j-1] = np.cos(self.x[mod_prev, 5])
            # # y
            #
            # self.opti.subject_to(
            #     self.x[j,8] == self.x[mod_prev,8] + (self.A91[j-1] * self.x[mod_prev,0] + self.A92[j-1] * self.x[mod_prev,1])*self.dt
            # )

            self.opti.subject_to(
                self.x[j,8] == self.x[mod_prev,8] + ((np.sin(self.x[mod_prev, 5])) * self.x[mod_prev,0] + (np.cos(self.x[mod_prev, 5])) * self.x[mod_prev,1])*self.dt
            )

            # propagate the control ation
            if j < self.N:
                self.opti.subject_to(self.u[j, 0] == self.u[mod_prev, 0] + self.du[mod_prev, 0])
                self.opti.subject_to(self.u[j, 1] == self.u[mod_prev, 1] + self.du[mod_prev, 1])

    def update_parameters(self, states, u):

        # Update the parametric expresions inside the solver

        ey = get_ey(states[:, 6], self.map)  # update lateral error limits
        self.opti.set_value(self.initial_u, u[0, :])

        try:
            self.opti.set_value(self.ey_ub, ey * self.sm)
            self.opti.set_value(self.ey_lb, -ey * self.sm)
        except:
            self.opti.set_value(self.ey_ub, ey[1:] * self.sm)
            self.opti.set_value(self.ey_lb, -ey[1:] * self.sm)

        # update model variables
        for j in range(0, self.N):
            self.opti.set_value(self.cur[j], Curvature(states[j, 6], self.map))

        # update initial states
        for i in range(0, self.n_s):
            self.opti.set_value(self.initial_states[i], states[0, i])

    def solve(self, ini_xPredicted=None, Last_xPredicted=None, uPred=None, lambdas=None, x_agents=None, agents_id=None,
              data=None):
        """Computes control action
        Arguments:
            ini_xPredicted: current state predicted
            EA: Last_xPredicted: it is just used for the warm up
            EA: uPred: set of last predicted control inputs used for updating matrix A LPV
            EA: A_L, B_L ,C_L: Set of LPV matrices
        """
        startTimer = time.time()

        # if it's the first iteration we need to generate the system matrixes
        if not self.initialised:
            # store constraint and cost function
            self.ineq_constraints()
            self.eq_constraints()
            J = self.cost()
            self.opti.minimize(J)
            self.initialised = True

        ## set initial statse
        if not ini_xPredicted[0] is None:
            try:
                self.opti.set_initial(self.x, ini_xPredicted[0])
            except:
                self.opti.set_initial(self.x, np.vstack((ini_xPredicted[0],np.ones((1,9)))))

        else:
            self.opti.set_initial(self.x, np.ones((11,9)))
        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        # # set control actions
        if not uPred[0] is None:
            self.opti.set_initial(self.u, uPred[0])

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        self.update_parameters(ini_xPredicted[0],
                               uPred[0])  # update parameters in the control problem and run the optimisation

        p_opts = {"ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": 0}
        s_opts = {"max_iter": 10000000}
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

            slack_agent = sol.value(self.slack_agent)

            self.xPred = x  # retrieve states
            self.uPred = u  # retrieve control actions

            self.solverTime = time.time() - startTimer  # keep the solver

            # load the slack variables into the placeholder
            # print(slack_agent)
            data = [x, du, slack_agent]  # return the data

        except:
            return False, 0, 0

        return status, x, data  # the 0 is a placeholder to the generated planes, which do not exist here
