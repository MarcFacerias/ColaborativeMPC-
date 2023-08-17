#!/usr/bin/env python
from plan_lib.utilities import curvature, get_ey
from casadi import *

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class base_nl_constr:
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given ini_xPredicted computes the control action
    """
    def __init__(self, Q, Qs, R, dR, N, dt, map, id, model_param = None, sys_lim = None):
        # system parameters:
        self.n_s = 9
        self.n_u = 2
        self.n_sl = 3
        self.id = id
        self.initialised = False # Flag to instantiate the optimisation problem
        self.g = 9.81

        # Vehicle parameters:
        if model_param is None:
            self.lf = 0.125
            self.lr = 0.125
            self.m  = 1.98
            self.I  = 0.06
            self.Cf = 60.0
            self.Cr = 60.0
            self.mu = 0.05

        else:
            self.lf = model_param["lf"]
            self.lr = model_param["lr"]
            self.m  = model_param["m"]
            self.I  = model_param["I"]
            self.Cf = model_param["Cf"]
            self.Cr = model_param["Cr"]
            self.mu = model_param["mu"]

        if sys_lim is None:
            self.vx_ref = 3.0
            self.max_vel = 5
            self.min_vel = 0.2
            self.max_rs = 0.45
            self.max_ls = 0.45
            self.max_ac = 4.0
            self.max_dc = 3.0
            self.sm     = 0.9
            self.dth    = 0.25
            self.LPV_flag = True
        else:
            self.vx_ref  = sys_lim["vx_ref"]
            self.max_vel = sys_lim["max_vel"]
            self.min_vel = sys_lim["min_vel"]
            self.max_rs  = sys_lim["max_rs"]
            self.max_ls  = sys_lim["max_ls"]
            self.max_ac  = sys_lim["max_ac"]
            self.max_dc  = sys_lim["max_dc"]
            self.sm      = sys_lim["sm"]
            self.dth     = sys_lim["min_dist"]
            self.LPV_flag = sys_lim["LPV"]

        # declaration of the optimisation variables
        self.opti = casadi.Opti()
        self.x = self.opti.variable(N+1,self.n_s) # x0, ..., xN
        self.u  = self.opti.variable(N,2)  # u1, ..., uN
        self.du = self.opti.variable(N,2)  # du1, ..., duN
        self.slack_agent = self.opti.variable(N,2)  # s00, s10, s20, s30, ..., s0N, s1N, s2N, s3N,
        self.ey_ub = self.opti.parameter(N)
        self.ey_lb = self.opti.parameter(N)

        # sistem matrices delaration
        self.A    = []
        self.B    = []
        self.C    = []

        if Q.shape[0] == self.n_s:
            self.Q    = Q
        else:
            msg = 'Q has not the correct shape!, defaulting to identity of ' + str(self.n_s)
            warnings.warn(msg)
            self.Q = np.eye(self.n_s)

        if Qs.shape[0] == self.n_sl:
            self.model_slack   = Qs[0,0]
            self.control_slack = Qs[1,1]
            self.obs_slack     = Qs[2,2]
        else:
            msg = "Qs has not the correct shape!, defaulting to inf of " + str(self.n_sl)
            warnings.warn(msg)
            self.model_slack   = 1000000
            self.control_slack = 1000000
            self.obs_slack     = 1000000

        if R.shape[0] == self.n_u:
            self.R   = R
        else:
            msg = "Qs has not the correct shape!, defaulting to identity of " + str(self.n_u)
            warnings.warn(msg)
            self.R = np.eye(self.n_u)

        if dR.shape[0] == self.n_u:
            self.dR   = dR
        else:
            msg = "Qs has not the correct shape!, defaulting to identity of " + str(self.n_u)
            warnings.warn(msg)
            self.dR = np.eye(self.n_u)

        self.N  = N
        self.dt = dt              # Sample time
        self.map = map            # Used for getting the road curvature
        self.pose_param = []      # neighbour x,y
        self.states_param = []      # neighbour x,y
        self.du_param = []        # neighbour du
        self.u_param  = []
        self.s_agent_param = []   # neighbour states slack
        self.param_slack_dis = [] # distance constraint slack

        # parameters
        self.initial_states = self.opti.parameter(self.n_s)
        self.initial_u = self.opti.parameter(self.n_u)


        if self.LPV_flag:
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
            self.A41 = self.opti.parameter(self.N)
            self.A42 = self.opti.parameter(self.N)

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
        
        else:
            self.low_vel = self.opti.parameter()
            self.cur = self.opti.parameter(self.N)

    def LPV_model(self):
        # add the model
        for j in range(1, self.N + 1):
            mod_prev = (j - 1)

            # vx
            self.opti.subject_to(
                self.x[j, 0] == self.x[mod_prev, 0] + (
                        -self.mu * self.x[mod_prev, 0] + self.A12[j - 1] * self.x[mod_prev, 1] + self.A13[j - 1] *
                        self.x[mod_prev, 2] + self.B11[j - 1] * self.u[mod_prev, 0] + self.u[mod_prev, 1]) * self.dt
            )

            # vy
            self.opti.subject_to(
                self.x[j, 1] == self.x[mod_prev, 1] + (
                        self.A22[j - 1] * self.x[mod_prev, 1] + self.A23[j - 1] * self.x[mod_prev, 2] + self.B21[
                    j - 1] * self.u[mod_prev, 0]) * self.dt
            )

            # wz
            self.opti.subject_to(
                self.x[j, 2] == self.x[mod_prev, 2] + (
                        self.A32[j - 1] * self.x[mod_prev, 1] + self.A33[j - 1] * self.x[mod_prev, 2] + self.B31[
                    j - 1] * self.u[mod_prev, 0]) * self.dt
            )

            # ey

            self.opti.subject_to(
                self.x[j, 3] == self.x[mod_prev, 3] + (
                        self.A41[j - 1] * self.x[mod_prev, 0] + self.A42[j - 1] * self.x[mod_prev, 1]) * self.dt
            )

            # epsi

            self.opti.subject_to(
                self.x[j, 4] == self.x[mod_prev, 4] + (
                        self.A51[j - 1] * self.x[mod_prev, 0] + self.A52[j - 1] * self.x[mod_prev, 1] + self.x[
                    mod_prev, 2]) * self.dt
            )

            # theta

            self.opti.subject_to(
                self.x[j, 5] == self.x[mod_prev, 5] + (self.x[mod_prev, 2]) * self.dt
            )

            # s

            self.opti.subject_to(
                self.x[j, 6] == self.x[mod_prev, 6] + (
                        self.A61[j - 1] * self.x[mod_prev, 0] + self.A62[j - 1] * self.x[mod_prev, 1]) * self.dt
            )

            # x

            self.opti.subject_to(
                self.x[j, 7] == self.x[mod_prev, 7] + (
                        self.A81[j - 1] * self.x[mod_prev, 0] + self.A82[j - 1] * self.x[mod_prev, 1]) * self.dt
            )

            # y

            self.opti.subject_to(
                self.x[j, 8] == self.x[mod_prev, 8] + (
                        self.A91[j - 1] * self.x[mod_prev, 0] + self.A92[j - 1] * self.x[mod_prev, 1]) * self.dt
            )

    def NL_model(self):

        # add the model
        for j in range(1, self.N + 1):
            mod_prev = (j - 1)

            self.opti.subject_to(
                    self.x[j,0] == self.x[mod_prev,0] + (
                            -self.mu*self.x[mod_prev,0] +
                            self.low_vel*((np.sin(self.u[mod_prev,0]) * self.Cf) / (self.m * self.x[mod_prev,0])) *self.x[mod_prev,1] +
                            self.low_vel*((np.sin(self.u[mod_prev,0]) * self.Cf * self.lf) / (self.m * self.x[mod_prev,0]) + self.x[mod_prev,1])*self.x[mod_prev,2] +
                            self.low_vel*(-(np.sin(self.u[mod_prev,0]) * self.Cf) / self.m)*self.u[mod_prev,0] +
                            self.u[mod_prev,1])*self.dt
                )
            # --------------------------------------------------------------------------------------------------------------------------
            # # vy

            self.opti.subject_to(
                self.x[j,1] == self.x[mod_prev,1] + (
                        self.low_vel*(-(self.Cr + self.Cf * np.cos(self.u[mod_prev, 0])) / (self.m * self.x[mod_prev, 0]))*self.x[mod_prev,1] +
                        self.low_vel*(-(self.lf * self.Cf * np.cos(self.u[mod_prev, 0]) - self.lr * self.Cr) / (self.m * self.x[mod_prev, 0]) - self.x[mod_prev, 0])*self.x[mod_prev,2] +
                        ((np.cos(self.u[mod_prev, 0]) * self.Cf) / self.m)*self.u[mod_prev,0])*self.dt
            )

            # --------------------------------------------------------------------------------------------------------------------------
            # # wz

            self.opti.subject_to(
                self.x[j,2] == self.x[mod_prev,2] + (
                        self.low_vel*(-(self.lf * self.Cf * np.cos(self.u[mod_prev, 0]) - self.lr * self.Cr) / (self.I * self.x[mod_prev, 0]))*self.x[mod_prev,1] +
                        self.low_vel*(-(self.lf * self.lf * self.Cf * np.cos(self.u[mod_prev, 0]) + self.lr * self.lr * self.Cr) / ( self.I * self.x[mod_prev, 0]))*self.x[mod_prev,2] +
                        ((self.lf * self.Cf * np.cos(self.u[mod_prev, 0])) / self.I)*self.u[mod_prev,0])*self.dt
            )

            # --------------------------------------------------------------------------------------------------------------------------
            # ey

            self.opti.subject_to(
                self.x[j,3] == self.x[mod_prev,3] + (sin(self.x[mod_prev,4])*self.x[mod_prev,0] +
                                                     cos(self.x[mod_prev,4])*self.x[mod_prev,1])*self.dt
            )
            # --------------------------------------------------------------------------------------------------------------------------
            # epsi

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
            self.opti.subject_to(
                self.x[j,6] == self.x[mod_prev,6] + (
                        (np.cos(self.x[mod_prev, 4]) / (1 - self.x[mod_prev, 3] * self.cur[j-1]))*self.x[mod_prev,0] +
                        (-np.sin(self.x[mod_prev, 4]) / (1 - self.x[mod_prev, 3] * self.cur[j-1]))*self.x[mod_prev,1])*self.dt
            )
            # --------------------------------------------------------------------------------------------------------------------------
            self.opti.subject_to(
                self.x[j,7] == self.x[mod_prev,7] + (
                        (np.cos(self.x[mod_prev, 5]))*self.x[mod_prev,0] +
                        (-np.sin(self.x[mod_prev, 5]))*self.x[mod_prev,1])*self.dt
            )
            # --------------------------------------------------------------------------------------------------------------------------
            self.opti.subject_to(
                self.x[j,8] == self.x[mod_prev,8] + ((np.sin(self.x[mod_prev, 5])) * self.x[mod_prev,0] + (np.cos(self.x[mod_prev, 5])) * self.x[mod_prev,1])*self.dt
            )

    def eq_constraints(self):

        # set initial states
        for i in range(0, self.n_s):
            self.opti.subject_to(
                self.x[0, i] == self.initial_states[i]
            )
        self.opti.subject_to(self.u[0, 0] == self.initial_u[0] + self.du[0, 0])
        self.opti.subject_to(self.u[0, 1] == self.initial_u[1] + self.du[0, 1])

        if self.LPV_flag:
            self.LPV_model()

        else:
            self.NL_model()

        for j in range(1, self.N-1):
                self.opti.subject_to(self.u[j, 0] == self.u[j-1, 0] + self.du[j, 0])
                self.opti.subject_to(self.u[j, 1] == self.u[j-1, 1] + self.du[j, 1])

    def update_parameters(self,states,u):

        # Update the parametric expresions inside the solver

        ey = get_ey(states[:, 6], self.map) # update lateral error limits

        try:
            self.opti.set_value(self.ey_ub*self.sm, ey*self.sm)
            self.opti.set_value(self.ey_lb*self.sm, -ey*self.sm)
        except:
            self.opti.set_value(self.ey_ub*self.sm, ey[1:]*self.sm)
            self.opti.set_value(self.ey_lb*self.sm, -ey[1:]*self.sm)

        # update model variables
        for j in range (0,self.N):

            if self.LPV_flag:

                vx = states[j, 0]
                vy = states[j, 1]
                ey = states[j, 3]
                epsi = states[j, 4]
                theta = states[j, 5]
                s = states[j, 6]

                cur = curvature(s, self.map)
                delta = u[j, 0]  # EA: steering angle at K-1

                if vx < 0.2:

                    # low vel model: straight line .
                    self.opti.set_value(self.A12[j], 0)
                    self.opti.set_value(self.A13[j], 0)

                    self.opti.set_value(self.A22[j], 0)
                    self.opti.set_value(self.A23[j], 0)

                    self.opti.set_value(self.A32[j], 0)
                    self.opti.set_value(self.A33[j], 0)

                    self.opti.set_value(self.B11[j], 0)

                else:

                    # standard model
                    self.opti.set_value(self.A12[j], (np.sin(delta) * self.Cf) / (self.m * vx))
                    self.opti.set_value(self.A13[j], (np.sin(delta) * self.Cf * self.lf) / (self.m * vx) + vy)

                    self.opti.set_value(self.A22[j], -(self.Cr + self.Cf * np.cos(delta)) / (self.m * vx))
                    self.opti.set_value(self.A23[j],
                                        -(self.lf * self.Cf * np.cos(delta) - self.lr * self.Cr) / (self.m * vx) - vx)

                    self.opti.set_value(self.A32[j],
                                        -(self.lf * self.Cf * np.cos(delta) - self.lr * self.Cr) / (self.I * vx))
                    self.opti.set_value(self.A33[j],
                                        -(self.lf * self.lf * self.Cf * np.cos(delta) + self.lr * self.lr * self.Cr) / (
                                                self.I * vx))

                    self.opti.set_value(self.B11[j], 1)


                self.opti.set_value(self.A41[j], np.sin(epsi))
                self.opti.set_value(self.A42[j], np.cos(epsi))

                self.opti.set_value(self.A51[j], (1 / (1 - ey * cur)) * (-np.cos(epsi) * cur))
                self.opti.set_value(self.A52[j], (1 / (1 - ey * cur)) * (np.sin(epsi) * cur))

                self.opti.set_value(self.A61[j], np.cos(epsi) / (1 - ey * cur))
                self.opti.set_value(self.A62[j], -np.sin(epsi) / (1 - ey * cur))

                self.opti.set_value(self.A81[j], np.cos(theta))
                self.opti.set_value(self.A82[j], -np.sin(theta))

                self.opti.set_value(self.A91[j], np.sin(theta))
                self.opti.set_value(self.A92[j], np.cos(theta))

                self.opti.set_value(self.B21[j], (np.cos(delta) * self.Cf) / self.m)
                self.opti.set_value(self.B31[j], (self.lf * self.Cf * np.cos(delta)) / self.I)
            
            else:

                cur = curvature(states[j, 6], self.map)
                self.opti.set_value(self.cur[j], cur)

                if np.any( states[j, 0] < 0.2):
                    # low vel model: straight line .
                    self.opti.set_value(self.low_vel, 0)

                else:
                    self.opti.set_value(self.low_vel, 1)

            for i, el in enumerate(self.agent_list):

                self.opti.set_value(self.pose_param[i][j,0], self.states_fixed[j,i,0])
                self.opti.set_value(self.pose_param[i][j,1], self.states_fixed[j,i,1])

        # update neighbouring agents
        for i, el in enumerate(self.agent_list):
            self.opti.set_value(self.pose_param[i][-1, 0], self.states_fixed[-1, i, 0])
            self.opti.set_value(self.pose_param[i][-1, 1], self.states_fixed[-1, i, 1])

        # update initial states
        for i in range(0, self.n_s):
            self.opti.set_value(self.initial_states[i] ,states[0,i])
