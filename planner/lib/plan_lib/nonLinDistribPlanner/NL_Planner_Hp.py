#!/usr/bin/env python
import time

from casadi import *

from plan_lib.planes import hyperplane_separator
from plan_lib.utilities import curvature, get_ey

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class PlannerHp:
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given x0 computes the control action
    """
    def __init__(self, Q, Qs, R, N, dt, map, id, dth, model_param = None, sys_lim = None):

        # Vehicle parameters:
        self.n_s = 9
        self.n_u = 2
        self.n_sl = 4
        self.dth = dth
        self.id = id
        self.initialised = False
        self.g  = 9.81

        # Vehicle parameters:
        if model_param is None:
            self.lf = 0.12
            self.lr = 0.14
            self.m  = 2.250
            self.I  = 0.06
            self.Cf = 60.0
            self.Cr = 60.0
            self.mu = 0.0

        else:
            self.lf = model_param["lf"]
            self.lr = model_param["lr"]
            self.m  = model_param["m"]
            self.I  = model_param["I"]
            self.Cf = model_param["Cf"]
            self.Cr = model_param["Cr"]
            self.mu = model_param["mu"]

        if sys_lim is None:
            self.vx_ref = 6.0
            self.max_vel = 10
            self.min_vel = 0.2
            self.max_rs = 0.45
            self.max_ls = 0.45
            self.max_ac = 8.0
            self.max_dc = 8.0
            self.sm     = 0.9

        else:
            self.vx_ref  = sys_lim["vx_ref"]
            self.max_vel = sys_lim["max_vel"]
            self.min_vel = sys_lim["min_vel"]
            self.max_rs  = sys_lim["max_rs"]
            self.max_ls  = sys_lim["max_ls"]
            self.max_ac  = sys_lim["max_ac"]
            self.max_dc  = sys_lim["max_dc"]
            self.sm      = sys_lim["sm"]

        self.opti = casadi.Opti()
        self.x = self.opti.variable(self.n_s*(N+1)) # x11, ... , x1N, s11, ... xTN
        self.u  = self.opti.variable(2 * (N))  # x11, ... , x1N, s11, ... xTN
        self.du = self.opti.variable(2 * (N))
        self.slack_agent = self.opti.variable(N,4)  # x11, ... , x1N, s11, ... xTN
        self.ey_ub = self.opti.parameter(N)
        self.ey_lb = self.opti.parameter(N)

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
            self.planes_slack  = Qs[3,3]
        else:
            msg = "Qs has not the correct shape!, defaulting to inf of " + str(self.n_sl)
            warnings.warn(msg)
            self.model_slack   = 1000000
            self.control_slack = 1000000
            self.obs_slack     = 1000000
            self.planes_slack  = 1000000

        if R.shape[0] == self.n_u:
            self.R   = R
        else:
            msg = "Qs has not the correct shape!, defaulting to identity of " + str(self.n_u)
            warnings.warn(msg)
            self.R = np.eye(self.n_u)

        self.N   = N
        self.dt  = dt                # Sample time 33 ms
        self.map = map              # Used for getting the road curvature
        self.planes_constraints = []

        # parameters
        self.initial_states = self.opti.parameter(self.n_s )
        self.planes_param = []
        self.pose_param = []
        self.du_param = []
        self.states_param = []
        self.s_agent_param = []
        self.slack_params_master = []
        self.slack_params_slave = []

        # LPV placeholders
        self.A12 = self.opti.parameter(self.N)
        self.A13 = self.opti.parameter(self.N)
        self.B11 = self.opti.parameter(self.N)

        # vy check
        self.A22 = self.opti.parameter(self.N)
        self.A23 = self.opti.parameter(self.N)
        self.B21 = self.opti.parameter(self.N)

        # wz check
        self.A32 = self.opti.parameter(self.N)
        self.A33 = self.opti.parameter(self.N)
        self.B31 = self.opti.parameter(self.N)

        # ey
        self.A41 = self.opti.parameter(self.N)
        self.A42 = self.opti.parameter(self.N)

        # epsi check
        self.A51 = self.opti.parameter(self.N)
        self.A52 = self.opti.parameter(self.N)

        # s check
        self.A61 = self.opti.parameter(self.N)
        self.A62 = self.opti.parameter(self.N)

        # x
        self.A81 = self.opti.parameter(self.N)
        self.A82 = self.opti.parameter(self.N)

        # y
        self.A91 = self.opti.parameter(self.N)
        self.A92 = self.opti.parameter(self.N)


    def cost(self):

        J  = 0
        for j in range (1,self.N+1):
            mod = j * self.n_s
            mod_u = (j-1) * self.n_u

            J += self.Q[0,0]*(self.x[0+mod] - self.vx_ref)**2 + self.Q[1,1]*self.x[1+mod]**2 + self.Q[2,2]*self.x[2+mod]**2 +\
                 self.Q[3,3]*self.x[3+mod]**2 + self.Q[4,4]*self.x[4+mod]**2 + self.Q[5,5]*self.x[5+mod]**2 + self.Q[6,6]*self.x[6+mod]**2 + self.Q[7,7]*self.x[7+mod]**2 \
                 + self.Q[8,8]*self.x[8+mod]**2 + self.R[0,0]*self.du[0+(mod_u)]**2 + self.R[1,1]*self.du[1+mod_u]**2 + \
                 self.model_slack*(self.slack_agent[j-1,0]**2 + self.slack_agent[j-1,1]**2)**2 + self.control_slack*(self.slack_agent[j-1,2]**2 + self.slack_agent[j-1,3]**2)

            it_s = 0
            it_m = 0

            for i, el in enumerate(self.agent_list):

                planes_idx = (j - 1) * 3 * self.aux + 3 * it_m

                J += self.Q[0,0] * (self.states_param[i][0 + mod] - self.vx_ref) ** 2 + self.Q[1,1] * self.states_param[i][1 + mod] ** 2 + self.Q[2,2] * self.states_param[i][2 + mod] ** 2 + \
                     self.Q[3,3] * self.states_param[i][3 + mod] ** 2 + self.Q[4,4] * self.states_param[i][4 + mod] ** 2 + self.Q[5,5] * self.states_param[i][5 + mod] ** 2 + self.Q[6,6] * self.states_param[i][6 + mod] ** 2 + \
                     self.Q[7,7] * self.states_param[i][7 + mod] ** 2 \
                     + self.R[0,0] * self.du_param[i][0 + mod_u] ** 2 + self.R[1,1] * self.du_param[i][1 + mod_u] ** 2 \
                     + self.model_slack * (self.s_agent_param[i][j-1,0] ** 2 + self.s_agent_param[i][j-1,1]**2) \
                     + self.control_slack * (self.s_agent_param[i][j-1,2]**2 + self.s_agent_param[i][j-1,3] ** 2) \
                     + self.planes_slack*(self.slack_params_master[i][j-1,self.id]**2) + self.planes_slack*(self.slack_params_slave[i][j-1,self.id]**2)

                if self.id < el:

                    J+= self.lambdas[i,j-1]*(-(self.slack_params_slave[i][j-1,self.id] + self.planes[planes_idx+0]*self.pose_param[i][j-1,0] +
                        self.planes[planes_idx+1]*self.pose_param[i][j-1,1] + self.planes[planes_idx+2] - self.dth/2 ) ) +\
                        self.planes_slack * (self.slack_planes_master[j-1,it_m] ** 2)
                    it_m += 1

                else:

                    J += self.planes_slack*(self.slack_planes_slave[j-1,it_s]**2)
                    it_s += 1

        return J

    def ineq_constraints(self):

        for j in range(1, self.N+1):
            mod = j * self.n_s
            mod_u = (j-1) * self.n_u

            self.opti.subject_to(self.opti.bounded(self.min_vel,self.x[0+mod] + self.slack_agent[j-1,0],self.max_vel))
            self.opti.subject_to(self.opti.bounded(self.ey_lb[j-1], self.x[4+mod] + self.slack_agent[j-1,1], self.ey_ub[j-1]))

            # bound control actions
            self.opti.subject_to(self.opti.bounded(-self.max_ls, self.u[0+mod_u] + self.slack_agent[j-1,2], self.max_rs))
            self.opti.subject_to(self.opti.bounded(-self.max_dc, self.u[1+mod_u] + self.slack_agent[j-1,3], self.max_ac))

            if j < self.N:

                self.opti.subject_to(self.u[0+mod_u +2] == self.u[0+mod_u] + self.du[0+mod_u])
                self.opti.subject_to(self.u[1+mod_u +2] == self.u[1+mod_u] + self.du[1+mod_u])

            #planes
            it_s = 0
            it_m = 0
            for i,el in enumerate(self.agent_list):

                planes_idx = (j-1)*3*self.aux + 3*it_m

                if self.id < el:

                    self.opti.subject_to( self.planes[planes_idx+0]*self.x[7+mod] + self.planes[planes_idx+1]*self.x[8+mod] + self.planes[planes_idx+2] + self.slack_planes_master[j-1,it_m] < -self.dth/2 )
                    self.opti.subject_to((self.planes[planes_idx + 0]**2 + self.planes[planes_idx + 1]**2) == 1.0)
                    it_m += 1

                else:

                    self.opti.subject_to((self.slack_planes_slave[j-1,it_s] + self.planes_param[i][j-1,0]*self.x[7+mod] + self.planes_param[i][j-1,1]*self.x[8+mod] + self.planes_param[i][j-1,2]) > self.dth/2)
                    it_s += 1
    def eq_constraints(self):

        # set initial states
        for i in range(0, self.n_s):

            self.opti.subject_to(
                self.x[i] == self.initial_states[i]
            )


        for j in range(1, self.N + 1):
            mod = j * self.n_s
            mod_prev = (j-1) * self.n_s
            mod_u    = (j-1) * self.n_u

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
                self.x[3+mod] == self.x[3+mod_prev] + (self.A41[j-1]*self.x[0+mod_prev] + self.A42[j-1]*self.x[1+mod_prev])*self.dt
            )

            # epsi

            self.opti.subject_to(
                self.x[4+mod] == self.x[4+mod_prev] + (-self.A51[j-1] * self.x[0+mod_prev] + self.A52[j-1] * self.x[1+mod_prev] + self.x[2+mod_prev])*self.dt
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

    def update_parameters(self,states,u):

        # set_planes fixed
        ey = get_ey(states[:, 6], self.map) # asume one limit for the whole horizon

        try:
            self.opti.set_value(self.ey_ub*self.sm, ey*self.sm)
            self.opti.set_value(self.ey_lb*self.sm, -ey*self.sm)
        except:
            self.opti.set_value(self.ey_ub*self.sm, ey[1:]*self.sm)
            self.opti.set_value(self.ey_lb*self.sm, -ey[1:]*self.sm)

        for j in range (0,self.N):

            vx = states[j, 0]
            vy = states[j, 1]
            ey = states[j, 3]
            epsi = states[j, 4]
            theta = states[j, 5]
            s = states[j, 6]

            cur = curvature(s, self.map)
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

                self.opti.set_value(self.planes_param[i][j,0], self.planes_fixed[j,0,i])
                self.opti.set_value(self.planes_param[i][j,1], self.planes_fixed[j,1,i])
                self.opti.set_value(self.planes_param[i][j,2], self.planes_fixed[j,2,i])

                self.opti.set_value(self.pose_param[i][j,0], self.states_fixed[j,i,0])
                self.opti.set_value(self.pose_param[i][j,1], self.states_fixed[j,i,1])

        for i, el in enumerate(self.agent_list):
            self.opti.set_value(self.pose_param[i][-1, 0], self.states_fixed[-1, i, 0])
            self.opti.set_value(self.pose_param[i][-1, 1], self.states_fixed[-1, i, 1])

        for i in range(0, self.n_s):
            self.opti.set_value(self.initial_states[i] ,states[0,i])



    def solve(self, ini_xPredicted = None, Last_xPredicted = None, uPred = None, lambdas = None, x_agents = None, planes_fixed = None, agents_id = None, data = None):
        """Computes control action
        Arguments:
            ini_xPredicted: current state positionsolve
            EA: Last_xPredicted: it is just used for the warm up
            EA: uPred: set of last predicted control inputs used for updating matrix A LPV
            EA: A_L, B_L ,C_L: Set of LPV matrices
        """
        startTimer              = time.time()

        if not self.initialised:

            self.agent_list = np.asarray(agents_id)
            self.aux = (self.agent_list > self.id).sum()

            self.n_neighbours = self.agent_list.size
            self.plane_comp = hyperplane_separator(self.n_neighbours, self.N)
            self.states_fixed = np.zeros(((self.N), self.n_neighbours, 3))
            self.lambdas = self.opti.parameter(self.n_neighbours, self.N)

            for i in range(0, len(self.agent_list)):
                placeholder_planes = self.opti.parameter(self.N, 3)
                self.planes_param.append(placeholder_planes)

                placeholder_pose = self.opti.parameter(self.N, 2)
                self.pose_param.append(placeholder_pose)

                placeholder_states = self.opti.parameter(self.n_s * (self.N + 1))
                self.states_param.append(placeholder_states)

                placeholder_u = self.opti.parameter(2 * (self.N))
                self.du_param.append(placeholder_u)

                placeholder_sa = self.opti.parameter(self.N,4) #we have 4 slack variables
                self.s_agent_param.append(placeholder_sa)

                placeholder_spm = self.opti.parameter((self.N), (self.n_neighbours+1))
                self.slack_params_master.append(placeholder_spm)

                placeholder_sps = self.opti.parameter((self.N), (self.n_neighbours+1))
                self.slack_params_slave.append(placeholder_sps)

            if self.aux != 0:
                self.planes = self.opti.variable(
                        3 * (self.N) * self.aux)  # theta111, theta121 ... , theta11N, theta12N

                self.planes_fixed = np.zeros(((self.N), self.aux, 3))

                self.slack_planes_master = self.opti.variable((self.N ) , self.aux)  # x11, ... , x1N, s11, ... xTN

            if self.aux != self.n_neighbours:
                self.slack_planes_slave = self.opti.variable((self.N ) , (self.n_neighbours-self.aux))  # x11, ... , x1N, s11, ... xTN

            self.ineq_constraints()
            self.eq_constraints()
            J = self.cost()
            self.opti.minimize(J)
            self.initialised = True

        if not Last_xPredicted is None:
            self.opti.set_initial(self.x,Last_xPredicted.flatten())

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        if not uPred is None:
            self.opti.set_initial(self.u,uPred.flatten())

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        if lambdas is None:
            self.opti.set_value(self.lambdas,np.ones((self.n_neighbours, self.N+1)))

        else:
            self.opti.set_value(self.lambdas,lambdas)

        # unpack data

        for i,agent in enumerate(data):

            self.opti.set_value(self.states_param[i], agent[0])
            self.opti.set_value(self.du_param[i], agent[1])
            self.opti.set_value(self.s_agent_param[i], agent[2])
            self.opti.set_value(self.slack_params_master[i], agent[3])
            self.opti.set_value(self.slack_params_slave[i], agent[4])

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        if x_agents is None:
            x_agents = np.zeros((self.N,self.n_neighbours,2))

        self.states_fixed = x_agents

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        if planes_fixed is None:

            self.planes_fixed = self.plane_comp.compute_hyperplane(x_agents, ini_xPredicted[:,[7, 8]], self.id, agents_id)

        else:
            self.planes_fixed = planes_fixed

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        try:
            self.opti.set_initial(self.slack_planes_master, self.slack_planes_master_old)
        except:
            pass

        try:
            self.opti.set_initial(self.slack_planes_slave, self.slack_planes_slave_old)
        except:
            pass
        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        self.update_parameters(ini_xPredicted,uPred)

        p_opts = {"ipopt.print_level":0, "ipopt.sb":"yes", "print_time":0}
        s_opts = {"max_iter": 1}
        self.opti.solver("ipopt", p_opts,
                    s_opts)

        self.settingTime = time.time() - startTimer

        try:
            sol = self.opti.solve()
            status = True
            x = sol.value(self.x)
            u = sol.value(self.u)
            du = sol.value(self.du)

            try:
                planes = np.reshape(sol.value(self.planes), (self.N, 3, -1))
            except:
                planes = self.planes_fixed
                lambdas = []
                for cts in self.planes_constraints:
                    lambdas.append(sol.value(self.opti.dual(cts)))

            # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

            if self.aux != 0:
                self.slack_planes_master_old = sol.value(self.slack_planes_master)


            if self.aux != self.n_neighbours:
                self.slack_planes_slave_old = sol.value(self.slack_planes_slave)

            # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

            try:
                slack_agent = sol.value(self.slack_agent)
            except:
                slack_agent = None

        except Exception as e:
            print(e)
            status = False
            x = self.opti.debug.value(self.x)
            u = self.opti.debug.value(self.u)
            du = self.opti.debug.value(self.du)

            try:
                planes = np.reshape(self.opti.debug.value(self.planes), (self.N, 3, -1))
            except:
                planes = self.planes_fixed

            # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

            if self.aux != self.n_neighbours:
                self.slack_planes_slave_old = self.opti.debug.value(self.slack_planes_slave)

            if self.aux != 0:
                self.slack_planes_master_old = self.opti.debug.value(self.slack_planes_master)

            # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

            try:
                slack_agent = self.opti.debug.value(self.slack_agent)
            except:
                slack_agent = None

        idx = np.arange(0, self.n_s)
        for i in range(1, self.N + 1):
            aux = np.arange(0, self.n_s) + i * self.n_s
            idx = np.hstack((idx, aux))

        self.xPred = np.reshape((x)[idx], (self.N + 1, self.n_s))
        self.uPred = np.reshape(u, (self.N, self.n_u))

        self.solverTime = time.time() - startTimer

        spm_data = np.zeros(((self.N),(self.n_neighbours+1)))
        sps_data = np.zeros(((self.N),(self.n_neighbours+1)))
        planes_std = np.zeros((self.N, 3, self.n_neighbours+1))

        tst = (self.id < self.agent_list)
        idx_master = np.where(tst == True)[0]
        idx_slave  = np.where(tst == False)[0]

        if self.aux != 0:

            planes_std[:, :, idx_master] = planes

            try:
                spm_data[:,idx_master] = self.slack_planes_master_old
            except ValueError:
                spm_data[:, idx_master] = self.slack_planes_master_old[:,np.newaxis]

        if self.aux != self.n_neighbours:
            try:
                spm_data[:,idx_slave] = self.slack_planes_slave_old
            except ValueError:
                spm_data[:, idx_slave] = self.slack_planes_slave_old[:,np.newaxis]

        data = [x,du,slack_agent,spm_data,sps_data]

        return status, x, planes_std, data
