#!/usr/bin/env python
import time

from .base_nl import *

from plan_lib.planes import hyperplane_separator
from plan_lib.utilities import curvature, get_ey

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class PlannerHp(base_nl_constr):
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given x0 computes the control action
    """
    def __init__(self, Q, Qs, R, dR,N, dt, map, id, model_param = None, sys_lim = None):
        super().__init__(Q, Qs, R, dR, N, dt, map, id, model_param, sys_lim) # initialise the initialiser

        self.planes_constraints = []
        self.planes_param = []
        self.slack_params_master = []
        self.slack_params_slave = []
        self.planes_slack = 1000000

    def cost(self):

        J  = 0
        for j in range (1,self.N+1):

            J += (self.Q[0,0]*(self.x[j,0] - self.vx_ref)**2 + self.Q[1,1]*self.x[j,1]**2 +
                  self.Q[2,2]*self.x[j,2]**2 + self.Q[3,3]*self.x[j,3]**2 +
                  self.Q[4,4]*self.x[j,4]**2 + self.Q[5,5]*self.x[j,5]**2 +
                  self.Q[6,6]*self.x[j,6]**2 + self.Q[7,7]*self.x[j,7]**2 +\
                  self.Q[8,8]*self.x[j,8]**2 +
                  self.dR[0,0]*self.du[j-1,0]**2 + self.dR[1,1]*self.du[j-1,1]**2 +
                  self.R[0,0] * self.u[j-1,0] ** 2 + self.R[1,1] * self.u[j-1,1] ** 2 +
                  self.model_slack*(self.slack_agent[j-1,0]**2 + self.slack_agent[j-1,1]**2 ))

            it_s = 0
            it_m = 0

            for i, el in enumerate(self.agent_list):

                planes_idx = (j - 1) * 3 * self.aux + 3 * it_m

                J += (self.Q[0,0] * (self.states_param[i][j,0] - self.vx_ref)** 2+ self.Q[1,1] * self.states_param[i][j,1] ** 2 +
                      self.Q[2,2] * self.states_param[i][j,2] ** 2 + self.Q[3,3] * self.states_param[i][j,3] ** 2 +
                      self.Q[4,4] * self.states_param[i][j,4] ** 2 + self.Q[5,5] * self.states_param[i][j,5] ** 2 +
                      self.Q[6,6] * self.states_param[i][j,6] ** 2 + self.Q[7,7] * self.states_param[i][j,7] ** 2 +
                      self.Q[8,8] * self.states_param[i][j,8] ** 2 +
                      self.R[0,0] * self.u_param[i][j-1,0] ** 2 + self.R[1,1] * self.u_param[i][j-1,1] ** 2 +
                      self.dR[0,0]* self.du_param[i][j-1,0] ** 2 + self.dR[1,1]*self.du_param[i][j-1,1] ** 2  +
                      self.model_slack * (self.s_agent_param[i][j-1,0] ** 2 + self.s_agent_param[i][j-1,1]**2) +
                      self.planes_slack*(self.slack_params_master[i][j-1,self.id]**2) + self.planes_slack*(self.slack_params_slave[i][j-1,self.id]**2))

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

            self.opti.subject_to(self.opti.bounded(self.min_vel,self.x[j,0] + self.slack_agent[j-1,1], self.max_vel))
            self.opti.subject_to(self.opti.bounded(self.ey_lb[j-1], self.x[j,3] + self.slack_agent[j-1,0], self.ey_ub[j-1]))

            # bound control actions
            self.opti.subject_to(self.opti.bounded(-self.max_ls, self.u[j-1,0], self.max_rs))
            self.opti.subject_to(self.opti.bounded(-self.max_dc, self.u[j-1,1], self.max_ac))

            #planes
            it_s = 0
            it_m = 0
            for i,el in enumerate(self.agent_list):

                planes_idx = (j-1)*3*self.aux + 3*it_m

                if self.id < el:

                    self.opti.subject_to( self.planes[planes_idx+0]*self.x[j,7] + self.planes[planes_idx+1]*self.x[j,8] + self.planes[planes_idx+2] + self.slack_planes_master[j-1,it_m] < -self.dth/2 )
                    self.opti.subject_to((self.planes[planes_idx + 0]**2 + self.planes[planes_idx + 1]**2) == 1.0)
                    it_m += 1

                else:

                    self.opti.subject_to((self.slack_planes_slave[j-1,it_s] + self.planes_param[i][j-1,0]*self.x[7+mod] + self.planes_param[i][j-1,1]*self.x[8+mod] + self.planes_param[i][j-1,2]) > self.dth/2)
                    it_s += 1


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
