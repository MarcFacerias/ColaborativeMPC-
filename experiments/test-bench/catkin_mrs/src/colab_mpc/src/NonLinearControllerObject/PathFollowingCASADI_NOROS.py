#!/usr/bin/env python
import time
from casadi import *
import numpy as np
from utilities import Curvature, GBELLMF

# TODO Arreglar els idx per a fer que es pugui fer la decomposicio

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class PathFollowingLPV_MPC:
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given x0 computes the control action
    """
    def __init__(self, Q, R, dR, N, vt, dt, map):

        # Vehicle parameters:
        self.n_s = 9
        self.n_agents = 1
        self.n_neighbours = 1
        self.slack = 2
        self.n_exp = self.n_s + self.slack# slack variables
        self.dth = 0.5
        self.lf = 0.12
        self.lr = 0.14
        self.m  = 2.250
        self.I  = 0.06
        self.Cf = 60.0
        self.Cr = 60.0
        self.mu = 0.1

        self.g  = 9.81

        self.max_vel = 10
        self.opti = casadi.Opti()
        self.x = self.opti.variable(self.n_exp*(N+1)) # x11, ... , x1N, s11, ... xTN
        self.u = self.opti.variable(2 * (N))  # x11, ... , x1N, s11, ... xTN
        self.planes = self.opti.variable(2 * (N+1) * self.n_neighbours)  # x11, ... , x1N, s11, ... xTN

        self.cx = 0

        self.A    = []
        self.B    = []
        self.C    = []
        self.N    = N
        self.n    = Q.shape[0]
        self.d    = R.shape[0]
        self.vt   = vt
        self.Q    = Q
        self.R    = R
        self.dR   = dR              # Slew rate
        self.LinPoints = np.zeros((self.N+2,self.n))
        self.dt = dt                # Sample time 33 ms
        self.map = map              # Used for getting the road curvature

        self.first_it = 1

    def cost(self):

        J = 0
        for j in range (1,self.N+1):
            mod = j * self.n_exp
            J += 120*self.x[0+mod]**2 + self.x[1+mod]**2 + self.x[2+mod]**2 +\
                 1500*self.x[3+mod]**2 + 70*self.x[4+mod]**2 + 100000000*(self.x[9+mod]**2 + self.x[10+mod]**2)\
                 + self.u[0+(mod-self.n_exp)]**2 + self.x[1+mod-self.n_exp]**2 - 600*self.x[0+mod]

            for i, el in enumerate(self.agent_list):
                # TODO: do indexing
                if self.id < el:
                    J+= self.lambdas[i,0,j]*(-(self.planes[i]*x[i] + self.planes[i]*x[i]  + self.planes[i] - self.dth ) + self.cx)
        return J

    def ineq_constraints(self):

        for j in range(1, self.N + 1):
            mod = j * self.n_exp
            self.opti.subject_to(self.min_vel <= self.x[0+mod] <=self.max_vel)
            self.opti.subject_to(-0.60 <= self.x[0+mod] + self.x[10+mod] <=0.60)

            self.opti.subject_to(-0.45 <= self.u[0+mod] <= 0.45)
            self.opti.subject_to(-8.0 <= self.u[1+mod] <= 8.0)

            #planes
            for i,el in enumerate(self.agent_list):
                # TODO: fix this index/loop
                planes_idx = 0
                if self.id < el:
                    self.opti.subject_to( self.planes[planes_idx+0]*self.x[10+mod] + self.planes[planes_idx+1]*self.x[11+mod] + self.planes[planes_idx+2] <= 0.5)
                    self.opti.subject_to( self.planes[planes_idx+0]*self.x[mod + 11 + i] + self.planes[planes_idx+1]*self.x[mod + 11 + i +1] + self.planes[planes_idx+2] >= 0.5)
                    self.opti.subject_to( sqrt(self.planes[planes_idx+0]**2 + self.planes[planes_idx+1]**2) == 1.0)


    def eq_constraints(self,states, agents):
        # model constants
        lf = self.lf
        lr = self.lr
        m  = self.m
        I  = self.I
        Cf = self.Cf
        Cr = self.Cr
        mu = self.mu

        # set initial states
        for i in range(0, 9):

            self.opti.subject_to(
                self.x[i] == states[0,i]
            )

        # set neighbouring vehicles states
        for i in range(0, 9):

            self.opti.subject_to(
                self.x[i] == states[0,i]
            )


        for j in range(1, self.N + 1):
            mod = j * self.n_exp
            mod_prev = (j-1) * self.n_exp
            s = states[j,6]

            cur = Curvature(s, self.map)

            A12 = (np.sin(self.u[0+mod_prev]) * Cf) / (m * self.x[0+mod_prev])
            A13 = (np.sin(self.u[0+mod_prev]) * Cf * lf) / (m * self.x[0+mod_prev]) + self.x[1+mod_prev]

            A22 = -(Cr + Cf * np.cos(self.u[0+mod_prev])) / (m * self.x[0+mod_prev])
            A23 = -(lf * Cf * np.cos(self.u[0+mod_prev]) - lr * Cr) / (m * self.x[0+mod_prev]) - self.x[1+mod_prev]

            A32 = -(lf * Cf * np.cos(self.u[0+mod_prev]) - lr * Cr) / (I * self.x[0+mod_prev])
            A33 = -(lf * lf * Cf * np.cos(self.u[0+mod_prev]) + lr * lr * Cr) / (I * self.x[0+mod_prev])

            B11 = -(np.sin(self.u[0+mod_prev]) * Cf) / m
            B12 = 1

            A11 = -mu

            A51 = (1 / (1 - self.x[1+mod_prev] * cur)) * (-cur)
            A52 = (1 / (1 - self.x[0+mod_prev] * cur)) * (np.sin( self.x[4+mod_prev]) * cur)

            A61 = np.cos( self.x[4+mod_prev]) / (1 - self.x[1+mod_prev] * cur)
            A62 = -np.sin( self.x[4+mod_prev]) / (1 - self.x[1+mod_prev] * cur)

            A7 = 1
            A8 = self.x[0+mod_prev]

            A81 = np.cos(self.x[5+mod_prev])
            A82 = -np.sin(self.x[5+mod_prev])

            A91 = np.sin(self.x[5+mod_prev])
            A92 = np.cos(self.x[5+mod_prev])

            B21 = (np.cos(self.u[0]) * Cf) / m
            B31 = (lf * Cf * np.cos(self.u[0+mod_prev])) / I

            # vx
            self.opti.subject_to(
                self.x[0+mod] == self.x[0+mod_prev] + (A11*self.x[0+mod_prev] + A12*self.x[1+mod_prev] + A13*self.x[2+mod_prev] + B11*self.u[0+mod_prev] + B12*self.u[1+mod_prev])*self.dt
            )

            # vy
            self.opti.subject_to(
                self.x[1+mod] == self.x[1+mod_prev] + (A22*self.x[1+mod_prev] + A23*self.x[2+mod_prev] + B21*self.u[0+mod_prev])*self.dt
            )

            # wz
            self.opti.subject_to(
                self.x[2+mod] == self.x[2+mod_prev] + (A32*self.x[1+mod_prev] + A33*self.x[2+mod_prev] + B31*self.u[0+mod_prev])*self.dt
            )

            # ey

            self.opti.subject_to(
                self.x[3+mod] == self.x[3+mod_prev] + (A7*self.x[1+mod_prev] + A8*self.x[4+mod_prev])*self.dt
            )

            # epsi

            self.opti.subject_to(
                self.x[4+mod] == self.x[4+mod_prev] + (A51 * self.x[0+mod_prev] + A52 * self.x[1+mod_prev])*self.dt
            )

            # theta

            self.opti.subject_to(
                self.x[5+mod] == self.x[5+mod_prev] + (self.x[3+mod_prev])*self.dt
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


    def solve(self, x0, Last_xPredicted, uPred, lambdas, x_agents, agents_id, pose):
        """Computes control action
        Arguments:
            x0: current state position
            EA: Last_xPredicted: it is just used for the warm up
            EA: uPred: set of last predicted control inputs used for updating matrix A LPV
            EA: A_L, B_L ,C_L: Set of LPV matrices
        """
        startTimer              = time.time()

        if lambdas is None:
            self.lambdas = np.zeros((self.n_agents, 1, self.N)) # TODO fix lambdas into n neighbours x H time

        else:
            self.lambdas = lambdas

        self.A, self.B, self.C = _EstimateABC(self, Last_xPredicted, uPred)
        J = self.cost()
        self.ineq_constraints()
        self.eq_constraints(pose,x_agents)
        p_opts = {"expand": True}
        s_opts = {"max_iter": 100}
        self.opti.solver("ipopt", p_opts,
                    s_opts)
        self.solverTime = startTimer - time.time()


def _EstimateABC(Controller, states, u):

    lf = Controller.lf
    lr = Controller.lr
    m = Controller.m
    I = Controller.I
    Cf = Controller.Cf
    Cr = Controller.Cr
    mu = Controller.mu

    Atv = []
    Btv = []
    Ctv = []

    for i in range(0, Controller.N):

        vx = states[i, 0]
        vy = states[i, 1]
        ey = states[i, 3]
        epsi = states[i, 4]
        theta = states[i, 5]
        s = states[i, 6]

        cur = Curvature(s, Controller.map)
        delta = u[i, 0]  # EA: steering angle at K-1

        if vx < 0:

            # low vel model: straight line .
            A12 = 0
            A13 = 0

            A22 = 0
            A23 = 0

            A32 = 0
            A33 = 0

            B11 = 0
            B12 = 1


        else:

            # standard model

            A12 = (np.sin(delta) * Cf) / (m * vx)
            A13 = (np.sin(delta) * Cf * lf) / (m * vx) + vy

            A22 = -(Cr + Cf * np.cos(delta)) / (m * vx)
            A23 = -(lf * Cf * np.cos(delta) - lr * Cr) / (m * vx) - vx

            A32 = -(lf * Cf * np.cos(delta) - lr * Cr) / (I * vx)
            A33 = -(lf * lf * Cf * np.cos(delta) + lr * lr * Cr) / (I * vx)

            B11 = -(np.sin(delta) * Cf) / m
            B12 = 1

        A11 = -mu

        A51 = (1 / (1 - ey * cur)) * (-cur)
        A52 = (1 / (1 - ey * cur)) * (np.sin(epsi) * cur)

        A61 = np.cos(epsi) / (1 - ey * cur)
        A62 = -np.sin(epsi) / (1 - ey * cur)

        A7 = 1
        A8 = vx

        A81 = np.cos(theta)
        A82 = -np.sin(theta)

        A91 = np.sin(theta)
        A92 = np.cos(theta)

        B21 = (np.cos(delta) * Cf) / m
        B31 = (lf * Cf * np.cos(delta)) / I

        Ai = np.array([[A11, A12, A13, 0., 0., 0., 0., 0., 0.],  # [vx]
                       [0., A22, A23, 0., 0., 0., 0., 0., 0.],  # [vy]
                       [0., A32, A33, 0., 0., 0., 0., 0., 0.],  # [wz]
                       [0, A7, 0, 0., A8, 0., 0., 0., 0.],  # [ey]
                       [A51, A52, 1., 0., 0., 0., 0., 0., 0.],  # [epsi]
                       [0, 0, 1., 0., 0., 0., 0., 0., 0.],  # [theta]
                       [A61, A62, 0, 0., 0., 0., 0., 0., 0.],  # [s]
                       [A81, A82, 0, 0., 0., 0., 0., 0., 0.],  # [x]
                       [A91, A92, 0, 0., 0., 0., 0., 0., 0.],  # [y]
                       ])

        Bi = np.array([[B11, B12],  # [delta, a]
                       [B21, 0],
                       [B31, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0]])

        Ci = np.array([[0],
                       [0],
                       [0],
                       [0],
                       [0],
                       [0],
                       [0],
                       [0],
                       [0]])

        Ai = np.eye(len(Ai)) + Controller.dt * Ai
        Bi = Controller.dt * Bi
        Ci = Controller.dt * Ci

        Atv.append(Ai)
        Btv.append(Bi)
        Ctv.append(Ci)

    return Atv, Btv, Ctv


