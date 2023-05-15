#!/usr/bin/env python
import time
from casadi import *
import numpy as np


np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class PathFollowingLPV_MPC:
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given x0 computes the control action
    """
    def __init__(self, Q, R, dR, N, vt, dt, map, Solver, steeringDelay, velocityDelay):

        # Vehicle parameters:
        self.n_s = 9
        self.n_agents = 1
        self.slack = 2
        self.n_exp = self.n_s + self.slack# slack variables

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
        return J

    def ineq_constraints(self):

        for j in range(1, self.N + 1):
            mod = j * self.n_exp
            self.opti.subject_to(self.min_vel <= self.x[0+mod] <=self.max_vel)
            self.opti.subject_to(-0.60 <= self.x[0+mod] + self.x[10+mod] <=0.60)

            self.opti.subject_to(-0.45 <= self.u[0+mod] <= 0.45)
            self.opti.subject_to(-8.0 <= self.u[1+mod] <= 8.0)

    def eq_constraints(self):

        for j in range(1, self.N + 1):
            mod = j * self.n_exp
            self.opti.subject_to( self.x[(j-1)*self.n_exp:j*(self.n_exp)-2]
                                  - self.A[j-1]*self.x[(j)*self.n_exp:(j+1)*(self.n_exp)-2]
                                  - self.B[j-1]**self.u[(j-1)*2:j*2] == 0 )


    def solve(self, x0, Last_xPredicted, uPred, NN_LPV_MPC, vel_ref, curv_ref, A_L, B_L ,C_L, first_it):
        """Computes control action
        Arguments:
            x0: current state position
            EA: Last_xPredicted: it is just used for the warm up
            EA: uPred: set of last predicted control inputs used for updating matrix A LPV
            EA: A_L, B_L ,C_L: Set of LPV matrices
        """
        startTimer              = time.time()

        self.A, self.B, self.C = _EstimateABC(self, Last_xPredicted, uPred)
        J = self.cost()
        self.ineq_constraints()
        self.eq_constraints()
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


