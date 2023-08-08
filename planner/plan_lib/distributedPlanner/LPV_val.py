
import numpy as np
from planner.plan_lin.utilities import curvature, get_ey

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class LPV_Model:
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given x0 computes the control action
    """
    def __init__(self, N, dt, map, model_param = None, sys_lim = None):

        self.n_s = 9 # states
        self.slack = 3 # slack
        self.n_u   = 2 # control actions
        self.xPred = np.ones((N+1,self.n_s))
        self.dt = dt                # Sample time 33 ms
        self.map = map              # Used for getting the road curvature
        self.N = N
        self.first_it = True
        # previous values initialisation
        self.OldSteering = [0.0]
        self.OldAccelera = [0.0]*int(1)

        # Vehicle parameters:
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
            self.vx_ref = 4.5
            self.max_vel = 5
            self.min_vel = 0.2
            self.max_rs = 0.45
            self.max_ls = 0.45
            self.max_ac = 4.0
            self.max_dc = 3.0
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


        # variable placeholders
        self.A = []
        self.B = []
        self.C = []
        self.G = []
        self.E = []
        self.L = []
        self.Eu =[]


    def sim(self, u):
        """Computes control action
        Arguments:
            x0: current state position
            EA: Last_xPredicted: it is just used for the warm up
            EA: uPred: set of last predicted control inputs used for updating matrix A LPV
            EA: A_L, B_L ,C_L: Set of LPV matrices
        """

        Last_xPredicted = np.copy(self.xPred)
        self.A, self.B, self.C, ey = _EstimateABC(self, Last_xPredicted, u)

        for idx in range(0,self.N):
            self.xPred[idx+1,:] = self.A[idx]@self.xPred[idx,:] + self.B[idx]@u[idx,:]

        self.xPred = np.vstack((np.delete(self.xPred, 0,0), np.ones((1,self.n_s))))

        return self.xPred

def _EstimateABC(Controller,states, u):

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

        ey_hor = get_ey(states[:, 6], Controller.map)

        for i in range(0, Controller.N):

            vx = states[i,0]
            vy = states[i,1]
            ey = states[i,3]
            epsi = states[i,4]
            theta = states[i,5]
            s = states[i,6]

            cur = curvature(s, Controller.map)
            delta = u[i, 0]  # EA: steering angle at K-1

            # standard model
            A12 = ((np.sin(delta) * Cf) / (m * vx))
            A13 = ((np.sin(delta) * Cf * lf) / (m * vx) + vy)

            A22 = (-(Cr + Cf * np.cos(delta)) / (m * vx))
            A23 = (-(lf * Cf * np.cos(delta) - lr * Cr) / (m * vx) - vx)

            A32 = (-(lf * Cf * np.cos(delta) - lr * Cr) / (I * vx))
            A33 = (-(lf * lf * Cf * np.cos(delta) + lr * lr * Cr) / (I * vx))

            A41 = np.sin(epsi)
            A42 = np.cos(epsi)

            A51 = ((1 / (1 - ey * cur)) * (-np.cos(epsi) * cur))
            A52 = ((1 / (1 - ey * cur)) * (np.sin(epsi) * cur))

            A61 = (np.cos(epsi) / (1 - ey * cur))
            A62 = (-np.sin(epsi) / (1 - ey * cur))

            A81 = np.cos(theta)
            A82 = -np.sin(theta)

            A91 = np.sin(theta)
            A92 = np.cos(theta)

            B11 = (-(np.sin(delta) * Cf) / m)
            B21 = ((np.cos(delta) * Cf) / m)
            B31 = ((lf * Cf * np.cos(delta)) / I)

            Ai = np.array([[-mu, A12, A13, 0., 0., 0., 0., 0., 0.],  # [vx]
                           [0., A22, A23, 0., 0., 0., 0., 0., 0.],  # [vy]
                           [0., A32, A33, 0., 0., 0., 0., 0., 0.],  # [wz]
                           [A41, A42, 0, 0., 0, 0., 0., 0., 0.],    # [ey]
                           [A51, A52, 1.,0., 0., 0., 0., 0., 0.],  # [epsi]
                           [0, 0, 1.,0., 0., 0., 0., 0., 0.],  # [theta]
                           [A61, A62, 0, 0., 0., 0., 0., 0., 0.],  # [s]
                           [A81, A82, 0, 0., 0., 0., 0., 0., 0.],  # [x]
                           [A91, A92, 0, 0., 0., 0., 0., 0., 0.],  # [y]
                           ])

            Bi = np.array([[B11, 1],  # [delta, a]
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

        return Atv, Btv, Ctv, ey_hor



