#!/usr/bin/env python
"""
    File name: stateEstimator.py
    Author: Shuqi Xu and Ugo Rosolia
    Email: shuqixu@berkeley.edu (xushuqi8787@gmail.com)
    Modified: Eugenio Alcala
    Email: eugenio.alcala@upc.edu
    Date: 09/30/2018
    Python Version: 2.7.12
"""
from scipy import linalg, sparse
import numpy as np
from cvxopt.solvers import qp
from cvxopt import spmatrix, matrix, solvers
from utilities import Curvature, GBELLMF
import datetime
import numpy as np
from numpy import linalg as la
import pdb
from numpy import hstack, inf, ones
from scipy.sparse import vstack
from osqp import OSQP
from numpy import tan, arctan, cos, sin, pi

solvers.options['show_progress'] = False

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class PathFollowingLPV_MPC:
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given x0 computes the control action
    """
    def __init__(self, Q, R, N, dt, map, Solver):

        self.n_s = 9
        self.n_agents = 2
        self.n_exp = self.n_s + 3 * (self.n_agents)
        # Vehicle parameters:
        self.lf = 1
        self.lr = 1
        self.m  = 10
        self.I  = 0.05
        self.Cf = 0.5
        self.Cr = 0.5
        self.mu = 0.5
        self.radius = 1.5
        self.g  = 9.81
        self.lambdas = 5*np.ones(self.n_agents)

        self.max_vel = 10
        # self.max_vel = self.max_vel - 0.2*self.max_vel     

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

        self.first_it = 1

        self.OldSteering = [0.0]

        self.OldAccelera = [0.0]*int(1)        

        self.OldPredicted = [0.0]

        self.Solver = Solver

        self.F, self.b = _buildMatIneqConst(self)

        self.G = []
        self.E = []
        self.L = []
        self.Eu =[]      
        

    def _buildQ(self):

        Q_list = []

        for t in range(0,self.N):
            Q_states = np.zeros((self.n_s-2,self.n_exp))

            Q_x1 = np.zeros((1,self.n_exp))
            Q_x1[0,self.n_s-2] = np.sum(self.lambdas[:,t])

            j = 0
            for i in range(self.n_s,self.n_exp-self.d,2):

                Q_x1[0,i] = -self.lambdas[j,t]
                j += 1

                if j >= self.n_agents:
                    j = 0

            Q_y1 = np.zeros((1,self.n_exp))
            Q_y1[0,self.n_s-1] = np.sum(self.lambdas[:,t])

            j = 0
            for i in range(self.n_s+1,self.n_exp-self.d,2):

                Q_y1[0,i] = -self.lambdas[j,t]
                j += 1

                if j >= self.n_agents:
                    j = 0

            Q_n = np.zeros((self.n_agents*2,self.n_exp))
            j = 0
            for i in range(0,self.n_agents*2,2):

                Q_n[i,7]       = -self.lambdas[j,t]
                Q_n[i+1,8]     = -self.lambdas[j,t]
                Q_n[i,9+i]     = self.lambdas[j,t]
                Q_n[i+1,9+i+1] = self.lambdas[j,t]

                j += 1

            Q = np.vstack((Q_states,Q_x1,Q_y1,Q_n,np.zeros((self.d,self.n_exp))))
            Q_list.append(Q)

        # terminal cost
        Q_states = np.zeros((self.n_s - 2, self.n_exp))
        Q_x1 = np.zeros((1, self.n_exp))
        Q_y1 = np.zeros((1, self.n_exp))
        Q_n = np.zeros((self.n_agents * 2, self.n_exp))
        Q = np.vstack((Q_states, Q_x1, Q_y1, Q_n,np.zeros((self.d,self.n_exp))))
        Q_list.append(Q)
        return Q_list

    def solve(self, x0, Last_xPredicted, uPred, NN_LPV_MPC, A_L, B_L ,C_L, first_it, lambdas, x_agents):
        """Computes control action
        Arguments:
            x0: current state position
            EA: Last_xPredicted: it is just used for the warm up
            EA: uPred: set of last predicted control inputs used for updating matrix A LPV
            EA: A_L, B_L ,C_L: Set of LPV matrices
        """
        startTimer              = datetime.datetime.now()

        self.lambdas = lambdas

        if (NN_LPV_MPC == False) and (first_it < 5):
            self.A, self.B, self.C  = _EstimateABC(self, Last_xPredicted, uPred)
        else:
            self.A = A_L
            self.B = B_L
            self.C = C_L


        self.G, self.E, self.L, self.Eu, self.Eoa  = _buildMatEqConst(self,x_agents) # It's been introduced the C matrix (L in the output)

        self.M, self.q          = _buildMatCost(self)


        endTimer                = datetime.datetime.now()
        deltaTimer              = endTimer - startTimer
        self.linearizationTime  = deltaTimer

        M = self.M
        q = self.q
        G = self.G
        E = self.E
        L = self.L
        Eu= self.Eu
        F = self.F
        b = self.b
        n = self.n
        N = self.N
        d = self.d

        uOld  = [self.OldSteering[0], self.OldAccelera[0]]

        
        if self.Solver == "CVX":
            startTimer = datetime.datetime.now()
            sol = qp(M, matrix(q), F, matrix(b), G, E * matrix(x0))
            endTimer = datetime.datetime.now(); deltaTimer = endTimer - startTimer
            self.solverTime = deltaTimer
            if sol['status'] == 'optimal':
                self.feasible = 1
            else:
                self.feasible = 0

            self.xPred = np.squeeze(np.transpose(np.reshape((np.squeeze(sol['x'])[np.arange(n * (N + 1))]), (N + 1, n))))
            self.uPred = np.squeeze(np.transpose(np.reshape((np.squeeze(sol['x'])[n * (N + 1) + np.arange(d * N)]), (N, d))))
        else:
            startTimer = datetime.datetime.now()
            np.all(np.linalg.eigvals(M) >= 0)
            vstack([F, G])
            res_cons, feasible = osqp_solve_qp(sparse.csr_matrix(M), q, sparse.csr_matrix(F),
             b, sparse.csr_matrix(G), np.add( np.dot(E,x0) ,L[:,0],np.dot(Eu,uOld) ) + np.squeeze(self.Eoa) ) # TODO fix G and np.add( np.dot(E,x0) ,L[:,0],np.dot(Eu,uOld) )
            
            if feasible == 0:
                print ('QUIT...')

            Solution = res_cons.x

            endTimer = datetime.datetime.now(); deltaTimer = endTimer - startTimer
            self.solverTime = deltaTimer

            idx = np.arange(0,9)
            for i in range(1,N+1):
                aux = np.arange(0,9) + i*self.n_exp
                idx = np.hstack((idx,aux))

            self.xPred = np.reshape((Solution[idx]), (N + 1, self.n_s))
            self.uPred = np.reshape((Solution[self.n_exp * (N + 1) + np.arange(d * N)]), (N, d))

        self.LinPoints = np.concatenate( (self.xPred.T[1:,:], np.array([self.xPred.T[-1,:]])), axis=0 )

        return feasible, Solution



    def LPVPrediction(self, x, u, vel_ref):

        lf  = self.lf
        lr  = self.lr
        m   = self.m
        I   = self.I
        Cf  = self.Cf
        Cr  = self.Cr   
        mu  = self.mu

        STATES_vec = np.zeros((self.N, 6))

        Atv = []
        Btv = []
        Ctv = []
        # print("---------------------------")
        for i in range(0, self.N):

            if i==0:
                states  = np.reshape(x, (6,1))

            vy      = float(states[1])
            epsi    = float(states[3])
            s       = float(states[4])
            ey      = float(states[5])
            theta   = float(states[7])

            PointAndTangent = self.map.PointAndTangent[:,:,self.map.lane]
            cur     = Curvature(s, PointAndTangent)

            # print("s" + str(s))
            # print("Curvature" + str(cur))
            # TODO: Add error handling
            vx      = float(vel_ref[i,0])
            delta   = float(u[i,0])            # EA: steering angle at K-1
    
            if vx < 1.1:

                #low vel model: straight line .
                A12 = 0
                A13 = 0
                A22 = 0
                A23 = 0
                A32 = 0
                A33 = 0
                B11 = 0
                B12 = 1
                B22 = 0


            else:

                #standard model
                A12 = (np.sin(delta) * Cf) / (m*vx)
                A13 = (np.sin(delta) * Cf * lf) / (m*vx) + vy
                A22 = -(Cr + Cf * np.cos(delta)) / (m*vx)
                A23 = -(lf * Cf * np.cos(delta) - lr * Cr) / (m*vx) - vx
                A32 = -(lf * Cf * np.cos(delta) - lr * Cr) / (I*vx)
                A33 = -(lf * lf * Cf * np.cos(delta) + lr * lr * Cr) / (I*vx)
                B11     = -(np.sin(delta)*Cf)/m
                B12     = 1
                B22     = np.sin(delta)

            A11 = -mu
            A41 = np.sin(epsi)
            A42 = np.cos(epsi)
            A51 = (1 / (1 - ey * cur)) * (-np.cos(epsi) * cur)
            A52 = (1 / (1 - ey * cur)) * (+np.sin(epsi) * cur)
            A61 = np.cos(theta)
            A62 = -np.sin(theta)
            A71 = np.sin(theta)
            A72 = np.cos(theta)
            A91 = np.cos(epsi) / (1 - ey * cur)
            A92 = -np.sin(epsi) / (1 - ey * cur)

            B21 = (np.cos(delta) * Cf) / m
            B31 = (lf * Cf * np.cos(delta)) / I

            Ai = np.array([[A11, A12, A13, 0., 0., 0., 0., 0., 0.], # [vx]
                           [0., A22, A23, 0., 0., 0., 0., 0., 0.],  # [vy]
                           [0., A32, A33, 0., 0., 0., 0., 0., 0.],  # [wz]
                           [A41, A42, 0, 0., 0., 0., 0., 0., 0.],   # [ey]
                           [A51, A52, 1., 0., 0., 0., 0., 0., 0.],  # [epsi]
                           [0, 0, 1., 0., 0., 0., 0., 0., 0.],      # [theta]
                           [A91, A92, 0., 0., 0., 0., 0., 0., 0.],  # [s]
                           [A61, A62, 0., 0., 0., 0., 0., 0., 0.],  # [x]
                           [A71, A72, 0., 0., 0., 0., 0., 0., 0.],  # [y]
                           ])

            Bi = np.array([[B11, B12],  # [delta, a]
                           [B21, B22],
                           [B31, 0],
                           [0, 0],
                           [0, 0],
                           [0, 0],
                           [0, 0],
                           [0, 0],
                           [0, 0]])


            Ci  = np.array([[ 0 ], 
                            [ 0 ],
                            [ 0 ],
                            [ 0 ],
                            [ 0 ],
                            [0],
                            [0],
                            [0],
                            [ 0 ]])               


            Ai = np.eye(len(Ai)) + self.dt * Ai
            Bi = self.dt * Bi
            Ci = self.dt * Ci

            states_new = np.dot(Ai, states) + np.dot(Bi, np.transpose(np.reshape(u[i,:],(1,2))))

            STATES_vec[i] = np.reshape(states_new, (6,))

            states = states_new

            Atv.append(Ai)
            Btv.append(Bi)
            Ctv.append(Ci)

        return STATES_vec, Atv, Btv, Ctv
        




# ======================================================================================================================
# ======================================================================================================================
# =============================== Internal functions for MPC reformulation to QP =======================================
# ======================================================================================================================
# ======================================================================================================================




def osqp_solve_qp(P, q, G=None, h=None, A=None, b=None, initvals=None):
    # EA: P represents the quadratic weight composed by N times Q and R matrices.
    """
    Solve a Quadratic Program defined as:
        minimize
            (1/2) * x.T * P * x + q.T * x
        subject to
            G * x <= h
            A * x == b
    using OSQP <https://github.com/oxfordcontrol/osqp>.
    Parameters
    ----------
    P : scipy.sparse.csc_matrix Symmetric quadratic-cost matrix.
    q : numpy.array Quadratic cost vector.
    G : scipy.sparse.csc_matrix Linear inequality constraint matrix.
    h : numpy.array Linear inequality constraint vector.
    A : scipy.sparse.csc_matrix, optional Linear equality constraint matrix.
    b : numpy.array, optional Linear equality constraint vector.
    initvals : numpy.array, optional Warm-start guess vector.
    Returns
    -------
    x : array, shape=(n,)
        Solution to the QP, if found, otherwise ``None``.
    Note
    ----
    OSQP requires `P` to be symmetric, and won't check for errors otherwise.
    Check out for this point if you e.g. `get nan values
    <https://github.com/oxfordcontrol/osqp/issues/10>`_ in your solutions.
    """

    # TODO: Add the fixed values as equality constraints + add the initial states
    osqp = OSQP()
    if G is not None:
        l = -inf * ones(len(h))
        if A is not None:
            qp_A = vstack([G, A]).tocsc()
            qp_l = hstack([l, b])
            qp_u = hstack([h, b])
        else:  # no equality constraint
            qp_A = G
            qp_l = l
            qp_u = h
        osqp.setup(P=P, q=q, A=qp_A, l=qp_l, u=qp_u, verbose=False, polish=True)
    else:
        osqp.setup(P=P, q=q, A=None, l=None, u=None, verbose=True, polish=True)
    if initvals is not None:
        osqp.warm_start(x=initvals)
    res = osqp.solve()

    # ERROR MANAGEMENT https://osqp.org/docs/interfaces/status_values.html

    if res.info.status_val != 1:
        print("OSQP exited with status '%s'" % res.info.status)
    feasible = 0
    if res.info.status_val == 1 or res.info.status_val == 2 or  res.info.status_val == -2:
        feasible = 1
    return res, feasible



def _buildMatIneqConst(Controller):
    N = Controller.N
    n_exp = Controller.n_exp

    max_vel = Controller.max_vel

    # Ax< B
    Fx = np.zeros((4+ Controller.n_agents,n_exp))
    # limit velocity
    Fx[0,0] = -1
    Fx[1,0] = 1

    # limit lateral error
    Fx[2,3] = 1
    Fx[3,3] = -1

    # Limits slack variables

    # TODO MAKE THIS LOOPS BEETTER
    Fx[4, 13] = 1
    Fx[5, 14] = 1

    #B
    bx = np.array([[-0.0],
                   [max_vel],
                   [1.5],
                   [1.5]]) # vx min; vx max; ey min; ey max; t1 min ... tn min

    # btl = -(Controller.radius)**2*np.ones((Controller.n_agents,1)) # vx min; vx max; ey min; ey max; t1 min ... tn min
    btl = np.zeros((Controller.n_agents,1)) # vx min; vx max; ey min; ey max; t1 min ... tn min

    # btu = 100 * np.ones((Controller.n_agents, 1))  # vx min; vx max; ey min; ey max; t1 min ... tn min
    # bx = np.vstack((bx,btl,btu))

    bx = np.vstack((bx, btl))

    # Builc the matrices for the input constraint in each region. In the region i we want Fx[i]x <= bx[b]
    Fu = np.array([[1., 0.],
                   [-1., 0.],
                   [0., 1.],
                   [0., -1.]])

    bu = np.array([[0.45], # Max right Steering
                   [0.45], # Max left Steering
                   [4.0],   # Max Acceleration
                   [3.0]])  # Max DesAcceleration


    rep_a = [Fx] * (N) # add n times Fx to a list
    Mat = linalg.block_diag(*rep_a) # make a block diagonal where the elements of the diagonal are the matrices in the list
    NoTerminalConstr = np.zeros((np.shape(Mat)[0], n_exp))  # No need to constraint also the terminal point
    Fxtot = np.hstack((Mat, NoTerminalConstr))
    bxtot = np.tile(np.squeeze(bx), N)
    bxtot = np.append(bxtot,(0,0))
    t_limtis = np.zeros((Controller.n_agents, np.shape(Fxtot)[1]))

    # TODO generalitzar
    t_limtis[0,163] = 1
    t_limtis[1,164] = 1

    Fxtot = np.vstack((Fxtot,t_limtis))

    # Let's start by computing the submatrix of F relates with the input
    rep_b = [Fu] * (N)
    Futot = linalg.block_diag(*rep_b)
    butot = np.tile(np.squeeze(bu), N)

    # Let's stack all together
    rFxtot, cFxtot = np.shape(Fxtot)
    rFutot, cFutot = np.shape(Futot)
    Dummy1 = np.hstack((Fxtot, np.zeros((rFxtot, cFutot))))
    Dummy2 = np.hstack((np.zeros((rFutot, cFxtot)), Futot))
    F = np.vstack((Dummy1, Dummy2))
    b = np.hstack((bxtot, butot))

    if Controller.Solver == "CVX":
        F_sparse = spmatrix(F[np.nonzero(F)], np.nonzero(F)[0].astype(int), np.nonzero(F)[1].astype(int), F.shape)
        F_return = F_sparse
    else:
        F_return = F
    
    return F_return, b



def _buildMatCost(Controller):
    # EA: This represents to be: [(r-x)^T * Q * (r-x)] up to N+1
    # and [u^T * R * u] up to N

    # I consider Q to have the proper shape (9 base states +3N  with N being the neighbours )
    b  = Controller._buildQ()
    # I consider R to have the proper shape ( 2xN )
    R  = Controller.R
    N  = Controller.N

    Mx = linalg.block_diag(*b)

    c = [R] * (N) # Need to add dR for the derivative input cost

    Mu = linalg.block_diag(*c)

    # This is without slack lane:
    M0 = linalg.block_diag(Mx, Mu)

    # Maximise change on S
    P = np.zeros((N+1)*Controller.n_exp + N*Controller.d)
    P[Controller.n_exp*(N-1) + 6] = -1
    for i in range(0,Controller.d):
        P[2*Controller.n_exp - Controller.d + i :-(Controller.N-1)*2:Controller.n_exp ] = Controller.lambdas[i,:]

    M = 2 * M0  # Need to multiply by two because CVX considers 1/2 in front of quadratic cost

    if Controller.Solver == "CVX":
        M_sparse = spmatrix(M[np.nonzero(M)], np.nonzero(M)[0].astype(int), np.nonzero(M)[1].astype(int), M.shape)
        M_return = M_sparse
    else:
        M_return = M

    return M_return, P



def _buildMatEqConst(Controller, agents):
    # Buil matrices for optimization (Convention from Chapter 15.2 Borrelli, Bemporad and Morari MPC book)
    # We are going to build our optimization vector z \in \mathbb{R}^((N+1) \dot n \dot N \dot d), note that this vector
    # stucks the predicted trajectory x_{k|t} \forall k = t, \ldots, t+N+1 over the horizon and
    # the predicted input u_{k|t} \forall k = t, \ldots, t+N over the horizon
    # G * z = L + E * x(t) + Eu * OldInputs
    # TODO Check Eu and E
    A = Controller.A
    B = Controller.B
    C = Controller.C
    N = Controller.N # N horizon
    n_exp = Controller.n_exp # N horizon
    d = Controller.d # N horizon

    auxG = np.eye(Controller.n_exp-2)

    Gx = np.zeros((n_exp-2,n_exp ))
    Gx[:auxG.shape[0],:auxG.shape[0]] = auxG
    Gx = linalg.block_diag(*[Gx]*(N+1))
    Gx[-(n_exp - 2):, -(n_exp):] = np.zeros((n_exp - 2, n_exp))
    # Gx[:(Controller.n_exp - 2),:(Controller.n_exp - 2)] += np.eye(Controller.n_exp - 2)

    Gu = np.zeros(((n_exp-2) * (N + 1), d * (N)))

    E = np.zeros(((n_exp-2) * (N + 1), Controller.n_s))
    E[:Controller.n_s,:Controller.n_s] = np.eye(Controller.n_s)

    Eu = np.zeros(((n_exp-2) * (N + 1), d))

    Eoa = np.zeros(((n_exp-2) * (N + 1), 1))
    L = np.zeros(((n_exp-2) * (N + 1), 1)) # I guess L represents previous inputs? whatever rn is 0s

    # TODO ADD agent initial state

    for j in range(0, Controller.d * 2, 2):  # TODO fix this loop
        Eoa[Controller.n_s + j, 0] = agents[0, int(j / 2), 0]
        Eoa[Controller.n_s + j + 1, 0] = agents[0, int(j / 2), 1]

    for i in range(1, N):
        Gx[i * (n_exp-2):i * (n_exp-2)+Controller.n_s, (i-1) * n_exp:(i-1) * n_exp + Controller.n_s] -= A[i]

        for j in range(0, Controller.d*2,2): #TODO fix this loop
            Eoa[i * (Controller.n_exp-2) + Controller.n_s + j, 0] = agents[i,int(j/2),0]
            Eoa[i * (Controller.n_exp-2) + Controller.n_s + j + 1, 0] = agents[i,int(j/2),1]

        Gu[i * (n_exp-2):i * (n_exp-2)+Controller.n_s, (i-1) * Controller.d: (i-1) * Controller.d + Controller.d] -= B[i]

    G = np.hstack((Gx, Gu))
    
    return G, E, L, Eu, Eoa


def _EstimateABC(Controller,Last_xPredicted, uPredicted):

    N   = Controller.N
    dt  = Controller.dt
    lf  = Controller.lf
    lr  = Controller.lr
    m   = Controller.m
    I   = Controller.I
    Cf  = Controller.Cf
    Cr  = Controller.Cr
    mu  = Controller.mu

    Atv = []
    Btv = []
    Ctv = []

    for i in range(0, N):

        vy      = Last_xPredicted[i,1]
        epsi    = Last_xPredicted[i,3]
        s       = Last_xPredicted[i,8]
        ey      = Last_xPredicted[i,4]
        theta   = Last_xPredicted[i, 7]
        cur     = Curvature(s, Controller.map)
        vx      = Last_xPredicted[i,0]
        delta   = uPredicted[i,0]             #EA: set of predicted steering angles

        if vx < 1.1:

            # low vel model: straight line .
            A12 = 0
            A13 = 0
            A22 = 0
            A23 = 0
            A32 = 0
            A33 = 0
            B11 = 0
            B12 = 1
            B22 = 0


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
            B22 = np.sin(delta)

        A11 = -mu
        A41 = np.sin(epsi)
        A42 = np.cos(epsi)
        A51 = (1 / (1 - ey * cur)) * (-np.cos(epsi) * cur)
        A52 = (1 / (1 - ey * cur)) * (+np.sin(epsi) * cur)
        A61 = np.cos(theta)
        A62 = -np.sin(theta)
        A71 = np.sin(theta)
        A72 = np.cos(theta)
        A91 = np.cos(epsi) / (1 - ey * cur)
        A92 = -np.sin(epsi) / (1 - ey * cur)

        B21 = (np.cos(delta) * Cf) / m
        B31 = (lf * Cf * np.cos(delta)) / I

        Ai = np.array([[A11, A12, A13, 0., 0., 0., 0., 0., 0.],  # [vx]
                       [0., A22, A23, 0., 0., 0., 0., 0., 0.],  # [vy]
                       [0., A32, A33, 0., 0., 0., 0., 0., 0.],  # [wz]
                       [A41, A42, 0, 0., 0., 0., 0., 0., 0.],  # [ey]
                       [A51, A52, 1., 0., 0., 0., 0., 0., 0.],  # [epsi]
                       [0, 0, 1., 0., 0., 0., 0., 0., 0.],  # [theta]
                       [A91, A92, 0., 0., 0., 0., 0., 0., 0.], # [s]
                       [A61, A62, 0., 0., 0., 0., 0., 0., 0.],  # [x]
                       [A71, A72, 0., 0., 0., 0., 0., 0., 0.],  # [y]
                        ])

        Bi = np.array([[B11, B12],  # [delta, a]
                       [B21, B22],
                       [B31, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0]])

        Ai = np.eye(len(Ai)) + dt * Ai
        Bi = dt * Bi
        Ci = np.zeros((9,1))
        #############################################
        Atv.append(Ai)
        Btv.append(Bi)
        Ctv.append(Ci)

    return Atv, Btv, Ctv


