
from scipy import linalg, sparse
from cvxopt.solvers import qp
from cvxopt import spmatrix, matrix, solvers
from utilities import Curvature, GBELLMF
import datetime
import numpy as np
from numpy import hstack, inf, ones
from scipy.sparse import vstack
from osqp import OSQP
from compute_plane import hyperplane_separator

solvers.options['show_progress'] = False

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})



class PathFollowingLPV_MPC:
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given x0 computes the control action
    """
    def __init__(self, Q, R, N, dt, map, Solver, id):

        self.n_s = 9
        self.n_agents = 3 #TODO: remove this constant
        self.slack = 2
        self.n_exp = self.n_s + self.slack# slack variables

        # Vehicle parameters:
        self.lf = 0.12
        self.lr = 0.14
        self.m  = 2.250
        self.I  = 0.06
        self.Cf = 60.0
        self.Cr = 60.0
        self.mu = 0.1
        self.plane_comp = hyperplane_separator(self.n_agents, N)
        self.id = id
        self.radius = 0.3

        self.max_vel = 10
        self.min_vel = 0.2
        self.vel_ref = np.ones(N)*8

        self.A    = []
        self.B    = []
        self.C    = []
        self.N    = N
        self.n    = Q.shape[0]
        self.d    = R.shape[0]
        self.Q    = Q
        self.R    = R
        self.dt = dt                # Sample time 33 ms
        self.map = map              # Used for getting the road curvature

        self.first_it = 1

        self.OldSteering = [0.0]

        self.OldAccelera = [0.0]*int(1)

        self.OldPredicted = [0.0]

        self.Solver = Solver

        self.G = []
        self.E = []
        self.L = []
        self.Eu =[]


    def _buildQ(self):

        Q = np.zeros((self.n_exp,self.n_exp))
        Q[0:self.n, 0: self.n] = self.Q

        # TODO Update this with the slack variable count
        Q[-2, -2] = 100000000
        Q[-1, -1] = 100000000
        return Q

    def solve(self, x0, Last_xPredicted, uPred, x_agents, agents_id, pose):
        """Computes control action
        Arguments:
            x0: current state position
            EA: Last_xPredicted: it is just used for the warm up
            EA: uPred: set of last predicted control inputs used for updating matrix A LPV
            EA: A_L, B_L ,C_L: Set of LPV matrices
        """

        self.agent_list = agents_id

        if x_agents is None:
            self.planes = np.zeros((self.N+1,self.n_agents,3)) # TODO fix lambdas into n neighbours x H time
            x_agents = np.zeros((self.N+1,self.n_agents,2))
        else:
            self.planes = self.plane_comp.compute_hyperplane(x_agents, pose, self.id, agents_id)

        self.F, self.b = _buildMatIneqConst(self)

        self.A, self.B, self.C  = _EstimateABC(self, Last_xPredicted, uPred)

        self.G, self.E, self.L, self.Eu, self.Eoa  = _buildMatEqConst(self,x_agents) # It's been introduced the C matrix (L in the output)

        self.M, self.q          = _buildMatCost(self)

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
            sol = qp(M, matrix(q), F, matrix(b), G, E * matrix(x0))
            if sol['status'] == 'optimal':
                self.feasible = 1
            else:
                self.feasible = 0

            self.xPred = np.squeeze(np.transpose(np.reshape((np.squeeze(sol['x'])[np.arange(n * (N + 1))]), (N + 1, n))))
            self.uPred = np.squeeze(np.transpose(np.reshape((np.squeeze(sol['x'])[n * (N + 1) + np.arange(d * N)]), (N, d))))

        else:
            res_cons, feasible = osqp_solve_qp(sparse.csr_matrix(M), q, sparse.csr_matrix(F),
             b, sparse.csr_matrix(G), np.add( np.dot(E,x0) ,L[:,0],np.dot(Eu,uOld) ) + np.squeeze(self.Eoa) ) # TODO fix G and np.add( np.dot(E,x0) ,L[:,0],np.dot(Eu,uOld) )

            if feasible == 0:
                print ('QUIT...')

            Solution = res_cons.x

            # np.reshape((Solution[:-20]), (N, 14))
            idx = np.arange(0,9)
            for i in range(1,N+1):
                aux = np.arange(0,9) + i*self.n_exp
                idx = np.hstack((idx,aux))

            self.xPred = np.reshape((Solution[idx]), (N+1, self.n_s))
            self.uPred = np.reshape((Solution[self.n_exp * (N+1) + np.arange(d * N)]), (N, d))

        return feasible, Solution, self.planes


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
        osqp.setup(P=P.tocsc(), q=q, A=qp_A, l=qp_l, u=qp_u, verbose=False, polish=True, max_iter=500000)
    else:
        osqp.setup(P=P, q=q, A=None, l=None, u=None, verbose=False, polish=True)

    if initvals is not None:
        osqp.warm_start(x=initvals)
    res = osqp.solve()

    # ERROR MANAGEMENT https://osqp.org/docs/interfaces/status_values.html

    if res.info.status_val != 1:
        print("OSQP exited with status '%s'" % res.info.status)
    feasible = 0
    if res.info.status_val == 1 or res.info.status_val == 2 or  res.info.status_val == -2:
        # print("OSQP exited with status '%s'" % res.info.status)
        feasible = 1
    return res, feasible

def GenerateColisionAvoidanceConstraints(Controller):

    K_list = []
    Lim_list = []
    K = np.zeros((len(Controller.agent_list), Controller.n_exp))
    K_list.append(K)
    Lim_list.extend(np.zeros(len(Controller.agent_list)))

    for t in range(1,Controller.N + 1):

        K = np.zeros((len(Controller.agent_list),Controller.n_exp))

        for i,el in enumerate(Controller.agent_list):

            if Controller.id < el:
                K[i, 7] = Controller.planes[t-1, 0, i]
                K[i, 8] = Controller.planes[t-1, 1, i]
                K[i, -1] = -1
                Lim_list.append(- Controller.radius/2 - Controller.planes[t-1, 2, i] )

            else:
                K[i, 7] = - Controller.planes[t-1, 0, i]
                K[i, 8] = - Controller.planes[t-1, 1, i]
                K[i, -1] = -1
                Lim_list.append(Controller.planes[t-1, 2, i] - Controller.radius/2 )

        K_list.append(K)

    return K_list, Lim_list

    '''END GENERATE HYPER CONSTRAITNS'''

def _buildMatIneqConst(Controller):
    N = Controller.N
    n_exp = Controller.n_exp

    max_vel = Controller.max_vel
    min_vel = Controller.min_vel
    Fx = np.zeros((4,n_exp))

    Fx[0,0] = -1
    Fx[1,0] = 1

    # limit lateral error with slack variables
    Fx[2,3] = 1
    Fx[3,3] = -1
    Fx[2,-2] = 1
    Fx[3,-2] = 1

    #B
    bx = np.array([[-min_vel],
                   [max_vel],
                   [0.60],
                   [0.60]]) # vx min; vx max; ey min; ey max; t1 min ... tn min

    # Builc the matrices for the input constraint in each region. In the region i we want Fx[i]x <= bx[b]
    Fu = np.array([[1., 0.],
                   [-1., 0.],
                   [0., 1.],
                   [0., -1.]])

    bu = np.array([[0.45], # Max right Steering
                   [0.45], # Max left Steering
                   [8.0],   # Max Acceleration
                   [8.0]])  # Max DesAcceleration


    rep_a = [Fx] * (N+1) # add n times Fx to a list
    k_list, lim_list = GenerateColisionAvoidanceConstraints(Controller)

    for j,_ in enumerate(rep_a):
        rep_a[j] = np.vstack((rep_a[j],np.vstack(k_list[j])))

    Mat = linalg.block_diag(*rep_a) # make a block diagonal where the elements of the diagonal are the matrices in the list
    '''
    NoTerminalConstr = np.zeros((np.shape(Mat)[0], n_exp))  # No need to constraint also the terminal point
    Fxtot = np.hstack((Mat, NoTerminalConstr))
    '''

    Fxtot = Mat
    bxtot = np.tile(np.squeeze(bx), N+1)

    n = 5
    bxtot = iter(bxtot)
    res = []

    for idx in range(0,len(lim_list),Controller.n_agents):
        res.extend([next(bxtot) for _ in range(n - 1)])
        res += lim_list[idx:idx+Controller.n_agents]
    res.extend(bxtot)

    bxtot = np.array(res)

    rep_b = [Fu] * (N)
    Futot = linalg.block_diag(*rep_b)
    butot = np.tile(np.squeeze(bu), N)

    # Let's stack all together
    rFxtot, cFxtot = np.shape(Fxtot)
    rFutot, cFutot = np.shape(Futot)
    Dummy1 = np.hstack((Fxtot, np.zeros((rFxtot, cFutot))))
    Dummy2 = np.hstack((np.zeros((rFutot, cFxtot)), Futot))
    F = np.vstack((Dummy1, Dummy2))
    F = np.hstack((F,np.zeros((np.shape(F)[0], cFutot))))
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
    q  = Controller._buildQ()
    # I consider R to have the proper shape ( 2xN )
    R  = Controller.R
    N  = Controller.N
    d = [q] * (N+1)

    Mx = linalg.block_diag(*d)

    c = [R] * (N)

    Mu = linalg.block_diag(*c)
    Mdu = linalg.block_diag(*c)

    # This is without slack lane:
    M0 = linalg.block_diag(Mx, Mu, Mdu)

    # Maximise change on S
    Pu = np.zeros(N*Controller.d)
    Pdu = np.zeros(N * Controller.d)
    Px = np.zeros(Controller.n_exp)
    # Px[6] = -1
    Px[0] = -6*100
    Px_total = np.tile(Px, N+1)
    P= 2*np.hstack((Px_total, Pu, Pdu))

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
    N = Controller.N # N horizon
    n_exp = Controller.n_exp # N horizon
    d = Controller.d # N horizon

    auxG = np.eye(Controller.n_exp-Controller.slack)

    Gx = np.zeros((n_exp,n_exp ))
    Gx[:auxG.shape[0],:auxG.shape[0]] = auxG
    Gx = linalg.block_diag(*[Gx]*(N+1))

    Gu = np.zeros(((n_exp) * (N+1), d * (N)))
    Gdu_aux = np.zeros(((n_exp) * (N+1), d * (N)))
    Gdu     = np.zeros((d * (N), n_exp * (N+1) + d * (N)))

    E = np.zeros(((n_exp) * (N+1), Controller.n_s))
    E[:Controller.n_s,:Controller.n_s] = np.eye(Controller.n_s)

    Eu = np.zeros(((n_exp) * (N+1), d))

    Eoa = np.zeros(((n_exp) * (N+1), 1))
    L = np.zeros(((n_exp) * (N+1), 1)) # I guess L represents previous inputs? whatever rn is 0s

    # TODO ADD agent initial state


    for i in range(1, N+1): # TODO: there's redundancy in this loops
        Gx[i * (n_exp):i * (n_exp) + Controller.n_s, (i-1) * n_exp:(i-1) * n_exp + Controller.n_s] = -A[i-1]
        Gu[i * (n_exp):i * (n_exp) + Controller.n_s, (i - 1) * Controller.d: (i - 1) * Controller.d + Controller.d] = -B[i-1]
        Gdu[i * Controller.d:i * Controller.d + 1,  n_exp * (N+1) + (i - 1) * Controller.d : n_exp * (N+1) + (i - 1) * Controller.d +1] = np.ones((2,2))
        Gdu[i * Controller.d:i * Controller.d + 1,  n_exp * (N+1) + Controller.d * (N) + (i - 1) * Controller.d : (N+1) + Controller.d * (N) + (i - 1) * Controller.d +1] = np.ones((2,2))

    G = np.hstack((Gx, Gu, Gdu_aux))
    G = np.vstack((G, Gdu))
    return G, E, L, Eu, Eoa

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

        for i in range(0, Controller.N):

            vx = states[i,0]
            vy = states[i,1]
            ey = states[i,3]
            epsi = states[i,4]
            theta = states[i,5]
            s = states[i,6]

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
                           [0, A7, 0, 0., A8, 0., 0., 0., 0.],    # [ey]
                           [A51, A52, 1.,0., 0., 0., 0., 0., 0.],  # [epsi]
                           [0, 0, 1.,0., 0., 0., 0., 0., 0.],  # [theta]
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



