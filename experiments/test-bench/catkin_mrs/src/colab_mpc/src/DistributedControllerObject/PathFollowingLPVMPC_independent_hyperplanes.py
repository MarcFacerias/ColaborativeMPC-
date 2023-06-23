
from scipy import linalg, sparse
from cvxopt.solvers import qp
from cvxopt import spmatrix, matrix, solvers
from utilities import Curvature, get_ey
import numpy as np
from numpy import hstack, inf, ones
from scipy.sparse import vstack
from osqp import OSQP
from compute_plane import hyperplane_separator

solvers.options['show_progress'] = False

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

# TODO: check slack contraints it looks like there's some issue with them

class PathFollowingLPV_MPC:
    """Create the Path Following LMPC controller with LTV model
    Attributes:
        solve: given x0 computes the control action
    """
    def __init__(self, Q, R, N, dt, map, Solver, id):

        self.n_s = 9
        self.slack = 3
        self.n_exp = self.n_s + self.slack

        # Vehicle parameters:
        self.lf = 0.12
        self.lr = 0.14
        self.m  = 2.250
        self.I  = 0.06
        self.Cf = 60.0
        self.Cr = 60.0
        self.mu = 0.0
        self.vx_ref = 6.0

        self.id = id
        self.radius = 0.45

        self.max_vel = 10
        self.min_vel = 0.2

        # variable placeholders
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
        self.G = []
        self.E = []
        self.L = []
        self.Eu =[]

        self.first_it = True

        # previous values initialisation
        self.OldSteering = [0.0]
        self.OldAccelera = [0.0]*int(1)

        self.Solver = Solver # solver holder, string that tells if it's cvx or qp? TODO: remove cvx


    def _buildQ(self):
        # funtion to build the Q, which is states Q and slack variables
        Q = np.zeros((self.n_exp,self.n_exp))
        Q[0:self.n, 0: self.n] = self.Q

        Q[-self.slack:, -self.slack:] = 10000000*np.eye(self.slack)
        return Q

    def solve(self, x0, Last_xPredicted, uPred, x_agents, agents_id, pose):
        """Computes control action
        Arguments:
            x0: current state position
            EA: Last_xPredicted: it is just used for the warm up
            EA: uPred: set of last predicted control inputs used for updating matrix A LPV
            EA: A_L, B_L ,C_L: Set of LPV matrices
        """

        self.agent_list = agents_id # load the agent list

        if self.first_it:
            self.first_it = False
            self.n_agents = len(agents_id) # set number fo neighbours
            self.plane_comp = hyperplane_separator(self.n_agents, self.N) # load the hyperplane generation code

        # set the planes and compute them if necesary
        if x_agents is None:
            self.planes = np.zeros((self.N+1,self.n_agents,3))
            x_agents = np.zeros((self.N+1,self.n_agents,2))
        else:
            self.planes = self.plane_comp.compute_hyperplane(x_agents, pose, self.id, agents_id)

        # update system matrices
        self.A, self.B, self.C, ey = _EstimateABC(self, Last_xPredicted, uPred)

        # generate ineq constraints
        self.F, self.b = _buildMatIneqConst(self,ey)

        # generate eq constraints
        self.G, self.E, self.L, self.Eu, self.Eoa = _buildMatEqConst(self) # It's been introduced the C matrix (L in the output)

        # generate Q and q from the quadratic cost functions
        self.M, self.q          = _buildMatCost(self)

        uOld  = [self.OldSteering[0], self.OldAccelera[0]] # update the old control action to input it into the system matrices

        # solve the optimisation problem
        res_cons, feasible = osqp_solve_qp(sparse.csr_matrix(self.M), self.q, sparse.csr_matrix(self.F),
         self.b, sparse.csr_matrix(self.G), np.add(np.dot(self.E,x0), np.dot(self.Eu,uOld)))

        if feasible == 0:
            print ('QUIT...')

        Solution = res_cons.x

        # rearange the solution, packed form is (x0 x9, s ... , u1 u2, du1, du2) for all N+1
        idx = np.arange(0,self.n_s)
        for i in range(1,self.N+1):
            aux = np.arange(0,self.n_s) + i*self.n_exp
            idx = np.hstack((idx,aux))

        self.xPred = np.reshape((Solution[idx]), (self.N+1, self.n_s))
        self.uPred = np.reshape((Solution[self.n_exp * (self.N+1) + np.arange(self.d * self.N)]), (self.N, self.d)) # TODO: fix this with new variable structure
        self.OldSteering = [self.uPred[1,0]]
        self.OldAccelera = [self.uPred[1,1]]

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
        feasible = 1
    return res, feasible

def GenerateColisionAvoidanceConstraints(Controller):

    # function to generate the obstacle avoidance constraitns of the shape x * plane < d // x * plane > d note that both
    # constraints are expressed as < for convinience
    K_list = [] # list of submatrices
    Lim_list = [] # list of constraint limtis

    # we add the first elements to match dimensions, 0 as the initial state is fixed and not constrained
    K_list.append(np.zeros((len(Controller.agent_list), Controller.n_exp)))
    Lim_list.extend(np.zeros(len(Controller.agent_list)))

    # we will add a block of constraints from 1 to N
    for t in range(1,Controller.N + 1):

        K = np.zeros((len(Controller.agent_list),Controller.n_exp)) # generate a placeholder

        for i,el in enumerate(Controller.agent_list):

            if Controller.id < el: # if master
                K[i, 7] = Controller.planes[t-1, 0, i]
                K[i, 8] = Controller.planes[t-1, 1, i]
                K[i, -1] = -1
                Lim_list.append(- Controller.radius/2 - Controller.planes[t-1, 2, i] )

            else: # if slave
                K[i, 7] = - Controller.planes[t-1, 0, i]
                K[i, 8] = - Controller.planes[t-1, 1, i]
                K[i, -1] = -1
                Lim_list.append(Controller.planes[t-1, 2, i] - Controller.radius/2 )

        K_list.append(K) # append the block of constraints to the list

    return K_list, Lim_list

    '''END GENERATE HYPER CONSTRAITNS'''

def _buildMatIneqConst(Controller,ey):
    # Build the set of inequality constraints note that all constaints are expressed as < for convenience
    # cast some constants for conveniece
    N = Controller.N
    n_exp = Controller.n_exp
    max_vel = Controller.max_vel
    min_vel = Controller.min_vel

    # agent constraints placeholder
    Fx = np.zeros((4,n_exp))

    # linear velocity constraints + slack variables
    Fx[0,0] = -1
    Fx[1,0] = 1
    Fx[0,-3] = -1
    Fx[1,-3] = 1

    # limit lateral error with slack variables
    Fx[2,3] = 1
    Fx[3,3] = -1
    Fx[2,-2] = 1
    Fx[3,-2] = 1

    # generate the upper bounds of the velocity constraints
    bx_vel = np.tile(np.squeeze(np.array([[-min_vel],
                   [max_vel]])), N+1) # vx min; vx max; ey min; ey max; t1 min ... tn min

    # generate the upper bounds of the lateral error constraitns constraints
    if ey.shape[0] < N+1:
        ey = np.append(ey, ey[-1])

    bx_ey = np.repeat(np.array(ey),2).tolist()

    # piece of code used to rearenge the constraitns so that (v_ub, v_lb, ey_ub, ey_lb) for all horizon
    # TODO: we could rearange the constraints to avoid having to add this piece of code
    bxtot = iter(bx_vel)
    res = []

    for idx in range(0,len(bx_ey),2):
        res.extend([next(bxtot) for _ in range(2)])
        res += bx_ey[idx:idx+2]

    bxtot = np.array(res)

    # The resulting matrices represent Fx[i]x <= bxtot[b]

    # Builc the matrices for the input constraint in each region. In the region i we want Fu[i]x <= bu[b]
    Fu = np.array([[1., 0.],
                   [-1., 0.],
                   [0., 1.],
                   [0., -1.]])

    bu = np.array([[0.45], # Max right Steering
                   [0.45], # Max left Steering
                   [8.0],   # Max Acceleration
                   [8.0]])  # Max DesAcceleration


    rep_a = [Fx] * (N+1) # expand the constraints for the whole horizon
    k_list, lim_list = GenerateColisionAvoidanceConstraints(Controller)

    # smaill loop to rearange the constraints so that they are more readable
    for j,_ in enumerate(rep_a):
        rep_a[j] = np.vstack((rep_a[j],np.vstack(k_list[j])))

    Mat = linalg.block_diag(*rep_a) # make a block diagonal where the elements of the diagonal are the matrices in the list
    Fxtot = Mat

    # smaill loop to rearange the constraints so that they are more readable
    n = 5
    bxtot = iter(bxtot)
    res = []

    for idx in range(0,len(lim_list),Controller.n_agents):
        res.extend([next(bxtot) for _ in range(n - 1)])
        res += lim_list[idx:idx+Controller.n_agents]
    res.extend(bxtot)
    bxtot = np.array(res)

    # repeat the control actions along the horizon to constraint it in all time instants
    rep_b = [Fu] * (N)
    Futot = linalg.block_diag(*rep_b)
    butot = np.tile(np.squeeze(bu), N)

    # Let's stack everything together
    rFxtot, cFxtot = np.shape(Fxtot)
    rFutot, cFutot = np.shape(Futot)
    Dummy1 = np.hstack((Fxtot, np.zeros((rFxtot, cFutot))))
    Dummy2 = np.hstack((np.zeros((rFutot, cFxtot)), Futot))
    F = np.vstack((Dummy1, Dummy2))
    F = np.hstack((F,np.zeros((np.shape(F)[0], cFutot))))
    b = np.hstack((bxtot, butot))

    return F, b

def _buildMatCost(Controller):
    # EA: This represents to be: [(r-x)^T * Q * (r-x)] up to N+1
    # and [u^T * R * u] up to N

    q  = Controller._buildQ() # we collect list of Qs

    # copy some varaibles for clarity purposes
    R  = Controller.R
    N  = Controller.N

    # expand the q along the horizon
    d = [q] * (N+1)

    Mx = linalg.block_diag(*d)  # generate a diagonal matrix with the expanded Qs

    cu = [np.zeros((2,2))] * (N) # we consider no penalty to the control action so it's weight is 0
    c = [R] * (N) # we penalise it's change rate to prevent an agresive behaviour

    # once this is done we generate block diagonal matrices with each list along the horizon
    Mu = linalg.block_diag(*cu)
    Mdu = linalg.block_diag(*c)

    # Finally we merge all the sub cost matrices together:
    M0 = linalg.block_diag(Mx, Mu, Mdu)

    # Create matrices to padd missing values
    Pu = np.zeros(N*Controller.d)
    Pdu = np.zeros(N * Controller.d)
    Px = np.zeros(Controller.n_exp)
    Px[0] = -Controller.vx_ref*Controller.Q[0,0] # added to represent vx - vref in a quadratic fashion
    Px_total = np.tile(Px, N+1) # expand p along the horizon
    P= 2*np.hstack((Px_total, Pu, Pdu)) # padd missing values

    return 2 * M0, P

def _buildMatEqConst(Controller):
    # Buil matrices for optimization (Convention from Chapter 15.2 Borrelli, Bemporad and Morari MPC book)
    # We are going to build our optimization vector z \in \mathbb{R}^((N+1) \dot n \dot N \dot d), note that this vector
    # stucks the predicted trajectory x_{k|t} \forall k = t, \ldots, t+N+1 over the horizon and
    # the predicted input u_{k|t} \forall k = t, \ldots, t+N over the horizon
    # G * z = L + E * x(t) + Eu * OldInputs

    # Parse variables intro local for conviniece
    A = Controller.A
    B = Controller.B
    N = Controller.N # N horizon
    n_exp = Controller.n_exp # N horizon
    d = Controller.d # N horizon


    Gx = np.zeros((n_exp,n_exp ))
    Gx[:Controller.n_s,:Controller.n_s] = np.eye(Controller.n_s)
    Gx = linalg.block_diag(*[Gx]*(N+1))

    Gu = np.zeros(((n_exp) * (N+1), d * (N)))
    Gdu_aux = np.zeros(((n_exp) * (N+1), d * (N)))
    Gdu     = np.zeros((d * (N), n_exp * (N+1) + 2*d * (N)))

    E = np.zeros(((n_exp ) * (N+1) + N*Controller.d , Controller.n_s))
    E[:Controller.n_s,:Controller.n_s] = np.eye(Controller.n_s)

    Eu = np.zeros(((n_exp) * (N+1) + N*Controller.d , d))
    Eu[(n_exp) * (N+1) : (n_exp) * (N+1) + Controller.d, :] = np.eye(2)

    Eoa = np.zeros(((n_exp) * (N+1) + N*Controller.d , 1))
    L = np.zeros(((n_exp) * (N+1) + N*Controller.d , 1))

    # TODO: there's redundancy in this loops

    for i in range(1, N+1):
        Gx[i * (n_exp):i * (n_exp) + Controller.n_s, (i-1) * n_exp:(i-1) * n_exp + Controller.n_s] = -A[i-1]
        Gu[i * (n_exp):i * (n_exp) + Controller.n_s, (i - 1) * Controller.d: (i - 1) * Controller.d + Controller.d] = -B[i-1]

    Gdu[0: Controller.d, n_exp * (N + 1) : n_exp * (N + 1) + Controller.d] = np.eye(2)

    for i in range(1, N ):
        Gdu[ i * Controller.d: i * Controller.d + Controller.d,  n_exp * (N+1) + (i - 1) * Controller.d : n_exp * (N+1) + (i - 1) * Controller.d + Controller.d] = np.eye(2)
        Gdu[ i * Controller.d: i * Controller.d + Controller.d,  n_exp * (N+1) + i * Controller.d : n_exp * (N+1) + i * Controller.d + Controller.d] = -np.eye(2)
        Gdu[ i * Controller.d: i * Controller.d + Controller.d,  n_exp * (N+1) + Controller.d * (N) + (i - 1) * Controller.d :  n_exp * (N+1) + Controller.d * (N) + (i - 1) * Controller.d + Controller.d] = np.eye(2)

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

        ey_hor = get_ey(states[:, 6], Controller.map)

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

        return Atv, Btv, Ctv, ey_hor



