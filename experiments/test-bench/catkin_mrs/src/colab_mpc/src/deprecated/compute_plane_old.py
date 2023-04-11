from scipy import linalg, sparse
from cvxopt.solvers import qp
from cvxopt import spmatrix, matrix, solvers
from utilities import Curvature, GBELLMF
import datetime
import numpy as np

from numpy import hstack, inf, ones
from scipy.sparse import vstack
from osqp import OSQP

class hyperplane_separator():
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

    def __init__(self, n_agents, horizon, poly_dim = 4, d_safety = 0.5):

        self.n_agents = n_agents
        self.horizon = horizon
        self.poly_dim  = poly_dim
        self.d_safety = d_safety
        self.plane_states = 3
        self.slack_vars = 1
        self.planer_states = self.plane_states+self.slack_vars * (n_agents-1)


    def build_QP(self):

        # submatrix related to 3 hyperplane parameters + 2 slack variables
        aux = np.zeros((self.plane_states+self.slack_vars, self.plane_states+self.slack_vars))
        # aux[-self.slack_vars:, -self.slack_vars:] = 10000*np.eye(self.slack_vars)
        aux[:self.plane_states, :self.plane_states] = np.eye(self.plane_states)

        Q_local = linalg.block_diag(*[aux] * (self.n_agents -1))
        P = linalg.block_diag(*[Q_local] * (self.horizon))

        return P, np.zeros((1,self.planer_states*self.horizon))

    def build_cons(self, agents):
        #agents of the shape (agent, points,x/y ,horizon)
        # submatrix related to 3 hyperplane parameters + 2 slack variables

        for i in range(0, self.horizon):
            for j in range(1,self.n_agents):
                for l in range(0,self.poly_dim):

                    aux_hype_ineq = np.array([[agents[0,l,0,i]        , agents[0,l,1,i], +1, 0],
                                              [-agents[j, l, 0, i]    , -agents[j, l, 1, i],  -1, 0]])



                    aux_safety = np.array([self.d_safety, -self.d_safety])

                    # stack the ineq constaitns
                    try:
                        hype_ = np.vstack((hype_,aux_hype_ineq))
                    except:
                        hype_ = aux_hype_ineq

                    # stack the values for ineq constraints
                    try:
                        h = np.hstack((h,aux_safety))
                    except:
                        h = aux_safety

                # stack the ineq constaitns
                try:
                    plane_conts_local = linalg.block_diag(*(plane_conts_local, hype_))

                except:
                    plane_conts_local = hype_

                # stack the eq constaitns
                try:
                    plane_eq_local = linalg.block_diag(*(plane_eq_local, np.array([1, 1, 0, 0])))

                except:
                    plane_eq_local = np.array([1, 1, 0, 0])

                # stack the values for eq constraints
                try:
                    b = np.hstack((b,np.array([1])))
                except:
                    b = np.array([1])

                del hype_


            try:
                plane_conts_global = linalg.block_diag(*(plane_conts_global, plane_conts_local))

            except:
                plane_conts_global = plane_conts_local

            # stack the eq constaitns
            try:
                plane_eq_global = linalg.block_diag(*(plane_eq_global, plane_eq_local))

            except:
                plane_eq_global = plane_eq_local

            print(plane_conts_global.shape[0], h.shape[0], plane_eq_global.shape[0], b.shape[0])

            del plane_conts_local
            del plane_eq_local

        G = plane_conts_global
        A = plane_eq_global

        return G,h,None,b

    def osqp_solve_qp(self, P, q, G=None, h=None, A=None, b=None):

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
            osqp.setup(P=sparse.csr_matrix(P), q=q.flatten(), A=sparse.csr_matrix(qp_A), l=qp_l, u=qp_u, verbose=False, polish=True, max_iter=1000000)
        else:
            osqp.setup(P=P, q=q.flatten(), A=None, l=None, u=None, verbose=False, polish=True)

        res = osqp.solve()

        # ERROR MANAGEMENT https://osqp.org/docs/interfaces/status_values.html

        if res.info.status_val != 1:
            print("OSQP exited with status '%s'" % res.info.status)
        feasible = 0
        if res.info.status_val == 1 or res.info.status_val == 2 or res.info.status_val == -2:
            feasible = 1
        return res, feasible


    def compute_hyperplane(self, agents):

        P,q = self.build_QP()
        #TODO: Fix this
        G,h,A,b = self.build_cons(agents)

        res, feasible = self.osqp_solve_qp( P, q, G, h, A, b)

        idx = np.arange(0, 3)
        for i in range(1, self.horizon):
            aux = np.arange(0, 3) + i * self.planer_states
            idx = np.hstack((idx, aux))


        return np.reshape((res.x[idx]), (self.horizon, 3))