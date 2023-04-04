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

    def __init__(self, n_agents, horizon):

        self.n_agents = n_agents
        self.horizon = horizon
        self.plane_states = 3
        self.slack_vars = 1
        self.planer_states = self.plane_states+self.slack_vars * (n_agents-1)

    def compute_hyperplane(self, agents, pose):
        # Case with only 1 neighbour
        placeholder = np.zeros(((self.horizon, 3, self.n_agents)))

        for h in range(0,self.horizon):

            for n in range(0, self.n_agents):

                x_ego = pose[h,:]
                x_neg = agents[h,n,:]

                a =  x_neg - x_ego
                b = 0.5 * a@(x_ego + x_neg).T
                placeholder[h,0,n] = a[0]
                placeholder[h,1,n] = a[1]
                placeholder[h,2,n] = b


        return placeholder