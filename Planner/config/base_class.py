import numpy as np
import pickle
import os
import warnings
import sys
import time

sys.path.append(sys.path[0]+'/plotter')
from plot_tools import *

class experiment_utilities():

    def __init__(self, data, path_csv, path_pck):

        self.path_csv = path_csv
        self.path_pck = path_pck
        self.data = data
        self.uPred_hist = []
        self.sPred_hist = []

    def save(self, xPred, uPred, planes, feas):

        self.data.states.append(xPred[0,:])
        self.data.u.append(uPred[0,:])
        self.data.planes.append(planes[0,:])
        self.data.status.append(feas)

        self.uPred_hist.append(uPred)
        self.sPred_hist.append(xPred)

    def save_to_csv(self):

        parent_path = self.path_csv + "csv/"

        if not os.path.exists(parent_path):
            os.makedirs(parent_path, exist_ok=True)

        path = parent_path + str(self.data.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path + '/states.dat', self.data.states, fmt='%.5e',delimiter=' ')
        np.savetxt(path + '/u.dat', self.data.u, fmt='%.5e', delimiter=' ')
        np.savetxt(path + '/time.dat', self.data.time_op, fmt='%.5e', delimiter=' ')

    def save_var_to_csv(self,var, name):

        parent_path = self.path_csv + "csv/"

        if not os.path.exists(parent_path):
            os.makedirs(parent_path, exist_ok=True)

        path = parent_path + str(self.data.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path + '/' + str(name) + '.dat', var, fmt='%.5e',delimiter=' ')


    def save_var_pickle(self, vars = None, tags = None):

        parent_path = self.path_csv + "pck/"

        if not os.path.exists(parent_path):
            os.makedirs(parent_path, exist_ok=True)

        path = parent_path + str(self.data.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        if vars is None:
                with open(path + '/u.pkl', 'wb') as f1:  # Python 3: open(..., 'wb')
                    pickle.dump(self.data.uPred_hist, f1)
                with open(path + '/states.pkl', 'wb') as f2:  # Python 3: open(..., 'wb')
                    pickle.dump(self.data.sPred_hist, f2)
        else:

            for i, var in enumerate(vars):

                try:
                    with open(path + '/' + tags[i] + '.pkl', 'wb') as f1:  # Python 3: open(..., 'wb')
                        pickle.dump(var, f1)

                except:
                    with open(path +'/def' + str(i) + '.pkl', 'wb') as f2:  # Python 3: open(..., 'wb')
                        pickle.dump(var, f2)
                        msg = "WARNING no name asigned !"
                        warnings.warn(msg)

    def save_exp(self):
        parent_path = self.path_csv + "pck/"

        if not os.path.exists(parent_path):
            os.makedirs(parent_path, exist_ok=True)

        path = parent_path + str(self.data.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

            with open(path + '/states.pkl', 'wb') as f2:  # Python 3: open(..., 'wb')
                pickle.dump(self.data.sPred_hist, f2)




