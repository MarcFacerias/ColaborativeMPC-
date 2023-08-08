import os.path
import pickle
import warnings

import numpy as np


class experiment_utilities():

    def __init__(self, data, settings, model = "SCALED CAR"):

        self.path_csv = settings["path_csv"]
        self.path_pck = settings["path_pck"]
        self.data = data
        self.uPred_hist = []
        self.sPred_hist = []
        self.look_ahead = []

        if model == "SCALED CAR":
            self.model_param = {
                "lf" : 0.125,
                "lr" : 0.125,
                "m"  : 1.98,
                "I"  : 0.09,
                "Cf" : 70.0,
                "Cr" : 70.0,
                "mu" : 0.05
            }

            self.sys_lim     = {
                "vx_ref" : settings["vx_ref"],
                "min_dist" : 0.25,
                "max_vel"  : 5.5,
                "min_vel"  : 0,
                "max_rs" : 0.3,
                "max_ls" : 0.3,
                "max_ac" : 5.0,
                "max_dc" : 10.0,
                "sm"     :0.9,
                "LPV": True
            }

        else:
            self.sys_lim = None
            self.model_param = None

    def save(self, xPred, uPred= None, feas = None, planes = None):

        self.data.states.append(xPred[0,:])
        self.sPred_hist.append(xPred)
        self.look_ahead.append(xPred[-1,6] - xPred[0,6])

        if uPred is not None:
            self.data.u.append(uPred[0,:])
            self.uPred_hist.append(uPred)

        if planes is not None:
            self.data.planes.append(planes[0,:])

        if feas is not None:
            self.data.status.append(feas)


    def save_to_csv(self, OCD_it = None):

        parent_path = self.path_csv + "csv/"

        if not os.path.exists(parent_path):
            os.makedirs(parent_path, exist_ok=True)

        path = parent_path + str(self.data.id)

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

        np.savetxt(path + '/states.dat', self.data.states, fmt='%.5e',delimiter=' ')
        np.savetxt(path + '/u.dat', self.data.u, fmt='%.5e', delimiter=' ')
        np.savetxt(path + '/time.dat', self.data.time_op, fmt='%.5e', delimiter=' ')
        np.savetxt(path + '/plan_dist.dat', self.data.look_ahead, fmt='%.5e', delimiter=' ')

        if OCD_it is not None:
            try:
                np.savetxt(path + '/time_it.dat', self.time_OCD(OCD_it), fmt='%.5e', delimiter=' ')
                np.savetxt(path + '/OCD_it.dat', OCD_it, fmt='%.5e', delimiter=' ')
            except Exception as e:
                print(e)

    def save_var_to_csv(self, var, name):

        parent_path = self.path_csv + "csv/"

        if not os.path.exists(parent_path):
            os.makedirs(parent_path, exist_ok=True)

        np.savetxt(parent_path + '/' + str(name) + '.dat', var, fmt='%.5e',delimiter=' ')


    def save_var_pickle(self, vars , tags = None):

        path = self.path_pck + "pck/"

        if not os.path.exists(path):
            os.makedirs(path, exist_ok=True)

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

            with open(path + '/u.pkl', 'wb') as f1:  # Python 3: open(..., 'wb')
                pickle.dump(self.data.uPred_hist, f1)


    def time_OCD(self,OCD_it):

        lim = np.max(np.array(OCD_it))
        placeholder = np.zeros((len(OCD_it),lim))

        for i in range(0,len(OCD_it)):
            placeholder[i,:] = self.data.time_op[i*lim:(i+1)*lim]

        return placeholder


