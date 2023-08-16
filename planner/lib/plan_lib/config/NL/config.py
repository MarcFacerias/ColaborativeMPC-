# Imports
import numpy as np
from plan_lib.config import experiment_utilities

def get_alpha():

    alpha = 0.25
    return alpha

class initialiserNL(experiment_utilities):
    def __init__(self, data, settings, model = "SCALED CAR"):
        super().__init__(data, settings, model)
        self.Qs = settings["Qs"]
        self.Q  = settings["Q"]
        self.R  = settings["R"]
        self.dR = settings["dR"]
        self.wq = settings["wq"]

def eval_constraintEU(x1, x2, D):

    cost1 = D - np.sqrt(sum((x1-x2)**2)) # the OCD update depends on on the diference between the minimum D and the euclidean dist

    return np.array(cost1)

def eval_constraintHp(x2, planes, D):

    cost1 = -planes[0] * x2[0] - planes[1] * x2[1] - planes[2] + D/2

    return np.array(cost1)