# Imports
import numpy as np
from plan_lib.config import experiment_utilities

class initialiserLPV(experiment_utilities):
    def __init__(self, data, settings, model = "SCALED CAR"):
        super().__init__(data, settings, model)

        self.Qs = settings["Qs"]
        self.Q  = settings["Q"]
        self.R  = settings["R"]
        self.dR = settings["dR"]
        self.wq = settings["wq"]