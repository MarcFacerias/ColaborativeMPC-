import time

from NL_EU_N_main import main
from config_files.config_NL import settings, path_gen, lbp_gen,save_config
import numpy as np 
import copy
# call main in a loop changing settings

# gains
set_vect = []
settings["verb"]= 0
settings["plot"]= -2
active_threads = 3

N = [30]
qvxl = [15, 35]
qeyl = [12,25]
qewl = [10]
qdUl = [100,150]
qUl  = [0]

for i,n in enumerate(N):
    for j,qvx in enumerate(qvxl):
        for k, qey in enumerate(qeyl):
            for l, qew in enumerate(qewl):
                for z, qdU in enumerate(qdUl):
                    for w, qU in enumerate(qUl):

                        settings["N"]= int(n)
                        settings["Q"]= np.diag([qvx, 0.0, 0.0, qey, qew, 0.0, 0.0, 0, 0])
                        settings["R"]= qU * np.diag([1, 5])
                        settings["dR"]= qdU * np.diag([3, 1])

                        path_gen(settings, "NL_3agents_" + str(i) + str(j) + str(k) + str(l) + str(z)+ str(w))
                        lbp_gen(settings, "lambdas")
                        save_config(settings)

                        main(copy.deepcopy(settings))
