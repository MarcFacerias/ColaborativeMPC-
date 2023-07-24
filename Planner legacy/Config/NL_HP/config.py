# Imports
import matplotlib.colors as mcolors
import numpy as np
plot = False
plot_end = True
verb = False
verb_OCD = False
color_list = list(mcolors.TABLEAU_COLORS)
n_agents = 4
it_conv = 1

max_it = 500
max_it_OCD = 10
N = 10
dt = 0.01
dth = 0.25

x0_database = [""] * 4
x0_database[0] = [1.3, -0.16, 0.00, 0.45, 0, 0.0, 0, 0.0, 1.45]  # [vx vy psidot y_e thetae theta s x y]
x0_database[1] = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]
x0_database[2] = [1.3, -0.16, 0.00, 0.25, 0, 0.0, 0.25, 0.0, 1.5]  # [vx vy psidot y_e thetae theta s x y]
x0_database[3] = [1.3, -0.16, 0.00, -0.25, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]

# map_type = "Highway"
map_type = "Oval2"

def get_alpha():

    alpha = 0.25
    return alpha

class initialiserNL_HP():
    def __init__(self, model = "CAR"):
        if model == "SCALED CAR":
            self.model_param = {
                "lf" : 0.12,
                "lr" : 0.14,
                "m"  : 2.250,
                "I"  : 0.06,
                "Cf" : 60.0,
                "Cr" : 60.0,
                "mu" : 0.0
            }

            self.sys_lim     = {
                "vx_ref" : 4.5,
                "max_vel"  : 6,
                "min_vel"  : -6,
                "max_rs" : 0.45,
                "max_ls" : 0.45,
                "max_ac" : 8.0,
                "max_dc" : 8.0,
                "sm"     : 0.9
            }

        else:
            self.sys_lim = None
            self.model_param = None

        self.Qs = np.diag([10000000,1000000,1000000,1000000])
        self.Q =  np.diag([120.0, 1.0, 1.0, 1500.0, 70.0, 0.0, 0.0, 0, 0])
        self.R =  0.01 * np.diag([1, 1])