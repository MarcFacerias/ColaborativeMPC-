# Imports
import matplotlib.colors as mcolors
import numpy as np
plot = False
plot_end = True
verb = True
verb_OCD = False
color_list = list(mcolors.TABLEAU_COLORS)
n_agents = 2
it_conv = 1

max_it = 1000
max_it_OCD = 10
N = 10
dt = 0.01
dth = 0.25

x0_database = [""] * 4
x0_database[0] = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.45]  # [vx vy psidot y_e thetae theta s x y]
x0_database[1] = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]
x0_database[2] = [1.3, -0.16, 0.00, 0.25, 0, 0.0, 0.25, 0.0, 1.5]  # [vx vy psidot y_e thetae theta s x y]
x0_database[3] = [1.3, -0.16, 0.00, -0.25, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]

# map_type = "Highway"
map_type = "oval"

path_csv = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/DistributedPlanner/TestsPaperL4/"
path_img = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/DistributedPlanner/TestsPaperL4/"

def get_alpha():

    alpha = 0.25
    return alpha

class initialiserNL_EU():
    def __init__(self, model = "SCALED CAR"):
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
                "vx_ref" : 6.0,
                "radius" : 0.45,
                "max_vel"  : 10,
                "min_vel"  : 0.2,
                "max_rs" : 0.45,
                "max_ls" : 0.45,
                "max_ac" : 8.0,
                "max_dc" : 8.0,
                "sm"     : 0.8
            }

        else:
            self.sys_lim = None
            self.model_param = None

        self.Qs = np.diag([10000000,1000000,1000000])
        self.Q = np.diag([10.0, 0.0, 0.0, 200.0, 50.0, 0.0, 0.0, 0, 0])
        self.R = 50 * np.diag([1, 1])