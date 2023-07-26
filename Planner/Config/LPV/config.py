# Imports
import matplotlib.colors as mcolors
import numpy as np

plot = False
plot_end = True
save_data = True
verb_level = 2
color_list = list(mcolors.TABLEAU_COLORS)
n_agents = 1

x0_database = [""] * 4
x0_database[0] = [1.3, -0.16, 0.00, 0.25, 0, 0.0, 0.25, 0.0, 1.5]  # [vx vy psidot y_e thetae theta s x y]
x0_database[1] = [1.3, -0.16, 0.00, -0.25, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]
x0_database[2] = [1.3, -0.16, 0.00, 0.45, 0, 0.0, 0, 0.0, 1.45]  # [vx vy psidot y_e thetae theta s x y]
x0_database[3] = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]


max_it = 1000
N = 25
dt = 0.01

# map_type = "Highway"
map_type = "oval"
# map_type = "SL"


path_csv = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/DistributedPlanner/TestsPaperL4/"
path_img = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/DistributedPlanner/TestsPaperL4/"

class initialiserLPV():
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
                "vx_ref" : 3.5,
                "min_dist" : 0.25,
                "max_vel"  : 4,
                "min_vel"  : 1.1,
                "max_rs" : 0.45,
                "max_ls" : 0.45,
                "max_ac" : 6.0,
                "max_dc" : 4.0,
                "sm"     :0.8
            }

        else:
            self.sys_lim = None
            self.model_param = None

        self.Qs = 10000000 * np.eye(3)
        self.Q  = np.diag([120.0, 1.0, 1.0, 300.0, 20.0, 0.0, 0.0, 0, 0])
        self.R  = 1 * np.diag([1, 1])
        self.dR = 200 * np.diag([1, 1])
        self.wq = 0.0