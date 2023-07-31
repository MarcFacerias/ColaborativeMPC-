# Imports
import matplotlib.colors as mcolors
import numpy as np

plot = False
plot_end = True
save_data = False
verb_level = 0
color_list = list(mcolors.TABLEAU_COLORS)
n_agents = 1

x0_database = [""] * 4
x0_database[3] = [1.3, -0.16, 0.00, 0.25, 0, 0.0, 0.25, 0.0, 1.5]  # [vx vy psidot y_e thetae theta s x y]
x0_database[1] = [1.3, -0.16, 0.00, -0.25, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]
x0_database[2] = [1.3, -0.16, 0.00, 0.45, 0, 0.0, 0, 0.0, 1.45]  # [vx vy psidot y_e thetae theta s x y]
x0_database[0] = [1.3, -0.16, 0.00, 0.0, 0, 0.0, 0, 0.0, 1.0]  # [vx vy psidot y_e thetae theta s x y]


max_it = 300
N = 10
dt = 0.01

# map_type = "Highway"
map_type = "oval"
# map_type = "SL"


path_csv = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/DistributedPlanner/TestsPaperL4/"
path_img = "/home/marc/git_personal/colab_mpc/ColaborativeMPC-/Planner/DistributedPlanner/TestsPaperL5/"

class initialiserLPV():
    def __init__(self, model = "SCALED CAR"):
        if model == "SCALED CAR":
            self.model_param = {
                "lf" : 0.125,
                "lr" : 0.125,
                "m"  : 1.98,
                "I"  : 0.06,
                "Cf" : 60.0,
                "Cr" : 60.0,
                "mu" : 0.05
            }

            self.sys_lim     = {
                "vx_ref" : 4.5,
                "min_dist" : 0.25,
                "max_vel"  : 5.5,
                "min_vel"  : 0.2,
                "max_rs" : 0.45,
                "max_ls" : 0.45,
                "max_ac" : 4.0,
                "max_dc" : 3.0,
                "sm"     :0.9
            }

        else:
            self.sys_lim = None
            self.model_param = None

        self.Qs = 10000000 * np.eye(3)
        self.Q  = np.diag([10.0, 0.0, 0.0, 200.0, 50.0, 0.0, 0.0, 0, 0])
        self.R  = 0 * np.diag([1, 1])
        self.dR = 50 * np.diag([1, 1])
        self.wq = 0.0