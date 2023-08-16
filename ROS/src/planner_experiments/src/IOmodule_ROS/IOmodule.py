
import time

class io_class_ROS():

    def __init__(self, settings, system):

        self.verb = settings["verb"]
        if "verb_OCD" in settings.keys():
            self.verb_OCD = settings["verb_OCD"]

        self.n_agent = settings["n_agents"]
        self.color = settings["color_list"]
        self.save = settings["save_data"]
        self.path = settings["path_img"]

        self.sys  = system # we store the robot class
        self._tic = 0
        self._et = 0
        self.et_hist = []
        self.it_count = 0
        self.it_plot = None
        self.it_OCD = []

        # flag to decide whether or not info is displayed
        if self.sys.id == settings["log_agent"]:
            self.act_flag = True
        else:
            self.act_flag = False


    def tic(self):
        self._tic = time.time()
    def toc(self):
        self._et = time.time() - self._tic
        self.et_hist.append(self._et)

    def updateOCD(self,x_pred,it_OCD,it):

        if not self.act_flag:
            return

        if self.verb_OCD:

            print("-------------------------------------------------")
            print("it " + str(it))
            print("length OCD" + str(it_OCD))
            for i in range(0, self.n_agent):
                print("---------------------Agents---------------------------------------")
                print("Agent " + str(i) + " track s: " + str(x_pred[i][1, 6]))
            print("-------------------------------------------------")

    def update(self, x_pred, u_pred ,agents, it, error = False, OCD_ct = None, end = False):

        self.it_count += 1

        track_len = str(self.sys.map.TrackLength[0])

        # Save OCD iteration data
        if OCD_ct is not None:
            self.it_OCD.append(OCD_ct)

        # After the experiment or errorsave all the data
        if (self.save and end) or (self.save and error ):

            self.sys.save_to_csv(self.it_OCD)
            self.sys.save_exp()
            self.sys.save_var_to_csv(self.et_hist, "ROS_et_hist")

        # Diferent prints depending on the verbosity definition
        if not self.act_flag:
            return

        if self.verb == 1:

            print("--------------------------------------------------------------")
            print("it: " + str(it))

            print("---------------------Agents---------------------------------------")
            print("Agent " + " track s: " + str(x_pred[1, -3]) + "/" + track_len)

            print("---------------------END Agents---------------------------------------")
            print("avg computational time: " + str((self._et)/self.sys.n_agents))
            print("--------------------------------------------------------------")

        elif self.verb == 2 :
            print("--------------------------------------------------------------")
            print("it: " + str(it))
            if OCD_ct is not None:
                print("length " + str(OCD_ct))
            print("agents x : " + str(agents[0,:,0]))
            print("agents y : " + str(agents[0,:,1]))

            print("---------------------Agents---------------------------------------")
            print("Agent "  + " track s: " + str(x_pred[0,-3]) + "/" + track_len)
            print("Agent "  + " u0: " + str(u_pred[0,0]) + " u1: " + str(u_pred[0,1]))
            print("Agent "  + " v: " + str(x_pred[1, 0]) + " ey: " + str(x_pred[1,3]))

            print("---------------------END Agents---------------------------------------")
            print("avg computational time: " + str((self._et)/self.n_agent))
            print("--------------------------------------------------------------")

        else:

            print("--------------------------------------------------------------")
            print("it: " + str(it))
            print("--------------------------------------------------------------")

