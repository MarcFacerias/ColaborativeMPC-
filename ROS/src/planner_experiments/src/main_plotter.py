# TODO check this

# ROS libs
import sys,ropsy

from Planner.packages.plotter_ROS import plotter_ROS


if __name__ == "__main__":
    myargv = rospy.myargv(argv=sys.argv)
    try:
        mode = myargv[1]
    except:
        mode = None # TODO add error handling here 

    plotter = plotter_ROS(mode)
    plotter.run()



