#!/usr/bin/env python
"""
    File name: Learning-LPV-MPC.py
    Author: Eugenio Alcala
    Email: eugenio.alcala@upc.edu.edu
    Date: 09/30/2018
    Python Version: 2.7.12
"""

import sys
import datetime
import rospy
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(sys.path[0]+'/ControllerObject')
sys.path.append(sys.path[0]+'/Utilities')

from lpv_mpc.msg import ECU, prediction, Racing_Info, My_Planning
from std_msgs.msg import Bool, Int16, UInt8
from utilities import wrap
from dataStructures import EstimatorData
from PathFollowingLPVMPC import PathFollowingLPV_MPC, ABC_computation_5SV
from trackInitialization_mod import Map, wrap

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


def main(group):

    # ROS PUBLISHERS
    input_commands  = rospy.Publisher('ecu', ECU, queue_size=1)
    pub_flag        = rospy.Publisher('flag', Bool, queue_size=1)
    controlVel_commands  = rospy.Publisher('/manual_control/speed',Int16,queue_size=1)
    controlSter_commands = rospy.Publisher('steering',UInt8,queue_size=1)

    # TODO: Publish Predicted path

    # END ROS PUBLISHERS

    # ROS SUBSCRIBERS

    # TODO: Subscribe to neighbouring agents predictions

    # END ROS SUBSCRIBERS

    # ROS PARAM
    N               = rospy.get_param("control/N")
    Vx_ref          = rospy.get_param("control/vel_ref")
    loop_rate = rospy.get_param("control/Hz")
    # END ROS PARAM

    #  ROS handlers
    rospy.init_node("LPV-MPC")
    dt              = 1.0/loop_rate
    rate            = rospy.Rate(loop_rate)
    # END ROS handlers



#########################################################
#########################################################

    #  msg initialisation
    rise_flag       = Bool()
    cmd             = ECU()
    cmd.servo       = 0.0
    cmd.motor       = 0.0

    # Class constructors
    map = Map()
    estimatorData = EstimatorData(group)

    # Variales and flags
    NN_LPV_MPC      = False
    first_it        = 1
    Pub_counter     = 0

    # Initialize variables for main loop
    GlobalState     = np.zeros(6)
    LocalState      = np.zeros(6)
    RunController   = 1

    vector_length   = 42000
    DATA            = np.zeros((vector_length,8))      # [vx vy psidot thetae s ye vxaccel vyaccel udelta uaccel]
    REFS            = np.zeros((vector_length,1))      # [refVx]
    GLOBAL_DATA     = np.zeros((vector_length,3))       # [x y psi]
    PREDICTED_DATA  = np.zeros((vector_length,120))     # [vx vy psidot thetae s ye] presicted to N steps
    ELAPSD_TIME     = np.zeros((vector_length,1))
    CONTROL_ACTIONS = np.zeros((vector_length,2))
    Hist_pos        = np.zeros((vector_length,4))
    Hist_traj       = np.zeros((vector_length,4))

    vel_ref = np.zeros([N,1])
    curv_ref = np.zeros([N,1])
    y_ref = np.zeros([N,1])
    yaw_ref = np.zeros([N,1])
    x_ref = np.zeros([N,1])

    # Loop running at loop rate
    TimeCounter     = 0
    PlannerCounter  = 0
    acc_ref         = 1.5
    dacc_ref        = 0
    velocity        = 0
    rise_flag.data  = True
    wheel_diameter  = 6.3/100

    rospy.sleep(1.2)   # Soluciona los problemas de inicializacion esperando a que el estimador se inicialice bien

    Counter   = 0

# TODO Update controller with multiagent version
###----------------------------------------------------------------###

    Q  = np.diag([120.0, 1.0, 1.0, 70.0, 0.0, 1500.0])   #[vx ; vy ; psiDot ; e_psi ; s ; e_y]
    R  = 1 * np.diag([3, 0.8])                         #[delta ; a]
    dR = 15 * np.array([1.0, 1.5])                        #Input rate cost u

    Controller  = PathFollowingLPV_MPC(Q, R, dR, N, Vx_ref, dt,map, "OSQP", Steering_Delay, Velocity_Delay)

###----------------------------------------------------------------###

    LPV_States_Prediction   = np.zeros((N,6))

    pub_flag.publish(rise_flag)
    print("Controller is running")

    while (not rospy.is_shutdown()) and RunController == 1: 

        # Read Measurements
        ct_start = datetime.datetime.now() 
        GlobalState[:] = estimatorData.CurrentState  # The current estimated state vector [vx vy w x y psi]
        LocalState[:]  = estimatorData.CurrentState  # [vx vy w x y psi]

        GLOBAL_DATA[TimeCounter,:] = LocalState[3::] # store state wrt to track
        LocalState[4], LocalState[5], LocalState[3], insideTrack = map.getLocalPosition(GlobalState[3], GlobalState[4], GlobalState[5],0) # get state wrt to track

        # Initalise first reference
        if (LocalState[4] <= 3*map.TrackLength[0]/4):

            for i in range(0,N):

                if i == N-1:
                    vel_ref[i]  = vel_ref[i-1] + acc_ref*dt

                    if vel_ref[i] > Vx_ref:
                        vel_ref[i] = Vx_ref

                else:
                    vel_ref[i] = vel_ref[i+1]

        else:

            for i in range(0,N):

                if i == N-1:

                    vel_ref[i,0]  = vel_ref[i-1,0] + dacc_ref*dt

                    if vel_ref[i,0] <= 0:
                        vel_ref[i,0] = 0

                else:
                    vel_ref[i,0] = vel_ref[i+1,0]

        REFS[TimeCounter,0] =  vel_ref[0,0]

        uApplied = np.array([cmd.servo, cmd.motor]) # experiment u record

        # Control action historic that is filled along the horizon (N spaces)
        Controller.OldSteering.append(cmd.servo)
        Controller.OldAccelera.append(cmd.motor)
        Controller.OldSteering.pop(0)
        Controller.OldAccelera.pop(0)

        Hist_pos[TimeCounter,:]   = [LocalState[3], LocalState[4], wrap(LocalState[5]), LocalState[0] ]
        Hist_traj[TimeCounter,:]  = [x_ref[0,0], y_ref[0,0], wrap(yaw_ref[0,0]), vel_ref[0,0] ]        

        GlobalState[5] = wrap(GlobalState[5])

        ###################################################################################################
        ###################################################################################################

        # Warm up cases for the two first iterations
        if first_it < 2:

            vel_ref     = 0*np.ones([N,1])
            accel_rate = 0.0
            xx, uu      = predicted_vectors_generation_V2(N, LocalState, accel_rate, dt)
            Controller.solve(LocalState[0:6], xx, uu, NN_LPV_MPC, vel_ref,curv_ref, 0, 0, 0, first_it)
            first_it    = first_it + 1
            Controller.OldPredicted = np.hstack((Controller.OldSteering[0:len(Controller.OldSteering)-1], Controller.uPred[Controller.steeringDelay:Controller.N,0]))
            Controller.OldPredicted = np.concatenate((np.matrix(Controller.OldPredicted).T, np.matrix(Controller.uPred[:,1]).T), axis=1)

        else:

            LPV_States_Prediction, A_L, B_L, C_L = Controller.LPVPrediction(LocalState[0:6], Controller.uPred, vel_ref, curv_ref)
            Controller.solve(LPV_States_Prediction[0,:], LPV_States_Prediction, Controller.uPred, NN_LPV_MPC, vel_ref, curv_ref, A_L, B_L, C_L, first_it)

        ###################################################################################################
        ###################################################################################################

        if Counter >-1:
            if first_it > 19:
                new_LPV_States_Prediction = LPV_States_Prediction[0, :]
                for i in range(1,N):
                    new_LPV_States_Prediction = np.hstack((new_LPV_States_Prediction, LPV_States_Prediction[i,:]))
                PREDICTED_DATA[Counter,:] = new_LPV_States_Prediction


            CONTROL_ACTIONS[TimeCounter,:] = [cmd.servo, cmd.motor]
            DATA[TimeCounter,:]   = np.hstack((LocalState, uApplied))

            endTimer = datetime.datetime.now();
            deltaTimer = endTimer-ct_start

            ELAPSD_TIME[Counter,:] = deltaTimer.total_seconds()

            TimeCounter     += 1
            Pub_counter     += 1 
            
            ## Publish input simulation IRI ##
            cmd.servo = Controller.uPred[0,0]
            cmd.motor = Controller.uPred[0,1]
            
            Pub_counter = 0
            PlannerCounter  += 1

            # Stop the vehicle if the controller fails
            if cmd.motor is None:
            	cmd.motor = 0
            if cmd.servo is None:
            	cmd.servo = 0

            input_commands.publish(cmd)
            velocity += cmd.motor * dt
            inputVel  = (velocity/(wheel_diameter/2)) * (60/(2*np.pi))
            inputVel  = sorted([-1000, inputVel, 1000])[1]
            inputSter = 90 - 1.3*((cmd.servo*180)/np.pi)
            controlVel_commands.publish(inputVel)
            controlSter_commands.publish(inputSter)


        Counter  += 1
        rate.sleep()

    # LOOING VARIABLES: UNCOMENT FOR PLOTTING AND SAVING DATA
    '''
    plt.figure(2)
    plt.subplot(611)
    plt.plot(CONTROL_ACTIONS[0:TimeCounter,0], '-')
    plt.legend(['Steering'], loc='best')
    plt.grid()

    plt.subplot(612)
    plt.plot(CONTROL_ACTIONS[0:TimeCounter,1], '-')
    plt.legend(['Acceleration'], loc='best')
    plt.grid()

    plt.subplot(613)
    plt.plot(Hist_traj[0:TimeCounter,1], '.')
    # plt.legend([], loc='best') #Hist_pos[:,0],Hist_pos[:,1],Hist_traj[:,0],Hist_traj[:,1]
    plt.subplot(613)
    plt.plot(Hist_pos[0:TimeCounter,1], '.')
    plt.legend(['Trajectory Y',' Y'], loc='best')
    plt.grid()

    plt.subplot(614)
    plt.plot(Hist_traj[0:TimeCounter,3], '.')
    # plt.legend([], loc='best') #Hist_pos[:,0],Hist_pos[:,1],Hist_traj[:,0],Hist_traj[:,1]
    plt.subplot(614)
    plt.plot(Hist_pos[0:TimeCounter,3], '.')
    plt.legend(['Trajectory Vel',' Vel'], loc='best')
    plt.grid()

    plt.subplot(615)
    plt.plot(Hist_traj[0:TimeCounter,2], 'r.')
    #plt.legend([], loc='best') #Hist_pos[:,0],Hist_pos[:,1],Hist_traj[:,0],Hist_traj[:,1]
    plt.subplot(615)
    plt.plot(Hist_pos[0:TimeCounter,2], 'g.')
    plt.legend(['Trajectory yaw','yaw'], loc='best')
    plt.grid()

    plt.subplot(616)
    plt.plot(DATA[0:TimeCounter,5], '-')
    plt.legend(['Lateral error'], loc='best')
    plt.grid()

    plt.show()

    day         = '28_1_2019'
    num_test    = 'ResultsPaper'
    newpath     = '/home/marc/Escritorio/results_simu_test/'+day+'/'+num_test+'/'
    if not os.path.exists(newpath):
        os.makedirs(newpath)
    np.savetxt(newpath+'/DATA2.dat', DATA, fmt='%.5e')
    np.savetxt(newpath+'/REFS2.dat', REFS, fmt='%.5e') 
    np.savetxt(newpath+'/GLOBAL_DATA2.dat', GLOBAL_DATA, fmt='%.5e')
'''
    # END OF LOOING VARIABLES: UNCOMENT FOR PLOTTING AND SAVING DATAs
    quit()

# ===============================================================================================================================
# ==================================================== END OF MAIN ==============================================================
# ===============================================================================================================================


def predicted_vectors_generation_V2(Hp, x0, dt, accel_rate = 0):
    # We need a prediction of the states for the start-up proces of the controller (To instantiate the LPV variables)

    Vx      = np.zeros((Hp+1, 1))
    Vx[0]   = x0[0]
    S       = np.zeros((Hp+1, 1))
    S[0]    = 0
    Vy      = np.zeros((Hp+1, 1))
    Vy[0]   = x0[1]
    W       = np.zeros((Hp+1, 1))
    W[0]    = x0[2]
    Ey      = np.zeros((Hp+1, 1))
    Ey[0]   = x0[3]
    Epsi    = np.zeros((Hp+1, 1))
    Epsi[0] = x0[4]

    Accel   = 1.0

    for i in range(0, Hp):
        Vy[i+1]      = x0[1]
        W[i+1]       = x0[2]
        Ey[i+1]      = x0[3]
        Epsi[i+1]    = x0[4]

    Accel   = Accel + np.array([ (accel_rate * i) for i in range(0, Hp)])

    for i in range(0, Hp):
        Vx[i+1]    = Vx[i] + Accel[i] * dt
        S[i+1]      = S[i] + Vx[i] * dt

    xx  = np.hstack([ Vx, Vy, W, Epsi ,S ,Ey]) # [vx vy omega theta_e s y_e]
    uu = np.zeros(( Hp, 1 ))
    return xx, uu



if __name__ == "__main__":

    try:
        myargv = rospy.myargv(argv=sys.argv)
        main(myargv[1])

    except rospy.ROSInterruptException:
        pass
