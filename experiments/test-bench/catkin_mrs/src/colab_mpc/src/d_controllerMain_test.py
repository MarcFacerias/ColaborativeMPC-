#!/usr/bin/env python27

import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time

sys.path.append(sys.path[0]+'/DistributedControllerObject')
sys.path.append(sys.path[0]+'/Utilities')

from PathFollowingLPVMPC_distri import PathFollowingLPV_MPC, _buildMatEqConst
from trackInitialization import Map, wrap


def predicted_vectors_generation_V2(Hp, x0, dt, map, accel_rate = 0):
    # We need a prediction of the states for the start-up proces of the controller (To instantiate the LPV variables)
    # [vx vy psidot y_e thetae theta s x y ]

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

    Theta = np.zeros((Hp + 1, 1))
    Theta[0] = x0[5]
    X = np.zeros((Hp + 1, 1))
    X[0] = x0[7]
    Y = np.zeros((Hp + 1, 1))
    Y[0] = x0[8]

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
        X[i+1], Y[i+1], Theta[i+1] = map.getGlobalPosition(S[i], Ey[i])

    xx  = np.hstack([ Vx, Vy, W,Ey, Epsi, Theta ,S ,X,Y]) # [vx vy psidot y_e thetae theta s x y]
    uu = np.zeros(( Hp, 1 ))
    return xx, uu

# def test_dist(pa, pb, th):
#
#     if pa.shape[0] == pb.shape[0]:
#         state = np.zeros(pa.shape[0])
#         for row in range(0,pa.shape[0]):
#
#             if((pa[row,0] + pb[row,0])**2 + (pa[row,1] + pb[row,1])**2 < th):
#
#                 state[row] = 1
#
#
#         return state
#
#     return None

def main():

#########################################################
#########################################################

    # Class constructors
    map = Map()

    N = 10
    datContent_agent1 = [i.split( ) for i in open("/home/marc/Escritorio/results_simu_test/02_2023/Trajectory_generation/agent1/path.dat").readlines()]
    datContent_agent2 = [i.split( ) for i in open("/home/marc/Escritorio/results_simu_test/02_2023/Trajectory_generation/agent2/path.dat").readlines()]
    agents = np.empty((N,2,2))

    for k,i in enumerate(range(10,10+N)):
        agents[k,0,:] = [float(datContent_agent1[i][0]),float(datContent_agent1[i][1])]
        agents[k,1,:] = [float(datContent_agent2[i][0]),float(datContent_agent2[i][1])]

# TODO Update controller with multiagent version
# TODO: Generate trajectories to test one optimisation problem   
###----------------------------------------------------------------###

    Q  = np.diag([120.0, 1.0, 1.0, 70.0, 0.0, 1500.0])   #[vx ; vy ; psiDot ; e_psi ; s ; e_y]
    R  = 0.1 * np.diag([3, 0.8])                         #[delta ; a]

    Controller  = PathFollowingLPV_MPC(Q, R, N, 0.033,map, "OSQP")
    # 7.20272e-01 -1.61402e-01 -7.62251e-01 9.64952e-03 8.09713e-01 -1.83660e-03
    x0 = [0.720, -0.16, 0.00,-0.2, 0, 0.260, 8.09713e-01, 0.75, 1.0 ] #[vx vy psidot y_e thetae theta s x y]
    # uPred = np.array([-0.154, 2.491])
    Last_xPredicted = np.array(x0)



    Last_xPredicted, uPred = predicted_vectors_generation_V2(N, Last_xPredicted, 0.033 , map)
    t = time.time()
    lambdas = 0.25*np.ones((2,N)) # one lambda per neighbour per optimisation within the horizon
    feas, Solution = Controller.solve(x0, Last_xPredicted, uPred, False, "A_L", "B_L" ,"C_L", 4,lambdas, agents )

    # print(Solution)
    # test_dist(Controller.xPred[:-1,[7,8]], agents[:,0,:] ,1.5**2)
    print(time.time() - t)
    # print(Controller.uPred)
    # print(Controller.xPred)
    data = Controller.xPred[:-1,[7,8]]
    # Note that even in the OO-style, we use `.pyplot.figure` to create the Figure.
    fig, ax = plt.subplots()
    ax.plot(data[:,0], data[:,1], label='data')  # Plot some data on the axes.
    ax.plot(agents[:,0,0], agents[:,0,1], label='agent 1')  #
    ax.plot(agents[:,1,0], agents[:,1,1], label='agent 1')  # Plot some data on the axes.
    ax.set(xlim=(-2, 2), ylim=(-2, 2))
    ax.set_xlabel('x label')  # Add an x-label to the axes.
    ax.set_ylabel('y label')  # Add a y-label to the axes.
    ax.set_title("Simple Plot")  # Add a title to the axes.
    ax.legend()  # Add a legend.
    plt.show()

if __name__ == "__main__":

    main()




