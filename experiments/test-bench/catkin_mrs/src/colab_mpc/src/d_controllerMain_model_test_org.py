#!/usr/bin/env python27

import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time
import math

sys.path.append(sys.path[0]+'/ControllerObject')
sys.path.append(sys.path[0]+'/Utilities')

from trackInitialization import Map, wrap
from plot_vehicle import *
from utilities import Curvature
plot = True
it = True


def OneStepSim(map,states, u):
    # Vehicle parameters:
    lf = 0.12
    lr = 0.14
    m = 2.250
    I = 0.06
    Cf = 60.0
    Cr = 60.0
    mu = 1
    dt = 0.1

    vx = states[0]
    vy = states[1]
    epsi = states[3]
    s = states[4]
    ey = states[5]

    cur = Curvature(s, map)
    print(cur)


    delta = float(u[0])  # EA: steering angle at K-1

    # standard model
    A12 = (np.sin(delta) * Cf) / (m * vx)
    A13 = (np.sin(delta) * Cf * lf) / (m * vx) + vy

    A22 = -(Cr + Cf * np.cos(delta)) / (m * vx)
    A23 = -(lf * Cf * np.cos(delta) - lr * Cr) / (m * vx) - vx

    A32 = -(lf * Cf * np.cos(delta) - lr * Cr) / (I * vx)
    A33 = -(lf * lf * Cf * np.cos(delta) + lr * lr * Cr) / (I * vx)

    B11 = -(np.sin(delta) * Cf) / m
    B12 = 1

    A11 = -mu
    A51 = (1 / (1 - ey * cur)) * (-cur)
    A52 = (1 / (1 - ey * cur)) * (np.sin(epsi) * cur)

    A61 = np.cos(epsi) / (1 - ey * cur)
    A62 = -np.sin(epsi) / (1 - ey * cur)
    # A7 = np.cos(epsi)
    A8 = vx

    B21 = (np.cos(delta) * Cf) / m
    B31 = (lf * Cf * np.cos(delta)) / I

    A = np.array([[A11, A12, A13, 0., 0., 0.],  # [vx]
                   [0., A22, A23, 0., 0., 0.],  # [vy]
                   [0., A32, A33, 0., 0., 0.],  # [wz]
                   [A51, A52, 1., 0., 0., 0.],  # [epsi]
                   [A61, A62, 0., 0., 0., 0.],  # [s]
                   [0, 1, 0., A8, 0., 0.]])  # [ey]

    B = np.array([[B11, B12],  # [delta, a]
                   [B21, 0],
                   [B31, 0],
                   [0, 0],
                   [0, 0],
                   [0, 0]])

    Ai = np.eye(len(A)) + dt * A
    Bi = dt * B

    states_new = np.dot(Ai, states) + np.dot(Bi, u)

    return states_new,A, B

def predicted_vectors_generation_V2(Hp, x0, accel_rate, dt):

    Vx      = np.zeros((Hp+1, 1))
    Vx[0]   = x0[0]
    S       = np.zeros((Hp+1, 1))
    S[0]    = 0
    Vy      = np.zeros((Hp+1, 1))
    Vy[0]   = x0[1]
    W       = np.zeros((Hp+1, 1))
    W[0]    = x0[2]
    Ey      = np.zeros((Hp+1, 1))
    Ey[0]   = x0[5]
    Epsi    = np.zeros((Hp+1, 1))
    Epsi[0] = x0[3]

    Accel   = 1.0

    for i in range(0, Hp):
        Vy[i+1]      = x0[1]
        W[i+1]       = x0[2]
        Ey[i+1]      = x0[5]
        Epsi[i+1]    = x0[3]

    Accel   = Accel + np.array([ (accel_rate * i) for i in range(0, Hp)])

    for i in range(0, Hp):
        Vx[i+1]    = Vx[i] + Accel[i] * dt
        S[i+1]      = S[i] + Vx[i] * dt

    xx  = np.hstack([ Vx, Vy, W, Epsi ,S ,Ey]) # [vx vy omega theta_e s y_e]
    uu = np.zeros(( Hp, 1 ))
    return xx, uu

def main():

#########################################################
#########################################################
    # set constants

    x0_0 = [0.5, 2.66390e-03, 3.16542e-02, -1.45239e-02, 1.85883e-01, -4.05752e-02]  # [vx vy psidot y_e thetae theta s x y]

    map = Map()
    states =  predicted_vectors_generation_V2(10, x0_0,0 , 0.1)[0][0,:]
    disp = plotter(map,2)

    states_old = np.array([1.0, 0,0,0,0,0] )

    controls = [i.split() for i in open("/home/marc/Escritorio/signals/control.dat").readlines()]
    u_hist = np.empty((len(controls),2))

    poses = [i.split() for i in open("/home/marc/Escritorio/signals/states.dat").readlines()]
    pose_hist = np.empty((len(poses), 6))
    states_hist = np.empty((len(poses), 6))

    for k,control in enumerate(controls):
        u_hist[k,:] = float(control[0]), float(control[1])

    for k,pose in enumerate(poses):
        pose_hist[k,:] = float(pose[0]), float(pose[1]),float(pose[2]),float(pose[3]), float(pose[4]),float(pose[5])

    states_old = pose_hist[20,:]
    states_hist[0,:] = states_old
    # states_old = np.array([1.0489,         0,         0,         0,         0, 0])
    for t,u in enumerate(u_hist[20:,:]):
        j += 20
        # states_old = pose_hist[t, :]
        states_hist[j,:] = states
        states, A, B = OneStepSim(map, states_old, u)
        print(states)
        print(pose_hist[j+1, :])
        pose = np.array(map.getGlobalPosition(states[4],states[5])).flatten()
        disp.plot_step(pose[0],pose[1], pose[2])
        states_old = states
        # disp.plot_step(pose_hist[t,0], pose_hist[t,1], pose_hist[t,2 ])


    # plt.ion()
    # i = 0
    # fig, ax = plt.subplots()
    # ax.plot(pose_hist[:,i])  # Plot some data on the axes.
    # ax.plot(states_hist[:,i])  #
    # ax.set_title("state" + str(i))  # Add a title to the axes.
    # plt.show()
    # input()

if __name__ == "__main__":

    main()




