#!/usr/bin/env python27

import matplotlib.pyplot as plt
import numpy as np

plot = True
it = True

def plot_hyperplanes(planes, pointX, pointY):
    plt.ion()
    plt.clf()
    plt.xlim([-1.3, 1.3])
    plt.ylim([-2, 2])
    ego = pointX
    neighbour = pointY

    # Define hyperplane equation
    a = planes[0][0]
    b = planes[0][1]
    c = -planes[1]

    color = np.random.rand(1)
    # Generate random data points
    print(a*pointX[0] + b*pointX[1] + c)
    print(a*pointY[0] + b*pointY[1] + c)
    print(np.sqrt( (pointX[0] - pointY[0])**2 + (pointX[1] - pointY[1])**2)/2)
    print((abs(a*pointX[0] + b*pointX[1] + c)) / np.sqrt(a ** 2 + b ** 2))

    print("---------------------")

    if not np.isclose(a, 0):
        y = np.arange(-1,1,0.1)
        x_hyperplane = (-b * y - c) / a
        plt.plot(x_hyperplane, y, color)

    elif not np.isclose(b, 0):
        x = np.arange(-1,1,0.1)
        y_hyperplane = (-a * x - c) / b
        plt.plot(x, y_hyperplane, color)

    plt.scatter(ego[0], ego[1], c=color,marker="o")
    plt.scatter(neighbour[0], neighbour[1], c=color,marker="o")
    plt.show()
    plt.pause(0.001)
    plt.waitforbuttonpress()


def generate_point_pairs(radius = 1, number = 6):

    # angles = np.linspace(0, 2*np.pi, number)
    angles = np.asarray([0, np.pi,np.pi/2, -np.pi/2 ])
    y = radius * np.sin(angles)
    x = radius * np.cos(angles)

    y = [1,1.45]
    x = [0,0]
    return np.asarray([x,y]).T

def compute_hyperplane(agent_X0, agent_X1):
    a =  agent_X0 - agent_X1
    a = a/np.sqrt(a[0]**2 + a[1]**2)
    b = 0.5 * a@(agent_X0 + agent_X1).T
    return a,b

def main():

#########################################################
#########################################################
    # set constants

    points = generate_point_pairs(0.5,6)

    for i, pointX in enumerate(points):
        for j, pointY in enumerate(points):

            if j != i:
                resu = compute_hyperplane(pointX,pointY)
                plot_hyperplanes(resu,pointX,pointY)

if __name__ == "__main__":

    main()




