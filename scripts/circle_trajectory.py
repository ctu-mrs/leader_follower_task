#!/usr/bin/python3

import numpy as np
import math 
from scipy.interpolate import splprep, splev
from os.path import expanduser


def point(A,B,h,k,t):

    x = h + A * np.cos(t)
    y = k + B * np.sin(t)
    
    return x,y 


def generate_trajectory():
    A = 10
    B = 10
    h = 0
    k = 0
    t = np.linspace(0, 2 * np.pi, 300)
    X = []
    Y = []
   
    for i in range(0, len(t)):
        x, y = point(A,B,h,k,t[i])
        X.append(x)
        Y.append(y)
    
    pts = np.array([X,Y])

    """
    v = 1.0
    dt = 0.2

    ds = v*dt

    # interpolate points
    tck, u = splprep(pts, u=None, s=0.0, per=1)
    # u_new = np.linspace(u.min(), u.max(), 1000)
    u_new = np.linspace(u.min(), u.max(), 1000)
    x_new, y_new = splev(u_new, tck, der=0)


    # calculate trajectory length
    trajectory_length = 0;
    for i in range(1, len(x_new)):
        trajectory_length = trajectory_length + math.sqrt((x_new[i]-x_new[i-1])**2 + (y_new[i]-y_new[i-1])**2)

    # resample to match trajectory expectation of MRS MPC tracker
    num_samples = int(trajectory_length / ds);
    u_resampled = np.linspace(u.min(), u.max(), num_samples)
    x_resampled, y_resampled= splev(u_resampled, tck, der=0)
    """
    yaw_values = []
    for i in range (1, len(X)):
        dx = X[i] - X[i-1]
        dy = Y[i] - Y[i-1]
        yaw = math.atan2(dy,dx)
        yaw_values.append(yaw)

    home = expanduser("~")
    fname = home + '/f4f_ws/src/leader_follower_task/trajectories/leader_circle' + '.txt'
    with open(fname, 'w', encoding='utf-8') as f:
        f.write(str(X[0]) + ', ' + str(Y[0]) + ', ' + str(3.0) + ', ' + '1.5' + '\n')
        for i in range(1, len(X)):
            x = X[i];
            y = Y[i];
            z = 3.0;
            yaw = yaw_values[i-1];

            line = str(x) + ', ' + str(y) + ', ' + str(z) + ', ' + str(yaw) + '\n'
            f.write(line)   


if __name__ == '__main__':
    generate_trajectory();

