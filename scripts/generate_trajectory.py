#!/usr/bin/python3

import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import math
from os.path import expanduser


# #{ generate_spline
def generate_spline():

    # #{ define pts
    pts = np.array([
             [ 16, 0 ],
             [ 15, 0 ],
             [ 5, 0 ],
             [ 2, -1 ],
             [ -8, 0 ],
             [ -14, -1 ],
             [ -14, -8 ],
             [ -15, -8 ],
             [ -12, -16 ],
             [ -10, -15 ],
             [ -4, -10 ],
             [ -4, -5 ],
             [ 0, 0 ],
             [ 5, -4 ],
             [ 8, -10 ],
             [ 12, -10 ],
             [ 9, -6 ],
             [ 8, -7 ],
             [ 13, -16 ],
             [ 14, -10 ],
             [ 15, -3 ],
             [ 19, 3 ],
             [ 17, 10 ],
             [ 17, 13 ],
             [ 16, 14 ],
             [ 17, 15  ],
             [ 16, 2 ]])
# #}

    v = 0.5
    dt = 0.2

    ds = v*dt

    # interpolate points
    tck, u = splprep(pts.T, u=None, s=0.0, per=1)
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

    yaw_values = []
    for i in range (1, len(x_resampled)):
        dx = x_resampled[i] - x_resampled[i-1]
        dy = y_resampled[i] - y_resampled[i-1]
        yaw = math.atan2(dy,dx)
        yaw_values.append(yaw)


    home = expanduser("~")
    fname = home + '/f4f_ws/src/leader_follower_task/trajectories/leader_' + str(v) + 'ms.txt'
    with open(fname, 'w', encoding='utf-8') as f:
        f.write(str(x_resampled[0]) + ', ' + str(y_resampled[0]) + ', ' + str (3.0) + ', ' + str(3.14) + '\n')
        for i in range(1, len(x_resampled)):
            x = x_resampled[i];
            y = y_resampled[i];
            z = 3.0;
            yaw = yaw_values[i-1];

            line = str(x) + ', ' + str(y) + ', ' + str(z) + ', ' + str(yaw) + '\n'
            f.write(line)
# #}

if __name__ == '__main__':
    generate_spline();

