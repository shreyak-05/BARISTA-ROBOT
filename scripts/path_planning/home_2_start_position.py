from sympy import * 
import numpy as np
import matplotlib.pyplot as plt

# Goal Create velocity trajectory for end effector to move form home position to start position

# In Home Position, Moving Joint 1 and Joint 6 by pi/2 in opposite directions in 6 seconds

total_time = 6

steps = total_time * 100       # 600 steps
step = total_time/steps        # 0.01s per step

inital_j1_dot = np.zeros(steps)
inital_j6_dot = np.zeros(steps)

home_ramp_up_vert = int(steps * 0.1)
home_ramp_down_vert = int(steps * 0.9)


for i in range(0, steps):

    if i < home_ramp_up_vert:
        home_vert_z_dot[i] = home_vert_z_dot[i-1] - (0.0513/home_ramp_up_vert) * 2

    if i >= home_ramp_up_vert and i < home_ramp_down_vert:
        home_vert_z_dot[i] = -0.0512 * 2

    if i >= home_ramp_down_vert and i < steps:
        home_vert_z_dot[i] = home_vert_z_dot[i-1] + ((0.0506/home_ramp_up_vert)) * 2
    
    if i >= steps:
        home_vert_z_dot[i] = 0