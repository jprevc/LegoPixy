# -*- coding: utf-8 -*-
"""
Created on Fri Dec 30 08:44:57 2016

@author: jostp
"""

import matplotlib.pyplot as plt
import numpy as np

from legopixy.constants import (
    PIXY_SIGNATURE_HEIGHT,
    PIXY_SIGNATURE_ROTATION,
    PIXY_SIGNATURE_WIDTH,
)
from legopixy.lego_kinematics import robot_compute_new_pose, robot_external_kinematics
from legopixy.lego_pixy_communication import receive_data
from legopixy.lego_sim_functions import draw_rectangle, draw_robot

ts = 0.1  # sample time in s
t_fin = 10  # Time of simulation

t_vec = np.arange(0, t_fin, ts)
num_of_samples = len(t_vec)

fig1 = plt.figure(1)
plt.xlim(-500, 1000)
plt.ylim(-500, 1000)
plt.grid("on")
robot_pose = np.zeros(
    (num_of_samples, 3)
)  # n×3 matrix of robot pose, first two columns are x and y position in mm,
# 3rd column is robot orientation in deg
# robot initial position and orientation
robot_pose[0, :] = [600, 400, 0]

# for i in range(1, num_of_samples):
#
#    # draw current robot position
#    plt.cla() # clear figure
#    draw_robot(fig1, robot_pose[i-1,:])
#    plt.pause(0.05)
#
#    # TODO: get real motor DC values from robot
#
#    # get current speed of robot according to DC values
#    cur_speed = robot_external_kinematics((20,30), robot_pose[i-1,2])
#
#    # compute new robot pose
#    robot_pose[i,:] = robot_compute_new_pose(robot_pose[i-1,:], cur_speed, ts)
#


###############################################################################
#############  pixy detect color and position in simulation  ##################
###############################################################################

fig1 = plt.figure(1)
plt.xlim(-1000, 1000)
plt.ylim(-1000, 1000)
plt.grid("on")
robot_pose = np.zeros(
    (num_of_samples, 3)
)  # n×3 matrix of robot pose, first two columns are x and y position in mm,
# 3rd column is robot orientation in deg

i = 2
# robot initial position and orientation
robot_pose[0, :] = [600, 400, 0]


while 1:

    lego_parameters = receive_data("a4:db:30:56:59:71", 4)

    print(lego_parameters[0, 4])

    # draw current robot position
    plt.cla()  # clear figure
    if lego_parameters[0, 0] == 1:
        draw_rectangle(
            fig1,
            (lego_parameters[0, 3], lego_parameters[0, 4]),
            PIXY_SIGNATURE_WIDTH,
            PIXY_SIGNATURE_HEIGHT,
            PIXY_SIGNATURE_ROTATION,
            "blue",
        )
    elif lego_parameters[0, 0] == 2:
        draw_rectangle(
            fig1,
            (lego_parameters[0, 3], lego_parameters[0, 4]),
            PIXY_SIGNATURE_WIDTH,
            PIXY_SIGNATURE_HEIGHT,
            PIXY_SIGNATURE_ROTATION,
            "red",
        )
    elif lego_parameters[0, 0] == 3:
        draw_rectangle(
            fig1,
            (lego_parameters[0, 3], lego_parameters[0, 4]),
            PIXY_SIGNATURE_WIDTH,
            PIXY_SIGNATURE_HEIGHT,
            PIXY_SIGNATURE_ROTATION,
            "green",
        )
    elif lego_parameters[0, 0] == 4:
        draw_rectangle(
            fig1,
            (lego_parameters[0, 3], lego_parameters[0, 4]),
            PIXY_SIGNATURE_WIDTH,
            PIXY_SIGNATURE_HEIGHT,
            PIXY_SIGNATURE_ROTATION,
            "yellow",
        )

    draw_robot(fig1, robot_pose[i - 1, :])
    plt.pause(0.005)

    # get current speed of robot according to DC values
    dc_l = lego_parameters[0, 1]
    dc_d = lego_parameters[0, 2]
    cur_speed = robot_external_kinematics((dc_l, dc_d), robot_pose[i - 1, 2])

    # compute new robot pose
    robot_pose[i, :] = robot_compute_new_pose(robot_pose[i - 1, :], cur_speed, ts)
