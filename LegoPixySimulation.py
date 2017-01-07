# -*- coding: utf-8 -*-
"""
Created on Fri Dec 30 08:44:57 2016

@author: jostp
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl

from LegoSimFunctions import *
from LegoKinematics import *
from LegoPixyCommunication import *

Ts = 0.1  # sample time in s
Tfin = 10 # Time of simulation

t_vec = np.arange(0, Tfin, Ts)
NumOfSamples = len(t_vec)

fig1 = plt.figure(1)
plt.xlim(-500, 1000)
plt.ylim(-500, 1000)
plt.grid('on')
robotPose = np.zeros((NumOfSamples,3)) # n×3 matrix of robot pose, first two columns are x and y position in mm,
                                       # 3rd column is robot orientation in deg
# robot initial position and orientation
robotPose[0,:] = [600,400,0]

#for i in range(1,NumOfSamples):
#    
#    # draw current robot position
#    plt.cla() # clear figure
#    drawRobot(fig1, robotPose[i-1,:])
#    plt.pause(0.05)
#    
#    # TODO: get real motor DC values from robot
#    
#    # get current speed of robot according to DC values    
#    curSpeed = robotExternalKinematics((20,30), robotPose[i-1,2])
#    
#    # compute new robot pose
#    robotPose[i,:] = robotComputeNewPose(robotPose[i-1,:], curSpeed, Ts)
#    

    
###############################################################################
#############  pixy detect color and position in simulation  ##################
###############################################################################

fig1 = plt.figure(1)
plt.xlim(-1000, 1000)
plt.ylim(-1000, 1000)
plt.grid('on')
robotPose = np.zeros((NumOfSamples,3)) # n×3 matrix of robot pose, first two columns are x and y position in mm,
                                       # 3rd column is robot orientation in deg

i = 2
# robot initial position and orientation
robotPose[0,:] = [600,400,0]

    
while(1):

    LegoPar = recieveData('a4:db:30:56:59:71', 4)    
    
    print(LegoPar[0,4])
    
    # draw current robot position
    plt.cla() # clear figure
    if LegoPar[0,0] == 1:
        drawRectangle(fig1, (LegoPar[0,3], LegoPar[0,4]), 64, 32, 90, "blue")
    elif LegoPar[0,0] == 2:
        drawRectangle(fig1, (LegoPar[0,3], LegoPar[0,4]), 64, 32, 90, "red")
    elif LegoPar[0,0] == 3:
        drawRectangle(fig1, (LegoPar[0,3], LegoPar[0,4]), 64, 32, 90, "green")
    elif LegoPar[0,0] == 4:
        drawRectangle(fig1, (LegoPar[0,3], LegoPar[0,4]), 64, 32, 90, "yellow")
        
    
    drawRobot(fig1, robotPose[i-1,:])
    plt.pause(0.005)

    # get current speed of robot according to DC values    
    dc_l = LegoPar[0,1]
    dc_d = LegoPar[0,2]
    curSpeed = robotExternalKinematics((dc_l, dc_d), robotPose[i-1,2])

    
    # compute new robot pose
    robotPose[i,:] = robotComputeNewPose(robotPose[i-1,:], curSpeed, Ts)
